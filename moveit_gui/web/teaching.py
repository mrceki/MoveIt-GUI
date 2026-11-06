"""PROGRAMS > Teach & more > Teach by hand (WebService mixin). The maths is in moveit_gui/teach.py.

Teaching only reads where the arm is and edits the open program: it never moves the robot (freedrive, or jogging,
is the operator's hands). It needs control like every program edit, and ends when control changes hands. It is
refused while a program, batch or policy runs, and a program run is refused while teaching.
"""
from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from .. import cart_speed
from .. import transforms as tf
from ..geometry import PoseSpec
from ..program import ProgramWaypoint, RunState, Step, path_label, walk
from ..teach import Sampler, StillDetector, TaughtPath, TeachError, check_tolerances, suggested_speed

MOVES = {"joints": "MoveJ", "pose": "MoveJ to pose", "linear": "MoveL"}
MAX_PREVIEW_POINTS = 500


class TeachCommands:
    """Needs from WebService: core, program, lease, _edit_lock, _snapshot, _record_edit, _editable, _insert_step,
    _path, _reply, _ok and _err (a CommandError)."""

    def _teach_init(self):
        self.teach: Optional[dict] = None

    def _teach_refusal(self) -> Optional[str]:
        c = self.core
        if c.runner.state != RunState.IDLE or c.batch.active:
            return "a program is running: stop it first"
        if c.policies.busy:
            return "a policy is running: stop it first"
        return None

    def _teach_owner(self) -> str:
        return str(self.lease.uid or self.lease.token or "")

    def _teach_session(self, mode: Optional[str] = None) -> dict:
        s = self.teach
        if s is None:
            raise self._err("not teaching: press Record first")
        if mode and s["mode"] != mode:
            raise self._err(f"this is {s['mode']} teaching")
        return s

    def _taught_name(self, taken=()) -> str:
        k = 1
        while f"taught_{k:03d}" in self.program.waypoints or f"taught_{k:03d}" in taken:
            k += 1
        return f"taught_{k:03d}"

    # ---------------------------------------------------------------- commands
    def cmd_teach_start(self, mode="waypoints", move="linear", frame=None, auto=False, path=None, where="after",
                        pos_tol_mm=1.0, rot_tol_deg=2.0):
        """Begin teaching. waypoints: Mark adds a waypoint + a move step (move: joints | pose | linear), auto=True
        also marks when the arm stood still ~1 s after it moved. path: samples the TCP until Finish."""
        if self.teach is not None:
            raise self._err("already teaching: finish or cancel first")
        why = self._teach_refusal()
        if why:
            raise self._err(why)
        self._editable()
        if mode not in ("waypoints", "path"):
            raise self._err("mode is waypoints or path")
        if move not in MOVES:
            raise self._err("move is joints, pose or linear")
        c = self.core
        frame = str(frame or c.profile.base_link)
        c.geo.T_base_frame(frame)                        # an unknown / untaught frame is refused now (GeometryError)
        tcp = c.store.active_tcp
        T0 = c.geo.current_T_base_tcp(tcp)
        try:
            pos_tol_mm, rot_tol_deg = check_tolerances(pos_tol_mm, rot_tol_deg)
        except TeachError as e:
            raise self._err(str(e)) from None
        start = None
        if path not in (None, []):
            start = self._path(path)
        s = {"mode": mode, "move": move if mode == "waypoints" else "linear", "frame": frame, "tcp": tcp,
             "auto": bool(auto) and mode == "waypoints", "owner": self._teach_owner(), "by": self.lease.name,
             "t0": time.monotonic(), "start": start, "where": where if where in ("after", "before", "into") else "after",
             "anchor": None, "marks": 0, "names": [], "samples": [], "phase": "recording", "full": False,
             "error": "", "pos_tol_mm": pos_tol_mm, "rot_tol_deg": rot_tol_deg, "taught": None, "simp": None,
             "sampler": None, "detector": None}
        if mode == "path":
            s["sampler"] = Sampler(lambda: c.geo.current_T_base_tcp(tcp), lambda t, T: s["samples"].append((t, T)),
                                   on_full=lambda: s.update(full=True))
        elif s["auto"]:
            det = StillDetector()
            det.reset(T0)
            s["detector"] = det
            s["sampler"] = Sampler(lambda: c.geo.current_T_base_tcp(tcp),
                                   lambda t, T: det.feed(t, T) and self._teach_auto_mark(s), hz=10.0,
                                   max_samples=10 ** 9)
        self.teach = s
        if s["sampler"]:
            s["sampler"].start()
        what = ("path: move the arm, then Finish" if mode == "path" else
                f"waypoints ({MOVES[s['move']]}): move the arm, then Mark" + (" (or stand still 1 s)" if s["auto"] else ""))
        c.bridge.log(f"teach: {self.lease.name or 'operator'} started teaching {what}, {tcp} in {frame}")
        return self._ok(f"teaching {what}", teach=self.teach_view())

    def _teach_mark(self, s: dict):
        """One waypoint + move step at the arm's position now. The caller holds the edit lock."""
        c = self.core
        name = self._taught_name()
        if s["move"] == "joints":
            snap = c.last_snap
            if not snap.joints or snap.joint_state_age is None or snap.joint_state_age > c.profile.joint_state_timeout_s:
                raise self._err("no fresh joint state to mark")
            self.program.waypoints[name] = ProgramWaypoint(name, joints=list(snap.joints))
            st = Step("movej", {"target": name, "velocity": 0.1})
        else:
            pose = c.geo.current_pose(s["frame"], s["tcp"])
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
            st = Step("move_pose", {"target": name, "cartesian": s["move"] == "linear", "velocity": 0.1})
        path = self._teach_place(st, s)
        s["anchor"], s["marks"] = st, s["marks"] + 1
        s["names"].append(name)
        if s["detector"] is not None:
            try:
                s["detector"].reset(c.geo.current_T_base_tcp(s["tcp"]))
            except Exception:                            # noqa: BLE001 - the next sample sets it
                pass
        return name, path

    def _teach_place(self, st: Step, s: dict):
        """After the last taught step (wherever it is now), else where the selection was at the start, else the end."""
        anchor = s["anchor"]
        at = next((p for p, x in walk(self.program.steps) if x is anchor), None) if anchor is not None else None
        if at is not None:
            return self._insert_step(st, at, "after")
        start, s["start"] = s["start"], None
        if start is not None:
            try:
                self.program.step_at(start)
                return self._insert_step(st, start, s["where"])
            except Exception:                            # noqa: BLE001 - the program changed meanwhile: at the end
                pass
        return self._insert_step(st, None, "after")

    def cmd_teach_mark(self):
        s = self._teach_session("waypoints")
        self._editable()
        name, path = self._teach_mark(s)
        return self._reply(f"marked {name} ({MOVES[s['move']]})", path)

    def _teach_auto_mark(self, s: dict):
        """From the sampler thread: the arm stood still. Same edit as Mark, with undo and a new revision."""
        if self.teach is not s or self._teach_owner() != s["owner"] or self._teach_refusal():
            return
        try:
            with self._edit_lock:
                if self.teach is not s:
                    return
                before = self._snapshot()
                name, _ = self._teach_mark(s)
                self._record_edit(before)
            self.core.bridge.log(f"teach: marked {name} (the arm stood still)")
        except Exception as e:                           # noqa: BLE001 - shown in the teach panel
            s["error"] = str(e)

    def cmd_teach_finish(self):
        s = self._teach_session()
        if s["sampler"]:
            s["sampler"].stop()
        if s["mode"] == "waypoints":
            self._teach_end(f"taught {s['marks']} waypoint{'s' * (s['marks'] != 1)}")
            return self._ok(f"taught {s['marks']} waypoint{'s' * (s['marks'] != 1)}", teach=None)
        if s["phase"] == "recording":
            try:
                s["taught"] = TaughtPath(list(s["samples"]))
            except TeachError as e:
                self._teach_end("path too short")
                raise self._err(str(e)) from None
            s["phase"] = "review"
            s["simp"] = s["taught"].simplify(s["pos_tol_mm"], s["rot_tol_deg"])
        v = self.teach_view()
        r = v["result"]
        return self._ok(f"{r['points']} points from {r['samples']} samples, max {r['max_mm']:.1f} mm / "
                        f"{r['max_deg']:.1f}°", teach=v)

    def cmd_teach_simplify(self, pos_tol_mm=1.0, rot_tol_deg=2.0):
        s = self._teach_session("path")
        if s["phase"] != "review":
            raise self._err("finish the recording first")
        try:
            s["simp"] = s["taught"].simplify(pos_tol_mm, rot_tol_deg)
        except TeachError as e:
            raise self._err(str(e)) from None
        s["pos_tol_mm"], s["rot_tol_deg"] = float(pos_tol_mm), float(rot_tol_deg)
        return self._ok(f"{len(s['simp'].keep)} points", teach=self.teach_view())

    def cmd_teach_insert(self, pos_tol_mm=None, rot_tol_deg=None, speed_mm_s=None, path=None, where="after"):
        """Put the simplified path into the program: a folder with a MoveJ to pose to its start, then MoveL steps
        (optional Max mm/s), waypoints taught_NNN in the teaching frame."""
        s = self._teach_session("path")
        if s["phase"] != "review":
            raise self._err("finish the recording first")
        self._editable()
        c = self.core
        if pos_tol_mm is not None or rot_tol_deg is not None:
            try:
                s["simp"] = s["taught"].simplify(pos_tol_mm if pos_tol_mm is not None else s["pos_tol_mm"],
                                                 rot_tol_deg if rot_tol_deg is not None else s["rot_tol_deg"])
            except TeachError as e:
                raise self._err(str(e)) from None
        try:
            mm_s = cart_speed.optional_mm_s(speed_mm_s, c.profile.max_cart_speed_mm_s)
        except ValueError as e:
            raise self._err(str(e)) from None
        T_frame_base = tf.inv(c.geo.T_base_frame(s["frame"]))
        names: List[str] = []
        kids = []
        for n, k in enumerate(s["simp"].keep):
            name = self._taught_name(names)
            names.append(name)
            pose = PoseSpec.from_T(s["frame"], s["tcp"], T_frame_base @ s["taught"].T[k])
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
            p = {"target": name, "cartesian": n > 0, "velocity": 0.1}
            if n > 0 and mm_s > 0:
                p["speed_mm_s"] = round(mm_s, 1)
            kids.append(Step("move_pose", p))
        folder = Step("folder", {"name": f"Taught path {time.strftime('%H:%M')}"}, kids)
        at = self._insert_step(folder, self._path(path) if path not in (None, []) else None, where)
        n = len(kids)
        self._teach_end(f"inserted {n} points")
        c.bridge.log(f"teach: path inserted as {n} steps at {path_label(at)} (max {s['simp'].max_mm:.1f} mm / "
                     f"{s['simp'].max_deg:.1f}° off the taught path" + (f", {mm_s:g} mm/s" if mm_s else "") + ")")
        return self._reply(f"inserted the path: {n} points in folder {path_label(at)}", at)

    def cmd_teach_cancel(self):
        if self.teach is None:
            return self._ok("not teaching")
        self._teach_end("cancelled")
        return self._ok("teaching cancelled", teach=None)

    # -------------------------------------------------------------- internals
    def _teach_end(self, why: str):
        s, self.teach = self.teach, None
        if s and s["sampler"]:
            s["sampler"].stop()
        if s:
            self.core.bridge.log(f"teach: ended ({why})")

    def _teach_tick(self):
        """Control changed hands, or a program started: teaching ends (marks already made stay)."""
        s = self.teach
        if s is None:
            return
        if self._teach_owner() != s["owner"]:
            self._teach_end("control changed hands")
        elif s["phase"] == "recording" and self._teach_refusal():
            self._teach_end(self._teach_refusal())

    def teach_view(self) -> Optional[dict]:
        s = self.teach
        if s is None:
            return None
        sp = s["sampler"]
        v = {"mode": s["mode"], "move": s["move"], "move_label": MOVES[s["move"]], "frame": s["frame"],
             "tcp": s["tcp"], "auto": s["auto"], "by": s["by"], "phase": s["phase"],
             "elapsed_s": round(time.monotonic() - s["t0"], 1), "samples": len(s["samples"]),
             "missed": sp.missed if sp else 0, "marks": s["marks"], "last": s["names"][-1] if s["names"] else "",
             "full": s["full"], "error": s["error"] or (sp.last_error if sp and sp.missed else ""),
             "pos_tol_mm": s["pos_tol_mm"], "rot_tol_deg": s["rot_tol_deg"], "result": None}
        if s["phase"] == "review" and s["simp"] is not None:
            tp, sm = s["taught"], s["simp"]
            st = tp.stats()
            step = max(1, int(np.ceil(len(tp) / MAX_PREVIEW_POINTS)))
            raw = [[round(float(x) * 1000.0, 1) for x in p] for p in tp.P[::step]]
            raw.append([round(float(x) * 1000.0, 1) for x in tp.P[-1]])
            kept = []
            for k in sm.keep:
                xyz, rpy = tf.T_to_mm_deg(tp.T[k])
                kept.append({"xyz_mm": [round(x, 1) for x in xyz], "rpy_deg": [round(x, 1) for x in rpy]})
            v["result"] = {"samples": len(tp), "points": len(sm.keep), "max_mm": round(sm.max_mm, 2),
                           "max_deg": round(sm.max_deg, 2), **st,
                           "speed_mm_s": suggested_speed(st["avg_mm_s"], self.core.profile.max_cart_speed_mm_s),
                           "raw_mm": raw, "kept": kept}
        return v
