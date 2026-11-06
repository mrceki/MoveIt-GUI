"""Cartesian speed limit (mm/s) for linear moves (MoveL): OPERATE > Pose and the program's linear motion steps.

Jazzy's GetCartesianPath.srv and MotionPlanRequest carry `max_cartesian_speed` / `cartesian_speed_limited_link`,
but move_group does not use them for Cartesian paths (MoveIt 2.12 in the image and 2.13 main: the
compute_cartesian_path capability only runs TOTG at the velocity / acceleration scaling; its own "\\todo" says so).
Setting them would change nothing. They also only accept a URDF link, not a virtual TCP.

So the GUI limits the speed itself: it computes the speed of the TCP POINT along the returned trajectory with
forward kinematics from the URDF (program_path.fk) and, when it is faster than the limit, slows the whole
trajectory down uniformly (time x k, joint velocities / k, accelerations / k^2). A uniform time scaling keeps the
path and the trajectory's smoothness exactly and only makes it slower, so joint and acceleration limits still
hold. The limit is a maximum: the TCP never goes faster, and the Speed % (joint velocity scaling) can make it slower.
"""
from __future__ import annotations

import math
from typing import Callable, Optional, Sequence

import numpy as np

DEFAULT_MAX_MM_S = 250.0
# steps whose linear moves take speed_mm_s (move_pose only when cartesian, move_relative unless cartesian: false)
SPEED_STEPS = ("move_pose", "move_relative", "move_until_contact")


def max_mm_s(profile) -> float:
    """The profile's upper limit (moveit.max_cartesian_speed_mm_s), default 250 mm/s."""
    try:
        v = float((getattr(profile, "moveit", None) or {}).get("max_cartesian_speed_mm_s", DEFAULT_MAX_MM_S))
    except (TypeError, ValueError):
        return DEFAULT_MAX_MM_S
    return v if v > 0 and math.isfinite(v) else DEFAULT_MAX_MM_S


def check_mm_s(v, limit: float) -> float:
    """A speed_mm_s value: a number in (0, limit]. Raises ValueError with a short sentence."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"speed must be a number of mm/s, got {v!r}")
    v = float(v)
    if not (math.isfinite(v) and 0.0 < v <= limit):
        raise ValueError(f"speed must be in (0, {limit:g}] mm/s")
    return v


def optional_mm_s(v, limit: float) -> float:
    """What the OPERATE page sends: empty / None / 0 = no limit (0.0), else check_mm_s."""
    if v is None or v == "":
        return 0.0
    if isinstance(v, str):
        try:
            v = float(v)
        except ValueError:
            raise ValueError(f"speed must be a number of mm/s, got {v!r}") from None
    if not isinstance(v, bool) and isinstance(v, (int, float)) and float(v) == 0.0:
        return 0.0
    return check_mm_s(v, limit)


def _secs(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _set_secs(d, t: float):
    ns = int(round(t * 1e9))
    d.sec, d.nanosec = divmod(ns, 1_000_000_000)


PointAt = Callable[[dict], np.ndarray]      # {joint: value} -> TCP point position in the base frame (m)


def peak_speed(traj, point_at: PointAt) -> float:
    """Highest speed (m/s) of the TCP point along a JointTrajectory: per segment (distance / time) and at each
    point from its joint velocities (a small step along qd), so a long segment does not hide a peak."""
    names = list(traj.joint_names)
    pts = list(traj.points)
    P = [np.asarray(point_at(dict(zip(names, p.positions))), float) for p in pts]
    peak = 0.0
    for i in range(1, len(pts)):
        dt = _secs(pts[i].time_from_start) - _secs(pts[i - 1].time_from_start)
        if dt > 1e-9:
            peak = max(peak, float(np.linalg.norm(P[i] - P[i - 1])) / dt)
    h = 1e-3
    for p, Pi in zip(pts, P):
        qd = list(getattr(p, "velocities", None) or [])
        if len(qd) == len(names):
            q2 = {n: float(q) + float(v) * h for n, q, v in zip(names, p.positions, qd)}
            peak = max(peak, float(np.linalg.norm(np.asarray(point_at(q2), float) - Pi)) / h)
    return peak


def limit_speed(traj, point_at: PointAt, v_max: float) -> float:
    """Slow `traj` down in place (uniform time scaling) so the TCP point stays at or below v_max (m/s).
    Returns the factor k >= 1 the duration grew by (1.0 = it was slow enough)."""
    if v_max <= 0 or len(traj.points) < 2:
        return 1.0
    peak = peak_speed(traj, point_at)
    if peak <= v_max:
        return 1.0
    k = peak / v_max
    for p in traj.points:
        _set_secs(p.time_from_start, _secs(p.time_from_start) * k)
        if getattr(p, "velocities", None):
            p.velocities = [float(v) / k for v in p.velocities]
        if getattr(p, "accelerations", None):
            p.accelerations = [float(a) / (k * k) for a in p.accelerations]
    return k


def point_fk(model: dict, base: str, link: str, point: Optional[Sequence[float]] = None) -> Optional[PointAt]:
    """point_at for a point fixed in `link` (xyz in m, None = the link origin), from a parsed URDF; None when the
    link is not below `base` in the model."""
    from .program_path import fk
    if fk(model, {}, base, link) is None:
        return None
    p = np.r_[np.asarray(point if point is not None else [0.0, 0.0, 0.0], float), 1.0]
    return lambda q: (fk(model, q, base, link) @ p)[:3]
