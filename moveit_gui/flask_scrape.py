"""Pure numpy pose generation for scraping inside a round-bottom flask."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from . import transforms as tf
from .flask_geometry import FlaskGeometry, T_base_flask
from .geometry import PoseSpec
from .program import Program, ProgramWaypoint, Step


class FlaskScrapeError(ValueError):
    pass


@dataclass(frozen=True)
class StraightTool:
    tcp: str
    shaft_radius: float
    shaft_length: float

    def __post_init__(self):
        if not self.tcp:
            raise FlaskScrapeError("tool TCP name must not be empty")
        if not math.isfinite(self.shaft_radius) or self.shaft_radius <= 0.0:
            raise FlaskScrapeError("shaft radius must be positive")
        if not math.isfinite(self.shaft_length) or self.shaft_length <= 0.0:
            raise FlaskScrapeError("shaft length must be positive")


ToolModel = StraightTool


@dataclass
class ScrapePose:
    name: str
    ring: int
    theta_deg: float
    azimuth_deg: float
    pose: PoseSpec


@dataclass
class UnreachablePose:
    ring: int
    theta_deg: float
    azimuth_deg: float
    reason: str
    pose: Optional[PoseSpec] = None


@dataclass
class ScrapePlan:
    reachable: list[ScrapePose]
    unreachable: list[UnreachablePose]
    pivot: np.ndarray
    geometry: FlaskGeometry
    tool: StraightTool
    margin: float
    press_depth: float
    reference_rotation: np.ndarray = field(repr=False)

    @property
    def poses(self) -> list[ScrapePose]:
        return self.reachable


@dataclass
class FlaskScrape:
    geometry: FlaskGeometry
    tool: StraightTool
    theta_min_deg: float
    theta_max_deg: float
    rings: int
    step_deg: float
    press_depth_mm: float = 0.0
    margin_mm: float = 1.0
    pivot_depth_m: Optional[float] = None
    reference_orientation: Optional[np.ndarray] = None

    def generate(self) -> ScrapePlan:
        return generate_scrape_poses(
            self.geometry, self.tool, self.theta_min_deg, self.theta_max_deg,
            self.rings, self.step_deg, press_depth_mm=self.press_depth_mm,
            margin_m=self.margin_mm / 1000.0, pivot_depth_m=self.pivot_depth_m,
            reference_orientation=self.reference_orientation,
        )


def _flask_values(g: FlaskGeometry) -> tuple[np.ndarray, np.ndarray, float, float]:
    T = T_base_flask(g)
    rim = T[:3, :3].T @ (g.rim_center - g.sphere_center)
    join = g.neck_join_height
    return T, rim, join, float(rim[2])


def pivot_point(geometry: FlaskGeometry, pivot_depth_m: Optional[float] = None) -> np.ndarray:
    """Pivot in the ``flask`` frame; depth is measured down from the rim."""
    _, rim, join, rim_z = _flask_values(geometry)
    z = join if pivot_depth_m is None else rim_z - float(pivot_depth_m)
    if not math.isfinite(z):
        raise FlaskScrapeError("pivot depth must be finite")
    return np.array([rim[0], rim[1], z], dtype=float)


def _rotation_value(value: Optional[np.ndarray]) -> np.ndarray:
    if value is None:
        return np.eye(3)
    if isinstance(value, PoseSpec):
        R = value.T()[:3, :3]
    else:
        a = np.asarray(value, dtype=float)
        if a.shape == (4, 4):
            R = a[:3, :3]
        elif a.shape == (3, 3):
            R = a
        elif a.shape == (4,):
            R = tf.quat_to_matrix(a)
        else:
            raise FlaskScrapeError("reference orientation must be a pose, rotation matrix, or quaternion")
    if not np.all(np.isfinite(R)) or not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise FlaskScrapeError("reference orientation is not a rotation")
    return np.array(R, dtype=float)


def _minimal_twist(z: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Set tool +Z to ``z`` and retain reference roll as closely as possible."""
    z = np.asarray(z, dtype=float)
    z /= np.linalg.norm(z)
    choices = (reference[:, 0], reference[:, 1], np.array([1.0, 0.0, 0.0]),
               np.array([0.0, 1.0, 0.0]))
    x = None
    for candidate in choices:
        projected = candidate - z * float(candidate @ z)
        if np.linalg.norm(projected) > 1e-7:
            x = projected / np.linalg.norm(projected)
            break
    if x is None:
        raise FlaskScrapeError("cannot construct tool orientation")
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack((x, y, z))


def _shaft_axis(T_flask_tcp: np.ndarray) -> np.ndarray:
    # Tip TCPs in this repository extend along local +Z; the shaft is behind the tip on -Z.
    return T_flask_tcp[:3, 2]


def clearance_reason(T_flask_tcp: np.ndarray, geometry: FlaskGeometry, tool: StraightTool,
                     pivot: Optional[Sequence[float]] = None, margin_m: float = 0.001,
                     press_depth_m: float = 0.0) -> Optional[str]:
    """Return ``None`` when the pose clears the neck and bulb, else a short reason."""
    if margin_m < 0.0 or not math.isfinite(margin_m):
        raise FlaskScrapeError("clearance margin must be non-negative")
    p = np.asarray(T_flask_tcp, dtype=float)[:3, 3]
    u = _shaft_axis(np.asarray(T_flask_tcp, dtype=float))
    u /= np.linalg.norm(u)
    pivot = pivot_point(geometry) if pivot is None else np.asarray(pivot, dtype=float)
    if pivot.shape != (3,):
        raise FlaskScrapeError("pivot must have three values")
    along = float((p - pivot) @ u)
    miss = np.linalg.norm((p - pivot) - along * u)
    if miss > 1e-5 or along <= 0.0:
        return "The shaft line does not pass through the pivot."
    _, rim, _, rim_z = _flask_values(geometry)
    join_z = float(pivot[2])
    if abs(u[2]) < 1e-9:
        return "The shaft cannot pass up through the neck."
    # p - s*u is the shaft centre from tip toward flange.
    s_join = (p[2] - join_z) / u[2]
    s_rim = (p[2] - rim_z) / u[2]
    lo, hi = sorted((s_join, s_rim))
    if lo < -1e-8 or hi > tool.shaft_length + 1e-9:
        return "The shaft is too short to span the neck."
    # An angled cylinder cuts a horizontal neck plane as an ellipse.
    allowed = geometry.neck_inner_radius - margin_m - tool.shaft_radius / abs(float(u[2]))
    if allowed < 0.0:
        return "The shaft is too wide for the neck clearance."
    axis_xy = rim[:2]
    q0 = (p - lo * u)[:2] - axis_xy
    q1 = (p - hi * u)[:2] - axis_xy
    if max(float(np.linalg.norm(q0)), float(np.linalg.norm(q1))) > allowed + 1e-9:
        return "The shaft does not stay inside the neck bore."
    # The first shaft-radius behind the TCP is the rounded tip.  Past it, the
    # cylindrical shaft must remain inside the inner sphere until the opening.
    s0 = max(tool.shaft_radius, 0.0)
    s1 = min(tool.shaft_length, s_join)
    inner_join = math.sqrt(max(0.0, geometry.R_inner ** 2 - geometry.neck_inner_radius ** 2))
    if u[2] < -1e-12:
        s_open = (p[2] - inner_join) / u[2]
        s1 = min(s1, s_open)
    if s1 > s0:
        # A shaft cross-section is a disk normal to u, not a ball.  Its
        # furthest point from the sphere centre has this exact radius.
        parallel = float(p @ u)
        perpendicular = float(np.linalg.norm(p - parallel * u))
        furthest = max(math.hypot(parallel - s, perpendicular + tool.shaft_radius)
                       for s in (s0, s1))
        if furthest > geometry.R_inner + max(0.0, press_depth_m) + 1e-8:
            return "The shaft touches the bulb wall away from the tip."
    return None


check_clearance = clearance_reason


def generate_scrape_poses(geometry: FlaskGeometry, tool: StraightTool,
                          theta_min_deg: float, theta_max_deg: float, rings: int,
                          step_deg: float, *, press_depth_mm: float = 0.0,
                          margin_m: float = 0.001, pivot_depth_m: Optional[float] = None,
                          reference_orientation: Optional[np.ndarray] = None) -> ScrapePlan:
    """Generate alternating rings, separating reachable and rejected poses."""
    if isinstance(rings, bool) or int(rings) != rings or int(rings) < 1:
        raise FlaskScrapeError("number of rings must be a positive whole number")
    rings = int(rings)
    values = [theta_min_deg, theta_max_deg, step_deg, press_depth_mm, margin_m]
    if not all(math.isfinite(float(v)) for v in values):
        raise FlaskScrapeError("scrape dimensions must be finite")
    if not 0.0 <= theta_min_deg <= theta_max_deg <= 180.0:
        raise FlaskScrapeError("polar angles must satisfy 0 <= minimum <= maximum <= 180 degrees")
    if step_deg <= 0.0 or step_deg > 360.0:
        raise FlaskScrapeError("ring step must be in (0, 360] degrees")
    if press_depth_mm < 0.0:
        raise FlaskScrapeError("press depth must be non-negative")
    if margin_m < 0.0:
        raise FlaskScrapeError("clearance margin must be non-negative")
    press = float(press_depth_mm) / 1000.0
    radius = geometry.R_inner + press
    pivot = pivot_point(geometry, pivot_depth_m)
    reference = _rotation_value(reference_orientation)
    thetas = np.linspace(float(theta_min_deg), float(theta_max_deg), rings)
    azimuths = list(np.arange(0.0, 360.0 - 1e-10, float(step_deg)))
    reachable: list[ScrapePose] = []
    unreachable: list[UnreachablePose] = []
    previous = reference
    for ring, theta_deg in enumerate(thetas):
        order = azimuths if ring % 2 == 0 else azimuths[::-1]
        theta = math.radians(float(theta_deg))
        for index, azimuth_deg in enumerate(order):
            phi = math.radians(float(azimuth_deg))
            radial = np.array([math.sin(theta) * math.cos(phi),
                               math.sin(theta) * math.sin(phi), -math.cos(theta)])
            tip = radial * radius
            direction = tip - pivot
            distance = float(np.linalg.norm(direction))
            if distance < 1e-9:
                reason = "The target point coincides with the pivot."
                unreachable.append(UnreachablePose(ring, float(theta_deg), float(azimuth_deg), reason))
                continue
            R = _minimal_twist(direction / distance, previous)
            T = tf.from_rotation(R, tip)
            pose = PoseSpec.from_T("flask", tool.tcp, T)
            reason = clearance_reason(T, geometry, tool, pivot, margin_m, press)
            if reason:
                unreachable.append(UnreachablePose(ring, float(theta_deg), float(azimuth_deg), reason, pose))
                continue
            name = f"ring_{ring + 1:02d}_{index + 1:03d}"
            reachable.append(ScrapePose(name, ring, float(theta_deg), float(azimuth_deg), pose))
            previous = R
    return ScrapePlan(reachable, unreachable, pivot, geometry, tool, margin_m, press, reference)


generate = generate_scrape_poses


def reachable_band(geometry: FlaskGeometry, tool: StraightTool, *, margin_m: float = 0.001,
                   pivot_depth_m: Optional[float] = None, resolution_deg: float = 0.25,
                   azimuth_step_deg: float = 15.0) -> Optional[tuple[float, float]]:
    """Largest polar-angle band for which a complete ring is reachable."""
    if resolution_deg <= 0.0 or azimuth_step_deg <= 0.0:
        raise FlaskScrapeError("band sampling steps must be positive")
    pivot = pivot_point(geometry, pivot_depth_m)
    ok: list[float] = []
    for theta_deg in np.arange(0.0, 180.0 + resolution_deg * 0.5, resolution_deg):
        theta = math.radians(float(theta_deg))
        ring_ok = True
        for phi_deg in np.arange(0.0, 360.0, azimuth_step_deg):
            phi = math.radians(float(phi_deg))
            tip = geometry.R_inner * np.array([math.sin(theta) * math.cos(phi),
                                                math.sin(theta) * math.sin(phi), -math.cos(theta)])
            u = tip - pivot
            u /= np.linalg.norm(u)
            T = tf.from_rotation(_minimal_twist(u, np.eye(3)), tip)
            if clearance_reason(T, geometry, tool, pivot, margin_m) is not None:
                ring_ok = False
                break
        if ring_ok:
            ok.append(float(theta_deg))
    if not ok:
        return None
    groups: list[list[float]] = [[ok[0]]]
    for value in ok[1:]:
        if value - groups[-1][-1] <= resolution_deg * 1.5:
            groups[-1].append(value)
        else:
            groups.append([value])
    best = max(groups, key=lambda group: group[-1] - group[0])
    return best[0], best[-1]


def _axis_pose(position: np.ndarray, tcp: str, reference: np.ndarray) -> PoseSpec:
    R = _minimal_twist(np.array([0.0, 0.0, -1.0]), reference)
    return PoseSpec.from_T("flask", tcp, tf.from_rotation(R, position))


def build_program(scrape: ScrapePlan | FlaskScrape, name: str, robot: str, *,
                  approach_m: float = 0.05, velocity: float = 0.05,
                  contact_velocity: float = 0.02, force_n: Optional[float] = None,
                  motion_mode: str = "position", force_mode: str = "admittance") -> Program:
    """Build an ordinary named-waypoint program from a generated scrape plan."""
    plan = scrape.generate() if isinstance(scrape, FlaskScrape) else scrape
    if not isinstance(plan, ScrapePlan):
        raise FlaskScrapeError("build_program needs a ScrapePlan or FlaskScrape")
    if not plan.reachable:
        raise FlaskScrapeError("the scrape has no reachable poses")
    if not math.isfinite(approach_m) or approach_m <= 0.0:
        raise FlaskScrapeError("approach height must be positive")
    if not 0.0 < velocity <= 1.0 or not 0.0 < contact_velocity <= 1.0:
        raise FlaskScrapeError("velocity scalings must be in (0, 1]")
    if force_n is not None and (not math.isfinite(force_n) or force_n <= 0.0):
        raise FlaskScrapeError("scrape force must be positive")
    _, rim, _, rim_z = _flask_values(plan.geometry)
    reference = plan.reachable[0].pose.T()[:3, :3]
    pivot_pose = _axis_pose(plan.pivot, plan.tool.tcp, reference)
    approach = np.array([rim[0], rim[1], rim_z + approach_m])
    approach_pose = _axis_pose(approach, plan.tool.tcp, reference)
    waypoints = {
        "approach_neck": ProgramWaypoint("approach_neck", pose=approach_pose),
        "neck_pivot": ProgramWaypoint("neck_pivot", pose=pivot_pose),
    }
    for target in plan.reachable:
        waypoints[target.name] = ProgramWaypoint(target.name, pose=target.pose)
    active_mode = force_mode if force_n is not None else motion_mode
    steps: list[Step] = [
        Step("comment", {"text": "Round-bottom flask scrape"}),
        Step("controller", {"mode": active_mode}),
        Step("move_pose", {"target": "approach_neck", "cartesian": False, "velocity": velocity}),
        Step("move_pose", {"target": "neck_pivot", "cartesian": True, "velocity": contact_velocity}),
        Step("move_pose", {"target": plan.reachable[0].name, "cartesian": True,
                           "velocity": contact_velocity}),
    ]
    if force_n is not None:
        steps.append(Step("force", {"wrench": [0.0, 0.0, float(force_n), 0.0, 0.0, 0.0],
                                    "frame": "tool"}))
    for target in plan.reachable[1:]:
        steps.append(Step("move_pose", {"target": target.name, "cartesian": True,
                                        "velocity": contact_velocity}))
    if force_n is not None:
        steps.append(Step("force_off", {}))
    steps.extend([
        Step("move_pose", {"target": "neck_pivot", "cartesian": True, "velocity": contact_velocity}),
        Step("move_pose", {"target": "approach_neck", "cartesian": True, "velocity": contact_velocity}),
    ])
    if force_n is not None and force_mode != motion_mode:
        steps.append(Step("controller", {"mode": motion_mode}))
    return Program(name=name, robot=robot, steps=steps, waypoints=waypoints)


# Bent tools need a second rigid segment and a swept-volume check at the elbow.
# TODO: add them only with those collision checks and dedicated tests.
