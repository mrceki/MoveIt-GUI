import math
import os

import numpy as np
import pytest

from moveit_gui.flask_geometry import fit_flask
from moveit_gui.flask_scrape import (
    FlaskScrapeError,
    StraightTool,
    build_program,
    clearance_reason,
    generate_scrape_poses,
    reachable_band,
)
from moveit_gui.profile import RobotProfile


ROOT = os.path.join(os.path.dirname(__file__), "..")


def _geometry():
    radius, neck_outer, tip = 0.0425, 0.0135, 0.002
    join = math.sqrt(radius * radius - neck_outer * neck_outer)
    rim_z = join + 0.060
    angles = np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)
    rim = [[(neck_outer + tip) * math.cos(a), (neck_outer + tip) * math.sin(a), rim_z] for a in angles]
    rng = np.random.default_rng(8)
    directions = rng.normal(size=(60, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    bulb = directions * (radius + tip)
    return fit_flask(rim, bulb, r_tip=tip)


def _tool():
    return StraightTool("spatula_tip", shaft_radius=0.0015, shaft_length=0.25)


def test_generated_poses_are_on_sphere_through_pivot_clear_and_smooth():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0.0, 12.0, 4, 30.0)
    assert plan.reachable and not plan.unreachable
    previous = None
    for target in plan.reachable:
        T = target.pose.T()
        tip, axis = T[:3, 3], T[:3, 2]
        assert np.linalg.norm(tip) == pytest.approx(g.R_inner, abs=0.0001)
        to_tip = tip - plan.pivot
        assert np.linalg.norm(np.cross(axis, to_tip)) < 1e-8
        assert axis @ to_tip > 0.0
        assert clearance_reason(T, g, tool, plan.pivot) is None
        if previous is not None:
            assert np.linalg.det(previous.T @ T[:3, :3]) == pytest.approx(1.0)
        previous = T[:3, :3]
    ring_az = [[p.azimuth_deg for p in plan.reachable if p.ring == k] for k in range(4)]
    assert ring_az[0][0] == 0.0 and ring_az[1][0] > ring_az[1][-1]


def test_unreachable_targets_have_reasons_and_band_agrees():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0.0, 65.0, 6, 30.0)
    assert plan.reachable and plan.unreachable
    assert all(u.reason and len(u.reason) < 100 for u in plan.unreachable)
    assert any("neck bore" in u.reason or "bulb wall" in u.reason for u in plan.unreachable)
    band = reachable_band(g, tool, resolution_deg=1.0, azimuth_step_deg=30.0)
    assert band is not None and band[0] == pytest.approx(0.0)
    assert 5.0 < band[1] < 40.0


def test_short_and_wide_tools_are_rejected_with_specific_reasons():
    g = _geometry()
    short = generate_scrape_poses(g, StraightTool("tip", 0.001, 0.05), 0, 0, 1, 90)
    assert not short.reachable and all("too short" in u.reason for u in short.unreachable)
    wide = generate_scrape_poses(g, StraightTool("tip", 0.012, 0.25), 0, 0, 1, 90)
    assert not wide.reachable and all("too wide" in u.reason for u in wide.unreachable)


def test_press_depth_and_input_errors():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0, 5, 2, 90, press_depth_mm=0.5)
    assert all(np.linalg.norm(p.pose.position) == pytest.approx(g.R_inner + 0.0005, abs=1e-7)
               for p in plan.reachable)
    with pytest.raises(FlaskScrapeError, match="polar angles"):
        generate_scrape_poses(g, tool, -1, 20, 2, 10)
    with pytest.raises(FlaskScrapeError, match="ring step"):
        generate_scrape_poses(g, tool, 0, 20, 2, 0)


def test_program_is_plain_named_flask_waypoints_and_validates():
    g, tool = _geometry(), _tool()
    plan = generate_scrape_poses(g, tool, 0, 10, 3, 45)
    program = build_program(plan, "flask scrape", "ur7e_lab", force_n=4.0)
    assert all(w.pose.frame == "flask" and w.pose.tcp == tool.tcp for w in program.waypoints.values())
    types = [step.type for step in program.steps]
    assert types[:5] == ["comment", "controller", "move_pose", "move_pose", "move_pose"]
    assert types.count("force") == 1 and types.count("force_off") == 1
    assert types[-3:] == ["move_pose", "move_pose", "controller"]
    assert program.steps[2].params["cartesian"] is False
    assert all(step.params.get("cartesian") is True for step in program.steps if step.type == "move_pose"
               and step is not program.steps[2])
    profile = RobotProfile.load(os.path.join(ROOT, "profiles", "ur7e_lab.yaml"))
    assert program.validate(profile) == []
