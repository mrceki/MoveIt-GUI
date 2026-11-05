# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

MoveIt GUI v2: operator GUI for MoveIt 2 + UR on ROS 2 Jazzy (lab robot: UR7e, Robotiq 2F-85, ATI F/T).
Two front ends: the Qt desktop app (`moveit_gui/gui/`) and the web app (`moveit_gui/web/`, used over Tailscale).
In the lab the web GUI runs inside the robot stack's docker image (`ur7e_robotiq`, launched by `scripts/ur7e_gui.sh`).

## Commands

```bash
python3 -m pytest -q test/                          # everything, no ROS needed (~2 min)
python3 -m pytest -q test/test_workcell.py -k setup # one file / one test
node --test test/*.test.cjs                         # front-end JS tests
MOVEIT_GUI_DOCKER_TESTS=1 python3 -m pytest test/test_ft_bias_restart_docker.py   # opt-in, image ur7e_robotiq:buildtest

# try the web GUI on a simulated robot (own config dir, loopback only)
XDG_CONFIG_HOME=$(mktemp -d) python3 -m moveit_gui.web.server --fake --host 127.0.0.1 --port 8199
ros2 run moveit_gui moveit_gui --fake               # Qt app, simulated robot

cd ~/ws && colcon build --packages-select moveit_gui   # ROS install (needs pymoveit2 in the workspace)
```

`tools/web_screenshots.py` makes the `docs/ux/web_*.png` screenshots (playwright).

## Architecture

- **`core.py` `RobotCore`** is shared by both front ends. It owns the bridge, profile, calibration store,
  geometry, control modes, program runner and policies, and the motion-safety rules (`idle()`,
  `begin_motion`, `stop_all`). Front ends call it; they never re-implement its rules.
- **Bridge** (`bridge_api.py` is the contract): `ros_bridge.py` is the single rclpy node; `fake_bridge.py` is the
  simulated robot used by `--fake` and by nearly every test. Motion calls block, so call them from worker threads.
  `update()` is polled at 10 Hz. ROS callbacks only update locked state.
- **Web**: `web/server.py` is stdlib HTTP plus an MJPEG camera stream. `web/service.py` `WebService` dispatches
  `POST /api/cmd` to `cmd_<name>` methods and builds the polled `state()`. SETUP commands live in the
  `web/workcell.py` mixin.
  - Commands need the control **lease** unless marked `cmd_x.read_only = True`.
  - Domain errors (`PartError`, `SetupError`, `ServiceError`, ...) are turned into user messages in `command()`.
  - New error types must be added there.
- **Logic without the web** stays in plain modules with their own tests, e.g. `parts.py` (URDF/SRDF parts),
  `handeye.py`, `setups.py`, `services.py` (driver/pipeline processes), `program.py`, `control_modes.py`.
- **Front end**: `web/static/*.js` ES modules, no build step. The DOM is built with `h(tag, props, ...kids)`.
  `app.js` holds the pages; `workcell.js` and `drivers.js` build SETUP views from a `ctx` of helpers. The server owns
  every rule; the page only shows and asks.
- **Config** is under `config_dir()` = `$XDG_CONFIG_HOME/moveit_gui` (in the container `/data/config/moveit_gui`).
  Robot profiles are `profiles/*.yaml`.
- **Restarts**: the web GUI exits with 75 (robot stack + GUI) or 76 (GUI only), and the launcher restarts
  accordingly. URDF/SRDF parts reach the robot model only after a stack restart.

## Conventions

- `T_a_b` is the pose of frame `b` in frame `a` (`p_a = T_a_b · p_b`). Quaternions are `xyzw`.
- Nothing moves the robot except through `RobotCore` (refused unless idle; STOP stops it). The GUI never
  switches control modes on its own.
- Child processes (`ros2 bag record`, drivers) get their own process group and stop with SIGINT → SIGTERM →
  SIGKILL.
- Only numpy + scipy are available in the robot image. Don't add dependencies on trimesh, open3d, etc.
- New or changed web features get a few plain sentences in `docs/WEB.md`. UI text is short and plain, with details
  in the ⓘ note.
