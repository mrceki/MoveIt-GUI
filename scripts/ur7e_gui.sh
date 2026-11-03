#!/usr/bin/env bash
# One command for the UR7e + MoveIt GUI:
#   1. rebuilds the ur7e_robotiq:jazzy image if ~/ur7e_robotiq changed since it was built
#   2. builds the GUI workspace (~/ws: moveit_gui + pymoveit2)
#   3. starts the robot stack (gui.launch.py) in the container
#   4. starts the GUI. Ctrl-C stops the GUI and the robot stack.
#
#   ur7e_gui.sh                         # real robot, web GUI on http://<tailscale ip>:8180/
#   ur7e_gui.sh --desktop               # real robot, desktop (Qt) GUI on this PC
#   ur7e_gui.sh --fake                  # no robot: simulated arm (web; add --desktop for Qt)
#   ur7e_gui.sh --robot-ip 10.6.203.101 -- rviz:=true servo:=true    # extra gui.launch.py args after --
#
# Saved data (programs, TCPs, fixtures, payload/vial results, logs) goes to ~/ur7e_data.
set -eo pipefail

ROBOT_IP=10.6.203.101
MODE=web
FAKE=0
PORT=8180
# Our own ROS domain: other PCs on the lab network run ROS on the default domain 0 (e.g. a Gazebo UR sim on
# 10.6.203.36), and a second /controller_manager on the same domain breaks controller switching.
# Anything that must talk to this stack (policy runner, ros2 CLI) needs  export ROS_DOMAIN_ID=42  too.
DOMAIN=${UR7E_ROS_DOMAIN_ID:-42}
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --robot-ip) ROBOT_IP="$2"; shift 2 ;;
    --desktop)  MODE=desktop; shift ;;
    --fake)     FAKE=1; shift ;;
    --port)     PORT="$2"; shift 2 ;;
    --)         shift; LAUNCH_ARGS=("$@"); break ;;
    -h|--help)  sed -n 2,14p "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 1 ;;
  esac
done

IMAGE=ur7e_robotiq:jazzy
NAME=${UR7E_CONTAINER:-ur7e_robotiq}
REPO="$HOME/ur7e_robotiq"
WS="$HOME/ws"
DATA="$HOME/ur7e_data"
mkdir -p "$DATA/logs"
say() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

# ---------------------------------------------------------------- 1. image
image_time=$(docker image inspect "$IMAGE" --format '{{.Created}}' 2>/dev/null || true)
if [ -z "$image_time" ]; then
  say "Image $IMAGE missing: building it (takes a while)"
  "$REPO/docker/build.sh"
else
  image_epoch=$(date -d "$image_time" +%s)
  src_epoch=$(find "$REPO" -path "$REPO/.git" -prune -o -name __pycache__ -prune -o -type f -printf '%T@\n' \
              | sort -n | tail -1 | cut -d. -f1)
  if [ "${src_epoch:-0}" -gt "$image_epoch" ]; then
    say "~/ur7e_robotiq changed since the image was built: rebuilding $IMAGE"
    "$REPO/docker/build.sh"
  fi
fi

# ---------------------------------------------------------------- 2. GUI workspace
say "Building the GUI workspace (~/ws)"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$WS:/ws" -w /ws "$IMAGE" \
  bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select pymoveit2 moveit_gui' \
  > "$DATA/logs/ws_build.log" 2>&1 || { tail -30 "$DATA/logs/ws_build.log"; exit 1; }

# ---------------------------------------------------------------- 3. robot stack
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "A container called $NAME is already running (another terminal?). Stop it first:"
  echo "    docker stop $NAME"
  exit 1
fi
cleanup() { say "Stopping the robot stack"; docker stop -t 10 "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM

xhost +local:root >/dev/null 2>&1 || true
stamp=$(date +%Y%m%d_%H%M%S)
if [ "$FAKE" = 1 ]; then
  STACK='sleep infinity'
else
  STACK="ros2 launch ur7e_robotiq_bringup gui.launch.py robot_ip:=$ROBOT_IP ${LAUNCH_ARGS[*]} 2>&1 | tee /data/logs/launch_$stamp.log"
fi
docker run -d --rm --name "$NAME" --net=host -e ROS_DOMAIN_ID="$DOMAIN" \
  -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATA:/data" -v "$WS:/ws" \
  "$IMAGE" bash -c "source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash && $STACK" >/dev/null

ENV='source /opt/ros/jazzy/setup.bash && source /opt/ur7e_ws/install/setup.bash && source /ws/install/setup.bash && export XDG_CONFIG_HOME=/data/config'

if [ "$FAKE" = 0 ]; then
  say "Robot stack starting (robot $ROBOT_IP). Log: ~/ur7e_data/logs/launch_$stamp.log"
  ok=0
  for _ in $(seq 1 45); do
    docker ps --format '{{.Names}}' | grep -qx "$NAME" || { echo "The robot stack exited:"; tail -30 "$DATA/logs/launch_$stamp.log"; exit 1; }
    if docker exec "$NAME" bash -c "$ENV && timeout 5 ros2 control list_controllers" 2>/dev/null | grep -qE "^joint_state_broadcaster +[^ ]+ +active"; then ok=1; break; fi
    sleep 2
  done
  if [ "$ok" = 1 ]; then
    say "Controllers are up"
  else
    echo "Controllers not up after 90 s: starting the GUI anyway (it shows what is missing)."
    echo "Last launch log lines:"; tail -15 "$DATA/logs/launch_$stamp.log"
  fi
  echo "    On the pendant: load and PLAY the External Control program, unless you passed headless_mode:=true."
fi

# ---------------------------------------------------------------- 4. GUI
GUI_ARGS="--programs-dir /data/programs --policies-dir /data/policies"
[ "$FAKE" = 1 ] && GUI_ARGS="$GUI_ARGS --fake"
if [ "$MODE" = web ]; then
  HOST=$(tailscale ip -4 2>/dev/null | head -1)
  if [ -z "$HOST" ]; then
    HOST=127.0.0.1
    echo "Tailscale is not up: the web GUI is only reachable from this PC."
  fi
  say "Web GUI: http://$HOST:$PORT/   (Ctrl-C here stops the GUI and the robot stack)"
  GUI_CMD="ros2 run moveit_gui moveit_gui_web --host $HOST --port $PORT $GUI_ARGS"
else
  say "Desktop GUI (closing the window stops the robot stack)"
  GUI_CMD="ros2 run moveit_gui moveit_gui $GUI_ARGS"
fi
# In the background + wait, so Ctrl-C reaches the cleanup at once; stopping the container ends the GUI.
docker exec "$NAME" bash -c "$ENV && $GUI_CMD" &
wait $!
