#!/usr/bin/env bash
#
# Reproducible end-to-end docking demo.
#
# Usage:
#   scripts/run_demo.sh [baseline|disturbed] [--headless] [--rviz] [--eval N] [--case N]
#
# Examples:
#   scripts/run_demo.sh baseline               # GUI, no disturbances
#   scripts/run_demo.sh disturbed --rviz       # GUI + RViz, all disturbances
#   scripts/run_demo.sh disturbed --case 1     # corruption(5s) + pose + obstacle
#   scripts/run_demo.sh disturbed --case 2     # scan dropout(5s) + pose + obstacle
#   scripts/run_demo.sh disturbed --autostart  # controller docks automatically
#   scripts/run_demo.sh disturbed --headless   # headless (CI / no display)
#   scripts/run_demo.sh baseline --eval 10     # 10-trial statistics run
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${WS_ROOT:-/root}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

MODE="${1:-baseline}"; shift || true
HEADLESS="false"
RVIZ="false"
EVAL_TRIALS="0"
CASE="0"
AUTOSTART="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS="true"; shift ;;
    --rviz)     RVIZ="true"; shift ;;
    --eval)     EVAL_TRIALS="${2:-10}"; shift 2 ;;
    --case)     CASE="${2:-1}"; shift 2 ;;
    --autostart) AUTOSTART="true"; shift ;;
    *) echo "unknown arg: $1"; exit 1 ;;
  esac
done

DISTURB="false"
[[ "$MODE" == "disturbed" ]] && DISTURB="true"
# Selecting a disturbance case implies disturbed mode; disturbed mode defaults to case 1.
[[ "$CASE" != "0" ]] && DISTURB="true"
[[ "$DISTURB" == "true" && "$CASE" == "0" ]] && CASE="1"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_ROOT}/install/setup.bash"
export TURTLEBOT3_MODEL=burger

# Headless requires a render context for the Gazebo GPU LiDAR -> use Xvfb.
if [[ "$HEADLESS" == "true" && -z "${DISPLAY:-}" ]]; then
  if command -v Xvfb >/dev/null 2>&1; then
    if ! pgrep -x Xvfb >/dev/null 2>&1; then
      Xvfb :99 -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
      sleep 2
    fi
    export DISPLAY=:99
    echo "[run_demo] headless: using Xvfb on DISPLAY=:99"
  else
    echo "[run_demo] WARNING: Xvfb not found; GPU LiDAR may produce no scan."
  fi
fi

echo "[run_demo] mode=${MODE} disturbances=${DISTURB} case=${CASE} headless=${HEADLESS} rviz=${RVIZ} autostart=${AUTOSTART}"

if [[ "$EVAL_TRIALS" != "0" ]]; then
  echo "[run_demo] starting simulation for ${EVAL_TRIALS}-trial evaluation..."
  ros2 launch docking_bringup demo.launch.py \
      headless:="$HEADLESS" disturbances:="$DISTURB" disturbance_case:="$CASE" rviz:="$RVIZ" \
      >/tmp/dock_demo.log 2>&1 &
  LAUNCH_PID=$!
  trap 'kill ${LAUNCH_PID} 2>/dev/null || true; pkill -f gzserver 2>/dev/null || true' EXIT
  sleep 12
  ros2 run docking_bringup dock_eval.py --ros-args \
      -p trials:="${EVAL_TRIALS}" \
      -p csv_path:="/tmp/dock_eval.csv"
else
  if [[ "$AUTOSTART" == "true" ]]; then
    # Bring the stack up first (controller publishes /docking/status from IDLE),
    # then trigger docking via the service after a settle so geometry is ready.
    AUTOSTART_DELAY="${AUTOSTART_DELAY:-30}"
    ros2 launch docking_bringup demo.launch.py \
        headless:="$HEADLESS" disturbances:="$DISTURB" disturbance_case:="$CASE" rviz:="$RVIZ" &
    LAUNCH_PID=$!
    trap 'kill ${LAUNCH_PID} 2>/dev/null || true; pkill -f gzserver 2>/dev/null || true' EXIT
    echo "[run_demo] auto-start: triggering /docking/start in ${AUTOSTART_DELAY}s..."
    sleep "$AUTOSTART_DELAY"
    ros2 service call /docking/start std_srvs/srv/Trigger || true
    wait ${LAUNCH_PID}
  else
    exec ros2 launch docking_bringup demo.launch.py \
        headless:="$HEADLESS" disturbances:="$DISTURB" disturbance_case:="$CASE" rviz:="$RVIZ"
  fi
fi
