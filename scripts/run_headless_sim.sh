#!/usr/bin/env bash
# Build (optional), launch the stock simulation headless, send one goal, and verify it.
#
#   scripts/run_headless_sim.sh [--no-build] [--seed N] [--goal X Y YAW] [--out DIR]
#
# Defaults: planner_sim.launch, map seed 2, goal = the launch file's final_x/y/yaw (3.7 0.3 3.14).
# Only planner_sim.launch declares the gui/map_seed args used here.
# Uses its own ROS master port (11411) so it never touches a running roscore.
# Exit code 0 = PASS, 1 = FAIL. Logs + report land in --out (default /tmp/ddr_headless/<timestamp>).
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$WS/src/DDR-opt-main"
BUILD=1; SEED=2; GOAL=(3.7 0.3 3.14); LAUNCH=planner_sim.launch
OUT="/tmp/ddr_headless/$(date +%Y%m%d_%H%M%S)"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0; shift ;;
    --seed) SEED="$2"; shift 2 ;;
    --goal) GOAL=("$2" "$3" "$4"); shift 4 ;;
    --out) OUT="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;   # passed through to smoke_test.py
  esac
done
mkdir -p "$OUT"

source /opt/ros/noetic/setup.bash
cd "$WS"
if [[ $BUILD -eq 1 ]]; then
  echo "[headless] catkin build ..."
  catkin build --no-status > "$OUT/build.log" 2>&1 || { echo "[headless] BUILD FAILED, see $OUT/build.log"; exit 1; }
fi
source "$WS/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11411
export ROS_HOME="$OUT/ros_home"          # keeps roslaunch logs with the report
unset DISPLAY                             # make sure nothing tries to open a window

echo "[headless] launching $LAUNCH (gui:=false map_seed:=$SEED), logs: $OUT/launch.log"
roslaunch -p 11411 plan_manager "$LAUNCH" gui:=false map_seed:="$SEED" > "$OUT/launch.log" 2>&1 &
LPID=$!
cleanup() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
trap cleanup EXIT

python3 "$SRC/scripts/smoke_test.py" --goal "${GOAL[@]}" --out "$OUT" "${EXTRA[@]}" 2>&1 | tee "$OUT/smoke_test.log"
RC=${PIPESTATUS[0]}
echo "[headless] artifacts in $OUT"
exit "$RC"
