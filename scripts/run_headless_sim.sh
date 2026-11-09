#!/usr/bin/env bash
# Build (optional), launch a DDR-opt simulation headless, send one goal, and verify it.
#
#   scripts/run_headless_sim.sh [--no-build] [--launch planner_sim|planner_nmpc|planner_sim_unknown]
#                               [--robot rov|car|rov_holo] [--seed N] [--goal X Y YAW] [--multigoal] [--out DIR]
#                               [smoke_test / multigoal args...]
#
# --multigoal (always on for rov_holo): scripts/multigoal.py sends 14 goals across the map (or the --goal one) and
# reports per goal: path length, time, clearance, tracking (rov_holo also NMPC timing and thruster allocation).
#
# Defaults: planner_sim, robot rov (ROV V4), the launch file's own map seed (rov: 4, car: random), goal (3.7 0.3 3.14).
# The command-limit checks follow the robot (rov: v in [-0.3, 1.0], |omega| <= 0.8; car: +-3, 3).
# Uses its own ROS master port (11411) so it never touches a running roscore.
# Exit code 0 = PASS, 1 = FAIL. Logs + report land in --out (default /tmp/ddr_headless/<timestamp>).
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$WS/src/DDR-opt-main"
BUILD=1; SEED=""; GOAL=(3.7 0.3 3.14); LAUNCH=planner_sim; ROBOT=rov; MULTI=0
OUT="/tmp/ddr_headless/$(date +%Y%m%d_%H%M%S)"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0; shift ;;
    --launch) LAUNCH="${2%.launch}"; shift 2 ;;
    --robot) ROBOT="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --goal) GOAL=("$2" "$3" "$4"); shift 4 ;;
    --out) OUT="$2"; shift 2 ;;
    --multigoal) MULTI=1; shift ;;
    *) EXTRA+=("$1"); shift ;;   # passed through to smoke_test.py
  esac
done
case "$ROBOT" in
  rov) LIMITS=(--max-vel 1.0 --min-vel -0.3 --max-omega 0.8) ;;
  car) LIMITS=(--max-vel 3.0 --min-vel -3.0 --max-omega 3.0) ;;
  rov_holo) LIMITS=(); MULTI=1 ;;   # limits are read from the running holo_nmpc parameters
  *) echo "unknown --robot $ROBOT (rov|car|rov_holo)"; exit 2 ;;
esac
mkdir -p "$OUT"

set +u; source /opt/ros/noetic/setup.bash; set -u   # ROS setup scripts reference unset variables
cd "$WS"
if [[ $BUILD -eq 1 ]]; then
  echo "[headless] catkin build ..."
  catkin build --no-status > "$OUT/build.log" 2>&1 || { echo "[headless] BUILD FAILED, see $OUT/build.log"; exit 1; }
fi
set +u; source "$WS/devel/setup.bash"; set -u

export ROS_MASTER_URI=http://localhost:11411
export ROS_HOME="$OUT/ros_home"          # keeps roslaunch logs with the report
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1  # INFO lines reach launch.log immediately
unset DISPLAY                             # make sure nothing tries to open a window

SEED_ARG=(); [[ -n "$SEED" ]] && SEED_ARG=(map_seed:="$SEED")
echo "[headless] launching $LAUNCH.launch (robot:=$ROBOT gui:=false ${SEED_ARG[*]:-default map seed}), logs: $OUT/launch.log"
roslaunch -p 11411 plan_manager "$LAUNCH.launch" robot:="$ROBOT" gui:=false "${SEED_ARG[@]}" > "$OUT/launch.log" 2>&1 &
LPID=$!
cleanup() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
trap cleanup EXIT

if [[ $MULTI -eq 1 ]]; then
  # multi-goal check across the map (14 goals by default; --goal X Y YAW replaces them with one)
  GOAL_ARGS=(); [[ " ${GOAL[*]} " != " 3.7 0.3 3.14 " ]] && GOAL_ARGS=(--goals "${GOAL[@]}")
  python3 "$SRC/scripts/multigoal.py" --robot "$ROBOT" --out "$OUT" "${GOAL_ARGS[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUT/multigoal.log"
  RC=${PIPESTATUS[0]}
else
python3 "$SRC/scripts/smoke_test.py" --goal "${GOAL[@]}" --out "$OUT" "${LIMITS[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUT/smoke_test.log"
RC=${PIPESTATUS[0]}
fi
echo "[headless] artifacts in $OUT"
exit "$RC"
