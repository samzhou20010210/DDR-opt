#!/usr/bin/env bash
# Run pool_nav.launch headless for each scenario, check the live config, send the scenario goal, judge the outcome.
#   pool_rescue/scripts/run_nav_scenarios.sh [--mode approach|rescue] [--out DIR] [scenario ...]
# Default: all scenarios in config/scenarios.yaml with a goal. Private ROS master on port 11411. Exit 0 = all PASS.
set -u
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$PKG/../../.." && pwd)"
MODE=approach
OUT="/tmp/pool_nav/$(date +%Y%m%d_%H%M%S)"
SCEN=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    *) SCEN+=("$1"); shift ;;
  esac
done
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
if [[ ${#SCEN[@]} -eq 0 ]]; then
  mapfile -t SCEN < <(python3 -c "import yaml;d=yaml.safe_load(open('$PKG/config/scenarios.yaml'))['scenarios'];print('\n'.join(k for k,v in d.items() if v.get('goal')))")
fi
export ROS_MASTER_URI=http://localhost:11411
unset DISPLAY
mkdir -p "$OUT"
RC=0
for s in "${SCEN[@]}"; do
  D="$OUT/$s"; mkdir -p "$D"; export ROS_HOME="$D/ros_home"
  : > "$D/launch.log"
  roslaunch -p 11411 pool_rescue pool_nav.launch gui:=false mode:="$MODE" scenario:="$s" > "$D/launch.log" 2>&1 &
  LP=$!
  for i in $(seq 1 60); do grep -q "^RESULT:" "$D/launch.log" && break; sleep 1; done
  CHECK=$(grep "^RESULT:" "$D/launch.log" || echo "RESULT: (no live check output)")
  if ! kill -0 "$LP" 2>/dev/null; then
    echo "[$s] roslaunch exited early, see $D/launch.log"; RC=1; continue
  fi
  timeout 300 rosrun pool_rescue nav_tests.py --out "$D" > "$D/nav_test.log" 2>&1
  T=$?
  [[ $T -ne 0 || "$CHECK" != *PASS* ]] && RC=1
  echo "[$s] live check: ${CHECK#RESULT: } | nav: $(grep -m1 '^== ' "$D/nav_test.log" | sed 's/^== //')"
  kill -INT "$LP"; wait "$LP" 2>/dev/null
done
echo "artifacts: $OUT"
exit $RC
