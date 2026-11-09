#!/usr/bin/env bash
# DDR-opt rov_holo on ROV_V4 in the uwv-simulator Gazebo pool, headless: 14 goals across the pool, then thrust-off checks.
#
#   scripts/run_headless_gazebo.sh [--no-build] [--goal X Y YAW] [--out DIR] [multigoal args...]
#
# Pool frame: origin at the pool corner, x in [0, 25], y in [0, 50]. Own ROS master (11412) and Gazebo master (11446).
# After the goals:
#   1. kill thruster_allocation -> the bridge must zero all 8 thrusters within thrust_timeout (Gazebo keeps the last
#      command otherwise);
#   2. publish a test thrust through the bridge, then kill the bridge -> its shutdown must zero them again.
# Exit 0 = all goals reached and both thrust-off checks passed.
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$WS/src/DDR-opt-main"
BUILD=1; GOAL=(); OUT="/tmp/ddr_gazebo/$(date +%Y%m%d_%H%M%S)"; EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0; shift ;;
    --goal) GOAL=(--goals "$2" "$3" "$4"); shift 4 ;;
    --out) OUT="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
mkdir -p "$OUT"
set +u; source /opt/ros/noetic/setup.bash; set -u
cd "$WS"
if [[ $BUILD -eq 1 ]]; then
  catkin build --no-status > "$OUT/build.log" 2>&1 || { echo "[gazebo] BUILD FAILED, see $OUT/build.log"; exit 1; }
fi
set +u; source "$WS/devel/setup.bash"; set -u
export ROS_MASTER_URI=http://localhost:11412 GAZEBO_MASTER_URI=http://localhost:11446
export ROS_HOME="$OUT/ros_home" ROSCONSOLE_STDOUT_LINE_BUFFERED=1
unset DISPLAY
echo "[gazebo] launching planner_gazebo.launch (headless), logs: $OUT/launch.log"
roslaunch -p 11412 rov_gazebo_bridge planner_gazebo.launch gui:=false rviz:=false > "$OUT/launch.log" 2>&1 &
LPID=$!
cleanup() { kill -INT "$LPID" 2>/dev/null; wait "$LPID" 2>/dev/null; }
trap cleanup EXIT

python3 "$SRC/scripts/multigoal.py" --robot rov_holo --gazebo --out "$OUT" "${GOAL[@]}" "${EXTRA[@]}" 2>&1 | tee "$OUT/multigoal.log"
RC=${PIPESTATUS[0]}

python3 - "$OUT" <<'PY' 2>&1 | tee "$OUT/thrust_off.log"
import subprocess, sys, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import rosgraph, time
def _sim_time_set():
    try:
        return rosgraph.Master("/thrust_off_check").hasParam("/use_sim_time")
    except Exception:   # master not up yet
        return False
while not _sim_time_set():
    time.sleep(0.2)
rospy.init_node("thrust_off_check", anonymous=True)
def use():
    m = rospy.wait_for_message("/rov_v4/thruster_use", JointState, timeout=5.0)
    return max(m.position) if m.position else 0.0
ok = True
before = use()
subprocess.call(["rosnode", "kill", "/thruster_allocation"], stdout=subprocess.DEVNULL)
rospy.sleep(1.0)
a = use()
print("[thrust_off] max thruster use %.1f%% before; after killing thruster_allocation (watchdog): %.2f%%" % (before, a))
ok &= a < 1e-6
pub = rospy.Publisher("/thruster_allocation/thrust", Float64MultiArray, queue_size=1)
rospy.sleep(0.5)
tm = rospy.Timer(rospy.Duration(0.02), lambda _: pub.publish(Float64MultiArray(data=[0, 0, 0, 5, 5, 0, 5, 5])))
rospy.sleep(0.5)
b = use()
subprocess.call(["rosnode", "kill", "/gazebo_bridge"], stdout=subprocess.DEVNULL)   # test thrust keeps coming
rospy.sleep(1.0)
c = use()
tm.shutdown()
print("[thrust_off] test thrust through the bridge: %.1f%%; after killing gazebo_bridge (shutdown): %.2f%%" % (b, c))
ok &= b > 1.0 and c < 1e-6
print("[thrust_off] %s" % ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
PY
RC2=${PIPESTATUS[0]}
echo "[gazebo] artifacts in $OUT"
[[ $RC -eq 0 && $RC2 -eq 0 ]] && exit 0 || exit 1
