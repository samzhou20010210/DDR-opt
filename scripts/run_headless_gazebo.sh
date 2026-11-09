#!/usr/bin/env bash
# DDR-opt rov_holo on ROV_V4 in the uwv-simulator Gazebo pool, headless: 14 goals across the pool, then thrust-off checks.
#
#   scripts/run_headless_gazebo.sh [--no-build] [--goal X Y YAW] [--out DIR] [multigoal args...]
#
# Pool frame: origin at the pool corner, x in [0, 25], y in [0, 50]. Own ROS master (11412) and Gazebo master (11446).
# After the goals, the stop tests (README "Stops"): planning stop keeps depth hold and resumes on the next goal;
# manual and comm-loss safety stops cut all 8 thrusters, latched until /gazebo_bridge/reset_safety_stop; bridge
# shutdown cuts all 8. Exit 0 = all goals reached and every stop test passed.
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

python3 - "$OUT" <<'PY' 2>&1 | tee "$OUT/stop_tests.log"
# Stop behaviour (README "Stops"):
#   1. planning stop (/planner/emergency_stop): planar thrust 0, depth / attitude hold continues, a new goal resumes
#   2. manual safety stop (/rov_safety/estop): all 8 thrusters 0, latched (test thrust ignored)
#   3. reset (/gazebo_bridge/reset_safety_stop): thrust passes again
#   4. comm loss (thruster_allocation killed): all 8 thrusters 0, latched
#   5. bridge shutdown: all 8 thrusters 0
import subprocess, sys, time, rosgraph, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from carstatemsgs.msg import Polynome
def _sim_time_set():
    try:
        return rosgraph.Master("/stop_tests").hasParam("/use_sim_time")
    except Exception:   # master not up yet
        return False
while not _sim_time_set():
    time.sleep(0.2)
rospy.init_node("stop_tests", anonymous=True)
V = [i for i, z in enumerate(rospy.get_param("/thruster_allocation/thrusters/axis_z")) if abs(z) > 0.5]
H = [i for i in range(8) if i not in V]
target = rospy.get_param("/gazebo_bridge/depth/target")
st = {"f": None, "d": [], "traj": 0}
rospy.Subscriber("/thruster_allocation/thrust", Float64MultiArray, lambda m: st.__setitem__("f", list(m.data)))
rospy.Subscriber("/gazebo_bridge/depth", Float64, lambda m: st["d"].append((rospy.get_time(), m.data)))
rospy.Subscriber("/planner/traj_poly", Polynome, lambda m: st.__setitem__("traj", st["traj"] + 1))
def use():
    m = rospy.wait_for_message("/rov_v4/thruster_use", JointState, timeout=5.0)
    return max(m.position) if m.position else 0.0
res = []
def check(name, ok, info):
    res.append(ok)
    print("[stop_tests] %-58s %s  %s" % (name, "PASS" if ok else "FAIL", info), flush=True)
fake = rospy.Publisher("/thruster_allocation/thrust", Float64MultiArray, queue_size=1)
estop = rospy.Publisher("/rov_safety/estop", Bool, queue_size=1, latch=True)
plan_stop = rospy.Publisher("/planner/emergency_stop", Bool, queue_size=1)
goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
rospy.sleep(1.0)

# 1. planning stop
plan_stop.publish(Bool(True))
rospy.sleep(1.0)
t1 = rospy.get_time()
rospy.sleep(5.0)
f = st["f"] or [0] * 8
dep = [d for t, d in st["d"] if t >= t1]
derr = max(abs(d - target) for d in dep) if dep else 99
check("1 planning stop: planar 0, depth held 5 s", max(abs(f[i]) for i in H) < 0.5 and max(abs(f[i]) for i in V) > 0.2
      and derr < 0.05, "horizontal max %.2f N, vertical max %.2f N, depth err max %.3f m"
      % (max(abs(f[i]) for i in H), max(abs(f[i]) for i in V), derr))
n0 = st["traj"]
g = PoseStamped(); g.header.frame_id = "world"; g.header.stamp = rospy.Time.now()
g.pose.position.x, g.pose.position.y, g.pose.orientation.w = 8.0, 8.0, 1.0
goal.publish(g)
rospy.sleep(5.0)
check("1 planning stop: next goal resumes", st["traj"] > n0, "%d new trajectory(ies)" % (st["traj"] - n0))

# 2. manual safety stop (while driving to the new goal)
estop.publish(Bool(True))
rospy.sleep(1.0)
u = use()
tm = rospy.Timer(rospy.Duration(0.02), lambda _: fake.publish(Float64MultiArray(data=[0, 0, 0, 5, 5, 0, 5, 5])))
rospy.sleep(1.0)
u2 = use()
tm.shutdown()
check("2 manual safety stop: all 0, latched (test thrust ignored)", u < 1e-6 and u2 < 1e-6, "use %.2f%% / %.2f%%" % (u, u2))
estop.publish(Bool(False))

# 3. reset -> thrust passes
rospy.wait_for_service("/gazebo_bridge/reset_safety_stop", timeout=5.0)
rospy.ServiceProxy("/gazebo_bridge/reset_safety_stop", Trigger)()
rospy.sleep(1.0)
u3 = use()
check("3 reset: thrust passes again", u3 > 1.0, "use %.2f%% (allocation depth hold)" % u3)

# 4. comm loss
subprocess.call(["rosnode", "kill", "/thruster_allocation"], stdout=subprocess.DEVNULL)
rospy.sleep(1.0)
u4 = use()
tm = rospy.Timer(rospy.Duration(0.02), lambda _: fake.publish(Float64MultiArray(data=[0, 0, 0, 5, 5, 0, 5, 5])))
rospy.sleep(1.0)
u5 = use()
check("4 comm loss (allocation killed): all 0, latched", u4 < 1e-6 and u5 < 1e-6, "use %.2f%% / %.2f%%" % (u4, u5))

# 5. reset, test thrust flowing, kill the bridge
rospy.ServiceProxy("/gazebo_bridge/reset_safety_stop", Trigger)()
rospy.sleep(0.5)
u6 = use()
subprocess.call(["rosnode", "kill", "/gazebo_bridge"], stdout=subprocess.DEVNULL)
rospy.sleep(1.0)
u7 = use()
tm.shutdown()
check("5 bridge shutdown: all 0", u6 > 1.0 and u7 < 1e-6, "use %.2f%% before, %.2f%% after" % (u6, u7))
print("[stop_tests] %s" % ("PASS" if all(res) else "FAIL"))
sys.exit(0 if all(res) else 1)
PY
RC2=${PIPESTATUS[0]}
echo "[gazebo] artifacts in $OUT"
[[ $RC -eq 0 && $RC2 -eq 0 ]] && exit 0 || exit 1
