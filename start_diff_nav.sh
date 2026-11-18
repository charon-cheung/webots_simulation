#!/usr/bin/env bash
# Usage: ./start_diff_nav.sh [params_file:=/absolute/path/navigation.yaml]
# Start start_diff_sim.sh first. Ctrl-C stops planning, leaving simulation running.
set -eo pipefail

navigation_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$navigation_script_dir/setup_simulation.bash"
navigation_log="${DIFF_NAV_LOG:-$simulation_workspace/log/start_diff_nav.log}"
mkdir -p "$(dirname "$navigation_log")"

# Check the current DDS domain, without relying on a cached ros2 daemon.
python3 - <<'PYTHON'
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

rclpy.init()
node = rclpy.create_node('diff_navigation_preflight')
buffer = Buffer()
listener = TransformListener(buffer, node)
seen = set()
first_clock = None


def clock_received(message):
    global first_clock
    stamp = message.clock.sec * 1000000000 + message.clock.nanosec
    if first_clock is None:
        first_clock = stamp
    elif stamp > first_clock:
        seen.add('/clock (advancing)')


subscriptions = [
    node.create_subscription(Clock, '/clock', clock_received, qos_profile_sensor_data),
    node.create_subscription(LaserScan, '/scan', lambda msg: seen.add('/scan') if msg.ranges else None,
                             qos_profile_sensor_data),
    node.create_subscription(OccupancyGrid, '/map', lambda msg: seen.add('/map') if msg.data else None,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
]
required = {'/clock (advancing)', '/scan', '/map', 'map -> base_footprint'}
try:
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'planner_server' in node.get_node_names():
            raise SystemExit('[error] planner_server already exists in this DDS domain.')
        try:
            buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            seen.add('map -> base_footprint')
        except TransformException:
            pass
        if required <= seen:
            break
    else:
        raise SystemExit('[error] Simulation/localization not ready: ' + ', '.join(sorted(required - seen)))
    print('[ready] Clock, map, scan and localization TF are available.')
finally:
    node.destroy_node()
    rclpy.shutdown()
PYTHON

exec > >(tee "$navigation_log") 2>&1
echo "[start] SmacPlanner2D only (log: $navigation_log)"
echo '[stage 1] ComputePathToPose is available after activation; no automatic robot motion.'
exec ros2 launch webots_simulation navigation_launch.py "$@"
