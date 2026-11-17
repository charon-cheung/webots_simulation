#!/usr/bin/env bash
# Usage: ./run_simulation.sh [teleop | launch arguments...]
set -e

source "$(dirname "${BASH_SOURCE[0]}")/setup_simulation.bash"

if [[ "${1:-}" == teleop ]]; then
  shift
  # This host's teleop publishes cmd_vel_nav; also cover the upstream cmd_vel name.
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel_nav:=/cmd_vel -r cmd_vel:=/cmd_vel \
    -p stamped:=false -p speed:=0.3 -p turn:=0.6 "$@"
fi
exec ros2 launch webots_simulation simulation_launch.py "$@"
