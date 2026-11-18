#!/usr/bin/env bash
# Usage: ./start_diff_sim.sh [map:=/path/hall.yaml] [rviz:=false] [rendering:=false]
# Foreground launch: Ctrl-C stops the complete simulation/localization session.
set -eo pipefail

simulation_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$simulation_script_dir/setup_simulation.bash"

# Never start a second simulator or let SLAM and AMCL compete for map -> odom.
pattern='ros2 launch webots_simulation (simulation_launch\.py|slam_launch\.py|localization_launch\.py|diff_sim_launch\.py)( |$)'
if pgrep -af "$pattern"; then
  echo '[error] A differential simulation or mapping session is already running. Stop it with Ctrl-C first.' >&2
  exit 1
fi

simulation_log="${DIFF_SIM_LOG:-$simulation_workspace/log/start_diff_sim.log}"
mkdir -p "$(dirname "$simulation_log")"
# Inherited by launch, nodes, and log workers, including orphaned children.
export RBS_DIFF_SESSION=simulation
exec > >(awk '
  { print }
  /DiffDrive ready:/ { driver_ready = 1 }
  /\[lifecycle_manager_localization\].*Managed nodes are active/ { localization_ready = 1 }
  driver_ready && localization_ready && !reported {
    print "\033[5;93m------ startup webots simulation done ! ------\033[0m"
    reported = 1
  }
  { fflush() }
' | tee "$simulation_log") 2>&1
echo "[start] Differential simulation + AMCL (log: $simulation_log)"
echo '[initial pose] Default: map origin (x=0, y=0, yaw=0); RViz can override it.'
exec ros2 launch webots_simulation diff_sim_launch.py "$@"
