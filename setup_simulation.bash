# Source in every terminal used for this simulation.
simulation_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash || return 1
source "$simulation_workspace/install/setup.bash" || return 1

simulation_ip="$(ip -4 -o addr show up scope global | awk '$4 ~ /^172\.16\./ {split($4, a, "/"); print a[1]; exit}')"
if [[ -z "$simulation_ip" ]]; then
  echo 'No active 172.16.* address found; cannot start local simulation DDS.' >&2
  return 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Isolate this simulation from the existing PR2 and real-robot domain.
export ROS_DOMAIN_ID="${WEBOTS_SIMULATION_DOMAIN_ID:-42}"
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
printf -v CYCLONEDDS_URI '<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="%s"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>64</MaxAutoParticipantIndex><Peers><Peer Address="%s"/></Peers></Discovery></Domain></CycloneDDS>' "$simulation_ip" "$simulation_ip"
export CYCLONEDDS_URI
echo "Simulation DDS: $simulation_ip, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
