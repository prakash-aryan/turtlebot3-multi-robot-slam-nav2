#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ $# -eq 0 ]; then
  echo "usage: $0 robot1:x:y:yaw_deg [robot2:x:y:yaw_deg ...]"
  exit 1
fi
source "$DIR/env.sh"
LOG=${TB3_LOG_DIR:-/tmp/tb3_multi_robot}
mkdir -p "$LOG"
trap 'kill -INT $(jobs -p) 2>/dev/null; wait' INT TERM

/usr/bin/python3 "$DIR/multi_robot_relay.py" "$@" > "$LOG/relay.log" 2>&1 &
sleep 2

for s in "$@"; do
  r=${s%%:*}
  ros2 launch slam_toolbox online_async_decentralized_multirobot_launch.py \
    namespace:="$r" use_sim_time:=false slam_params_file:="$DIR/config/multi_slam_params.yaml" \
    > "$LOG/slam_$r.log" 2>&1 &
done
sleep 8

echo "logs in $LOG, Ctrl+C stops everything"
wait
