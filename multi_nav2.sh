#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ $# -eq 0 ]; then
  echo "usage: $0 robot1 [robot2 ...]"
  exit 1
fi
source "$DIR/env.sh"
LOG=${TB3_LOG_DIR:-/tmp/tb3_multi_robot}
mkdir -p "$LOG"
trap 'kill -INT $(jobs -p) 2>/dev/null; wait' INT TERM

for r in "$@"; do
  sed "s#ROBOTNS#$r#g" "$DIR/config/nav2_multi_params.yaml" > "$LOG/nav2_$r.yaml"
  ros2 launch "$DIR/launch/multi_nav2_launch.py" namespace:="$r" \
    params_file:="$LOG/nav2_$r.yaml" > "$LOG/nav2_$r.log" 2>&1 &
  sleep 10
done
echo "logs in $LOG, Ctrl+C stops everything"
wait
