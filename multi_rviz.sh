#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ $# -eq 0 ]; then
  echo "usage: $0 robot1 [robot2 ...]"
  exit 1
fi
source "$DIR/env.sh"
LOG=${TB3_LOG_DIR:-/tmp/tb3_multi_robot}
mkdir -p "$LOG" "$DIR/rviz"
trap 'kill -INT $(jobs -p) 2>/dev/null; wait' INT TERM

for r in "$@"; do
  sed "s#<robot_namespace>#/$r#g" "$DIR/config/multi_nav2_view.rviz" > "$DIR/rviz/$r.rviz"
  rviz2 -d "$DIR/rviz/$r.rviz" --ros-args -r __ns:=/$r -r __node:=rviz \
    -r /map:=map -r /tf:=tf -r /tf_static:=tf_static -r /goal_pose:=goal_pose \
    -r /clicked_point:=clicked_point -r /initialpose:=initialpose \
    > "$LOG/rviz_$r.log" 2>&1 &
done
wait
