#!/usr/bin/env bash
# Drop or remove a 0.5 m box in the running Gazebo office, for the replanning demo.
#
#   ros2 run omninav_sim obstacle.sh add 9.0 -1.5      # box at x=9.0, y=-1.5 (map/world coordinates)
#   ros2 run omninav_sim obstacle.sh remove
#
# Tip: in RViz use "Publish Point", click a spot on the green path, and read it from
#      ros2 topic echo /clicked_point
set -e
NAME=box_obstacle
WORLD=office
 
case "$1" in
  add)
    X=${2:?give x}; Y=${3:?give y}
    ros2 run ros_gz_sim create -world "$WORLD" -name "$NAME" -x "$X" -y "$Y" -z 0.25 -string \
'<sdf version="1.9"><model name="box_obstacle"><static>true</static><link name="link">
<collision name="c"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.5 0.5 0.5</size></box></geometry>
<material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material></visual>
</link></model></sdf>'
    ;;
  remove)
    gz service -s /world/$WORLD/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
      --timeout 2000 --req "name: \"$NAME\" type: MODEL"
    ;;
  *)
    echo "usage: obstacle.sh add X Y | obstacle.sh remove"; exit 1 ;;
esac