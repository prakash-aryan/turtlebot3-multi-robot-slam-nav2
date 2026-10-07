source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/slam_toolbox_ws/install/setup.bash" ]; then
  source "$HOME/slam_toolbox_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=${TB3_DOMAIN_ID:-31}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TURTLEBOT3_MODEL=burger
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE FASTRTPS_DEFAULT_PROFILES_FILE

LAN_IP=${TB3_LAN_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')}
if [ -n "$LAN_IP" ]; then
  mkdir -p "$HOME/.ros"
  cat > "$HOME/.ros/tb3_dds_lan.xml" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>lan_udp</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>8388608</sendBufferSize>
      <receiveBufferSize>8388608</receiveBufferSize>
      <maxMessageSize>65000</maxMessageSize>
      <interfaceWhiteList><address>$LAN_IP</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="lan_part" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>lan_udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/tb3_dds_lan.xml"
fi
