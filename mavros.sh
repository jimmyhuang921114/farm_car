ros2 launch mavros apm.launch \
  fcu_url:=/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_3A004C000851333036393833-if00:115200 \
  gcs_url:=udp://@127.0.0.1:14550


ros2 run pixhawk_control veclocity --ros-args \
  -p mode:=brushed \
  -p input_topic:=/rc/cmd_vel \
  -p rc_steer_idx:=0 -p rc_thrtl_idx:=2 \
  -p turn_gain:=1.0 -p max_linear:=1.0 -p max_angular:=1.0