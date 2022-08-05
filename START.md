# start

  ```bash
  $ roscore
  $ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_camera_usb.launch
  $ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
  $ roslaunch vision vision.launch
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  $ export TURTLEBOT3_MODEL=burger
  $ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
  ```
## camera

  $ export AUTO_IN_CALIB=action
  $ export GAZEBO_MODE=false

  $ export AUTO_EX_CALIB=action
  $ export AUTO_EX_CALIB=calibration
  $ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_extrinsic_camera_calibration.launch

  $ export AUTO_DT_CALIB=action
  $ export AUTO_DT_CALIB=calibration
  $ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_lane.launch
  $ roslaunch turtlebot3_autorace_control turtlebot3_autorace_control_lane.launch

  $ roslaunch turtlebot3_autorace_camera_detect turtlebot3_autorace_detect_traffic_light.launch

  $ roslaunch turtlebot3_autorace_detect turtlebot3_autorace_detect_parking.launch
 
## triffic light
  $ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 3"

## dectect lane
  $ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"

## detect sign
  ```
  # pub -> name: detect/traffic_sign type -> UInt8  
  # sign 0-8 ->  self.TrafficSign = Enum('TrafficSign', 'divide way construction parking stop tunnel left right noentry')
  1 divide 
  2 way 
  3 construction 
  4 parking 
  5 stop 
  6 tunnel 
  7 left 
  8 right 
  9 noentry
  $ rostopic pub /detect/traffic_sign std_msgs/UInt8 "data: 7" 
  ```
