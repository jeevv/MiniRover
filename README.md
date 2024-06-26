# The Astra Rover Project

A repo containing various software utilised by Astra Robotics in our rover project. Also helps coordinate tasks.

## Progress

### Tasks remaining:
1. Tuning PID constants
2. Interfacing IMU and fusing data with wheel encoders using an EKF\
   -> IMU code complete but yet to be tested\
   -> EKF pending
### Tasks completed:

## Helpful tips
Microros installation (credits to Jeevottam) - \
command to run ekf - ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/src/rover_launch/config/params.yaml
## Issues
1. Map generated by SLAM has accurcy issues, requires IMU to improve 
2. PID constants are not tuned 

## Changelog
### Version 1.0.0
  -> Created a GitHub repo to store relevant code and update available tasks\
  -> Created interrupts to interface wheel encoders and published them to ROS topic\
  -> Interfaced motor drivers with the STM32 using PWM\
  -> Used cmd_vel to control motor speed\
  -> Interfaced a 2D lidar to publish to /scan using ROS drivers
### Version 1.1.0
  -> Used slam_toolbox to generate maps\
  -> Used wheel encoders for odomtery\
  -> Used teleop_twist_keyboard to publish to cmd_vel
### Version 1.2.0 (Current)
  -> Completed writing code for PID control for linear and angular velocity\
  -> Completed writing code to interface IMU
  
