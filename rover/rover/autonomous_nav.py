# Import nav2 commander API
from nav2_simple_commander.robot_navigator import BasicNavigator

# Import python API for ros2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from math import sin,cos,radians

def main():
    
    # Initialise ros2 functionality
    rclpy.init()

    # Instantize a BasicNavigator object
    navigator = BasicNavigator()

    # Set an initial pose
    initial_pose = PoseStamped()

    # Pass f]data for the header of initial pose required by nav2
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Read user input for initial point and orientation
    user_initial_point = input("Enter an initial point in x y z format: ").split()

    user_initial_angle = input("Enter an initial orientation in roll pitch yaw format: ").split()
    
    # Convert read data to float
    for i in range(len(user_initial_point)):
        user_initial_point[i] = float(user_initial_point[i])
    

    for i in range(len(user_initial_angle)):
        user_initial_angle[i] = float(user_initial_angle[i])

    # Convert user specified pose from euler angles to quaternions
    user_initial_angle = eul_to_quat(user_initial_angle)

    # Pass data to the pose object
    initial_pose.pose.position.x = float(user_initial_point[0])
    initial_pose.pose.position.y = float(user_initial_point[1])
    initial_pose.pose.position.z = float(user_initial_point[2])
    initial_pose.pose.orientation.x = float(user_initial_angle[0])
    initial_pose.pose.orientation.y = float(user_initial_angle[1])
    initial_pose.pose.orientation.z = float(user_initial_angle[2])
    initial_pose.pose.orientation.w = float(user_initial_angle[3])

    # Pass the initial pose to the navigator object
    navigator.setInitialPose(initial_pose)

def eul_to_quat(euler_angles: list) ->list:
    '''
    Function that returns an orientation in quaternion format

    Arguments:
        euler_angles (List): A list containing a pose specified in euler angles or
        roll, pitch, yaw in that order. Units are in degrees

    Returns:
        unit_quaternion (List): A list containing pose in unit quaternion format
    '''
    unit_quaternion = list([0.0,0.0,0.0,0.0])
    cr = cos(radians(euler_angles[0] * 0.5))
    sr = sin(radians(euler_angles[0] * 0.5))
    cp = cos(radians(euler_angles[1] * 0.5))
    sp = sin(radians(euler_angles[1] * 0.5))
    cy = cos(radians(euler_angles[2] * 0.5))
    sy = sin(radians(euler_angles[2] * 0.5))

    unit_quaternion[0] = float(cr * cp * cy + sr * sp * sy)
    unit_quaternion[1] = float(sr * cp * cy - cr * sp * sy)
    unit_quaternion[2] = float(cr * sp * cy + sr * cp * sy)
    unit_quaternion[3] = float(cr * cp * sy - sr * sp * cy)

    return unit_quaternion