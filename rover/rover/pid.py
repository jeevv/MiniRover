import rclpy 
from rclpy.node import Node

# Import odometry to get feedback from the ekf
from nav_msgs.msg import Odometry

# Import twist to read cmd_vel and provide final output to the STM32
from geometry_msgs.msg import Twist

# Defining a class to store PID constants, keep track of errors, and calculate PID output
class PID():
    '''
    Class used to store PID constants and perform PID output calculation

    Attributes:
        kp (float): Proportional constant
        kd (float): Derivative constant
        ki (float): Integral constant
        error_prev (float): Previous error. Used to calculate derivatives
        error_sum (float): Sum of the errors. Used to calculate integral
            
    '''

    def __init__(self,kp:float,kd:float,ki:float):
        '''
        Initialises a PID object

        Arguments:
            kp (float): Proportional constant
            kd (float): Derivative constant
            ki (float): Integral constant
        '''
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.error_prev = 0.0
        self.error_sum = 0.0

    def compute(self,target:float,current:float) ->float:
        '''
        A function that returns a calculated PID value for control

        Arguments:
            target (float): Desired value
            current (float): Current value obtained from feedback
        Returns:
            pid_output (float): Calculated PID output
        '''
        error = target - current

        pid_output = self.kp*error + self.kd*(error-self.error_prev) + self.ki*self.error_sum

        self.error_prev = error
        self.error_sum += error

        return pid_output
    

class PID_Control(Node):

    '''
    Class for a ROS2 node that implements PID control for linear x velocity and
    angular z velocity only

    Attributes:
        kp_linear (float): kp for linear x velocity
        kd_linear (float): kp for linear y velocity
        ki_linear (float): kp for linear z velocity
        kp_angular (float): kp for angular z velocity
        kd_angular (float): kd for angular z velocity
        ki_angular (float): ki for angular z velocity
        pid_publisher (Publisher): ROS2 publisher to publish pid output for both
        linear x and angular z velocity
        odom_subscriber (Subscription): ROS2 subscriber to read /odom for feedback
        cmd_vel_subscriber (Subscription): ROS2 subscriber to read /cmd_vel for 
        target velocity
        target (Twist): Twist message passed to the publisher. Updated by callbacks
    '''

    def __init__(self,kp_linear,kd_linear,ki_linear,kp_angular,kd_angular,ki_angular):
        '''
        Initialises a PID_Control node 

        Arguments:
            kp_linear (float): kp for linear x velocity
            kd_linear (float): kp for linear y velocity
            ki_linear (float): kp for linear z velocity
            kp_angular (float): kp for angular z velocity
            kd_angular (float): kd for angular z velocity
            ki_angular (float): ki for angular z velocity
        '''

        super().__init__('pid_controller')

        self.pid_linear = PID(kp=kp_linear,kd=kd_linear,ki=ki_linear)

        self.pid_angular = PID(kp=kp_angular,kd=kd_angular,ki=ki_angular)

        self.pid_publisher = self.create_publisher(Twist,'/pid_output',10)

        self.odom_subscriber = self.create_subscription(Odometry,'/filtered/odom',self.update_pid_output,10)

        self.cmd_vel_subscriber = self.create_subscription(Twist,'/cmd_vel',self.update_target,10)

        self.target = Twist()
    
    '''
    Callback function for /odom used to publish updated PID output using feedback from
    odometry topic

    Arguments:
        odomtery (Odomtery): Odometry data present in topic
    
    Returns:
        None
    '''
    def update_pid_output(self,odometry:Odometry):
        pid_output = Twist()

        pid_output.linear.x = self.pid_linear.compute(self.target.linear.x,odometry.twist.linear.x)
        pid_output.angular.z = self.pid_angular.compute(self.target.angular.z,odometry.twist.angular.z)

        # Publish updated PID output
        self.pid_publisher.publish(pid_output)

    '''
    Callback function for /cmd_vel used to update targets for velocities

    Arguments:
        cmd_vel (Twist): Desired velocity present in relevant topic
    
    Returns:
        None
    '''    
    def update_target(self,cmd_vel:Twist):
        self.target.linear.x = cmd_vel.linear.x
        self.target.angular.z = cmd_vel.angular.z
    

# Standard main function required by ros2
def main(args=None):
    rclpy.init(args=args)

    node = PID_Control(kp_linear=1,kd_linear=0,ki_linear=0,kp_angular=0.1,kd_angular=0,ki_angular=0)

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
