#include "pid.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"  

#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

/*
Class used to implement PID control for a 4 wheel skid steer robot using ROS2

Utilises the PID class to seperate control of linear and angular velocities. The left wheels and 
right wheels are assumed to have the same velocity

Paramters:
(double) linear_x, angular_z: Used to store the velocity of the bot from the odometry
(double) target_linear_x, target_linear_z: Target velocity from cmd_vel
(PID) linear_vel_pid, angular_z_pid: Used for the PID control of linear and angular velocity
*/
class PID_ROS : public rclcpp::Node
{
  public:

    double linear_x, angular_z;

    double target_linear_x, target_angular_z;

    PID linear_vel_pid, angular_vel_pid;

    /*
    Constructor used to initialise the ROS2 node and PID constant parameters

    Parameters:
    (double) kp_linear,kd_linear,ki_linear: PID constants for linear velocity
    (double) kp_angular,kd_angular, ki_angular: PID constants for angular velocity

    (std_msgs::msg::Int32) left_motor_pwm, right_motor_pwm: ROS2 messages publsihed to topics 
    to  control motor PWM. Capped at 1000 and always postitive.

    (rclcpp::Subscription) odom_subscription, cmd_vel_subscription: ROS2 subscribers 
    for odometry and cmd_vel
    (rclcpp::Publisher) pwm_publisher_left, pwm_publisher_right: ROS2 publishers for motor PWM
    */

    PID_ROS(double kp_linear, double kd_linear, double ki_linear, double kp_angular, double kd_angular, double ki_angular)
    : Node("pid_node")
    {
      // Initialising two PID variables and copying them into the class paramters. Might need 
      // different initialisation

      PID linear_vel_pid(kp_linear,kd_linear,ki_linear);

      PID angular_vel_pid(kp_angular,kd_angular,ki_angular);

      this->linear_vel_pid = linear_vel_pid;

      this->angular_vel_pid = angular_vel_pid;

      odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/filtered/odom",10,std::bind(&PID_ROS::odom_callback, this, _1));

      cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&PID_ROS::cmd_vel_callback, this, _1));

      pwm_publisher_left = this->create_publisher<std_msgs::msg::Int32>("/left_motor_pwm",10);

      pwm_publisher_right = this->create_publisher<std_msgs::msg::Int32>("/right_motor_pwm",10);

    }

  private:
    /*
    Function used to compute linear and angular PID using odomtery and target velocity
    */
    void odom_callback(const nav_msgs::msg::Odometry & msg)
    {
      double linear_pwm = linear_vel_pid.compute(target_linear_x - msg.twist.twist.linear.x, (double) PID_ROS::now().nanoseconds()/1000000000);

      double angular_pwm = angular_vel_pid.compute(target_angular_z - msg.twist.twist.angular.z, (double) PID_ROS::now().nanoseconds()/1000000000);

      left_motor_pwm.data = (int32_t) (linear_pwm - angular_pwm);

      right_motor_pwm.data = (int32_t) (linear_pwm + angular_pwm);

      if(left_motor_pwm.data<0)
      {
        left_motor_pwm.data = 0;
      }

      else if(left_motor_pwm.data>1000)
      {
        left_motor_pwm.data = 1000;
      }

      if(right_motor_pwm.data<0)
      {
        right_motor_pwm.data = 0;
      }

      else if(right_motor_pwm.data>1000)
      {
        right_motor_pwm.data = 1000;
      }

      pwm_publisher_left->publish(left_motor_pwm);

      pwm_publisher_right->publish(right_motor_pwm);

    }

    /*
    Function used to update target velocity from cmd_vel
    */
    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
      target_linear_x = msg.linear.x;
      target_angular_z = msg.angular.z;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pwm_publisher_left,pwm_publisher_right;

    std_msgs::msg::Int32 left_motor_pwm, right_motor_pwm;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PID_ROS>(0.0,0.0,0.0,0.0,0.0,0.0);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}