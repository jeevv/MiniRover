#include "pid.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"  

#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class PID_ROS : public rclcpp::Node
{
  public:

    double linear_x, angular_z;

    double linear_x_prev, angular_z_prev;

    double target_linear_x, target_angular_z;

    PID linear_vel_pid, angular_vel_pid;

    PID_ROS(float kp_linear, float kd_linear, float ki_linear, float kp_angular, float kd_angular, float ki_angular)
    : Node("pid_node")
    {
      PID linear_vel_pid(kp_linear,kd_linear,ki_linear);

      PID angular_vel_pid(kp_angular,kd_angular,ki_angular);

      this->linear_vel_pid = linear_vel_pid;

      this->angular_vel_pid = angular_vel_pid;

      odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/filtered/odom",10,std::bind(&PID_ROS::odom_callback, this, _1));

      cmd_vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&PID_ROS::cmd_vel_callback, this, _1));

      pwm_publisher_left = this->create_publisher<std_msgs::msg::Int32>("/left_motor_pwm",10);

      pwm_publisher_left = this->create_publisher<std_msgs::msg::Int32>("/right_motor_pwm",10);

    }

  private:

    void odom_callback(const nav_msgs::msg::Odometry & msg)
    {
      float linear_pwm = linear_vel_pid.compute(msg.twist.twist.linear.x - linear_x_prev, (double) PID_ROS::now().nanoseconds()/1000000000);

      float angular_pwm = angular_vel_pid.compute(msg.twist.twist.angular.z - angular_z_prev, (double) PID_ROS::now().nanoseconds()/1000000000);

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

    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
      target_linear_x = msg.linear.x;
      target_angular_z = msg.angular.z;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pwm_publisher_left,pwm_publisher_right;

    std_msgs::msg::Int32 left_motor_pwm, right_motor_pwm;

    nav_msgs::msg::Odometry robot_odom;

    rclcpp::Clock clock;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PID_ROS>(0.0,0.0,0.0,0.0,0.0,0.0);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}