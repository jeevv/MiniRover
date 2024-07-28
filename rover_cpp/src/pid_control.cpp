#include "pid.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"  

#include "std_msgs/msg/int32.hpp"

#include "std_msgs/msg/int32_multi_array.hpp"

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

    (std_msgs::msg::Int32MultiArray) motor_pwm: ROS2 message that contains left and right motor pwm
    to  control motor PWM. Left motor at 0 index and right motor at 1 index. Capped at 1000 and always postitive.

    (rclcpp::Subscription) odom_subscription, cmd_vel_subscription: ROS2 subscribers 
    for odometry and cmd_vel
    (rclcpp::Publisher) pwm_publisher: ROS2 publisher for motor PWM
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

      pwm_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("/motor_pwm",10);

    }

  private:
    /*
    Function used to compute linear and angular PID using odomtery and target velocity
    */
    void odom_callback(const nav_msgs::msg::Odometry & msg)
    {
      double linear_pwm = linear_vel_pid.compute(target_linear_x - msg.twist.twist.linear.x, (double) PID_ROS::now().nanoseconds()/1000000000);

      double angular_pwm = angular_vel_pid.compute(target_angular_z - msg.twist.twist.angular.z, (double) PID_ROS::now().nanoseconds()/1000000000);

      int pid_output[4] = {0,0,0,0};

      pid_output[0] = (int32_t) (linear_pwm - angular_pwm);

      pid_output[2] = (int32_t) (linear_pwm + angular_pwm);

      // Convert both PID outputs, same variable can be used for both conversions 
      // as the function only operates on start_index and start_index + 1 leaving the
      // remaining data untouched

      convert_pid_output(pid_output,0);
      convert_pid_output(pid_output,2);

      // Not sure why standard assignment doesn't work but this solution does
      for(int i = 0;i<4;i++)
      {
        motor_pwm.data[i] = pid_output[i];
      }

      pwm_publisher->publish(motor_pwm);

    }

    /*
    Function used to update target velocity from cmd_vel
    */
    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
      target_linear_x = msg.linear.x;
      target_angular_z = msg.angular.z;
    }

    /*
    Function used to convert the PID output into PWM for the motors, value always between 0 and 1000
    due to STM32 PWM setup. The PWM is written to two indices, the start_index and start_index + 1 which can
    be passed as arguments for increased flexibility. Might need variable name change.
    */

    void convert_pid_output(int *pid_output, int start_index)
    {
      if(pid_output[start_index] < 0)
      {
        if(pid_output[start_index] < -1000)
        {
          pid_output[start_index+1] = 1000;
        }

        else
        {
          pid_output[start_index+1] = -pid_output[0];
        }

        pid_output[start_index] = 0;

      }

      else if(pid_output[start_index] >= 0)
      {
        if(pid_output[start_index] > 1000)
        {
          pid_output[start_index] = 1000;
        }

        // Else statement in not required as PID output is a positive value less than 1000 
        // and does not need changing 

        pid_output[start_index+1] = 0;

      }

    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_publisher;

    std_msgs::msg::Int32MultiArray motor_pwm;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PID_ROS>(0.0,0.0,0.0,0.0,0.0,0.0);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}