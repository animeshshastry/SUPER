#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mars_quadrotor_msgs/msg/position_command.hpp"

using namespace std::chrono_literals;

struct Setpoint
{
  Eigen::Vector3d pos;   // world frame
  Eigen::Vector3d vel;   // world frame
  double yaw;
  double yaw_dot;
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("ControllerNode")
  {
    // Declare parameters
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("kd", 0.1);
    this->declare_parameter("z_scale", 1.0);
    this->declare_parameter("odom_in_body_frame", false);
    this->declare_parameter("control_rate", 10.0);
    this->declare_parameter("max_cmd_lin", 1.0);
    this->declare_parameter("kp_yaw", 1.0);
    this->declare_parameter("kd_yaw", 0.1);
    this->declare_parameter("max_yaw_cmd", 1.0);  // radians/sec

    // Cache parameters once
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    z_scale_ = this->get_parameter("z_scale").as_double();
    odom_in_body_frame_ = this->get_parameter("odom_in_body_frame").as_bool();
    control_rate_ = this->get_parameter("control_rate").as_double();
    max_cmd_lin_ = this->get_parameter("max_cmd_lin").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    kd_yaw_ = this->get_parameter("kd_yaw").as_double();
    max_yaw_cmd = this->get_parameter("max_yaw_cmd").as_double();

    // Subscribers
    setpoint_sub_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>("setpoint", 10, std::bind(&ControllerNode::setpointCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Timer for control loop
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_), std::bind(&ControllerNode::controlLoop, this));
  }

private:

  void setpointCallback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    // Desired position & velocity (world)
    setpoint_.pos = Eigen::Vector3d(
      msg->position.x,
      msg->position.y,
      msg->position.z);

    setpoint_.vel = Eigen::Vector3d(
      msg->velocity.x,
      msg->velocity.y,
      msg->velocity.z);

    setpoint_.yaw = msg->yaw;
    setpoint_.yaw_dot = msg->yaw_dot;

    sp_received_ = true;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Vector3d pos_world_(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    q_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

    vel_meas_ = Eigen::Vector3d(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);

    odom_received_ = true;
  }

  void controlLoop()
  {
    if (!sp_received_ || !odom_received_) {
      geometry_msgs::msg::Twist cmd;
      cmd_pub_->publish(cmd);
      return;
    }

    // Position error in world frame
    Eigen::Vector3d pos_err_w = setpoint_.pos - pos_world_;

    // Transform errors and setpoints into body frame
    Eigen::Vector3d pos_err_b = q_.conjugate() * pos_err_w;
    Eigen::Vector3d sp_vel_b  = q_.conjugate() * setpoint_.vel;

    // Use odom velocity either in body or world frame
    if (odom_in_body_frame_) {
      vel_body_ = vel_meas_;
    } else {
      vel_body_ = q_.conjugate() * vel_meas_;  // world -> body
    }

    // Velocity error in body frame
    Eigen::Vector3d vel_err_b = sp_vel_b - vel_body_;

    // PD control
    Eigen::Vector3d ctrl_lin = kp_ * pos_err_b + kd_ * vel_err_b;
    ctrl_lin.z() *= z_scale_;

    // Radial saturation
    double norm = ctrl_lin.norm();
    if (norm > max_cmd_lin_) {
      ctrl_lin *= (max_cmd_lin_ / norm);
    }

    // Desired yaw quaternion (rotation about Z axis)
    Eigen::Quaterniond q_setpoint_(Eigen::AngleAxisd(setpoint_.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_err = q_.conjugate() * q_setpoint_;
    double yaw_err = std::atan2(2.0 * (q_err.w()*q_err.z() + q_err.x()*q_err.y()),
                                1.0 - 2.0 * (q_err.y()*q_err.y() + q_err.z()*q_err.z()));
    // Saturation for yaw rate
    double yaw_cmd = kp_yaw_ * yaw_err + kd_yaw_ * setpoint_.yaw_dot;
    if (yaw_cmd > max_yaw_cmd) yaw_cmd = max_yaw_cmd;
    if (yaw_cmd < -max_yaw_cmd) yaw_cmd = -max_yaw_cmd;

    // Publish Twist
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = ctrl_lin.x();
    cmd.linear.y = ctrl_lin.y();
    cmd.linear.z = ctrl_lin.z();
    cmd.angular.z = - yaw_cmd;  // yaw control feedforward
    cmd_pub_->publish(cmd);

    sp_received_ = false;
  }

  // Cached parameters
  double kp_, kd_, z_scale_, control_rate_, max_cmd_lin_, kp_yaw_, kd_yaw_, max_yaw_cmd;
  bool odom_in_body_frame_, odom_received_{false}, sp_received_{false};

  // State
  Eigen::Quaterniond q_{1, 0, 0, 0};
  Eigen::Vector3d pos_world_{0, 0, 0};
  Eigen::Vector3d vel_meas_{0, 0, 0};
  Eigen::Vector3d vel_body_{0, 0, 0};
  Setpoint setpoint_{};

  // ROS2 interfaces
  rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

