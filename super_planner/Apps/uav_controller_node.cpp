#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "mars_quadrotor_msgs/msg/position_command.hpp"

using namespace std::chrono_literals;

struct Setpoint
{
  Eigen::Vector3d pos;   // world frame
  Eigen::Vector3d vel;   // world frame
  Eigen::Vector3d accel; // world frame
  double yaw;
  double yaw_dot;
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("ControllerNode")
  {
    // Declare parameters
    this->declare_parameter("kff", 1.0);
    this->declare_parameter("kp", 1.0);
    this->declare_parameter("kd", 0.1);
    this->declare_parameter("z_scale", 1.0);
    this->declare_parameter("odom_in_body_frame", false);
    this->declare_parameter("control_rate", 10.0);
    this->declare_parameter("max_pos_err", 10.0);
    this->declare_parameter("kp_yaw", 1.0);
    this->declare_parameter("kff_yaw", 0.1);
    this->declare_parameter("max_yaw_cmd", 1.0);  // radians/sec
    this->declare_parameter("tolerance", 0.5);
    this->declare_parameter("control_accel", false);

    // Cache parameters once
    kff_ = this->get_parameter("kff").as_double();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    z_scale_ = this->get_parameter("z_scale").as_double();
    odom_in_body_frame_ = this->get_parameter("odom_in_body_frame").as_bool();
    control_rate_ = this->get_parameter("control_rate").as_double();
    max_pos_err_ = this->get_parameter("max_pos_err").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    kff_yaw_ = this->get_parameter("kff_yaw").as_double();
    max_yaw_cmd = this->get_parameter("max_yaw_cmd").as_double();
    tolerance_ = this->get_parameter("tolerance").as_double();
    flag_control_accel_ = this->get_parameter("control_accel").as_bool();

    // Subscribers
    setpoint_sub_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>("setpoint", 10, std::bind(&ControllerNode::setpointCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    if (flag_control_accel_) cmd_accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("cmd_accel", 10);
    else cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

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
    
    setpoint_.accel = Eigen::Vector3d(
      msg->acceleration.x,
      msg->acceleration.y,
      msg->acceleration.z);

    setpoint_.yaw = msg->yaw;
    setpoint_.yaw_dot = msg->yaw_dot;

    sp_received_ = true;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_world_ = Eigen::Vector3d(
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
    geometry_msgs::msg::Twist cmd_vel;
    geometry_msgs::msg::Accel cmd_accel;

    if (!sp_received_ || !odom_received_) { return; }

    // Position error in world frame
    Eigen::Vector3d pos_err_w = setpoint_.pos - pos_world_;

    double norm = pos_err_w.norm();
    if (norm < tolerance_) { return; }
    if (norm > max_pos_err_) pos_err_w *= (max_pos_err_ / norm);

    // Transform errors and setpoints into body frame
    Eigen::Vector3d pos_err_b = q_.conjugate() * pos_err_w;
    Eigen::Vector3d sp_vel_b  = q_.conjugate() * setpoint_.vel;

    if (odom_in_body_frame_) {
      vel_body_ = vel_meas_;
      vel_world_ = q_*vel_body_;
    } else {
      vel_world_ = vel_meas_;
      vel_body_ = q_.conjugate() * vel_world_;  // world -> body
      // Eigen::Vector3d vel_err_b = sp_vel_b - vel_body_;
    }

    Eigen::Vector3d ctrl_lin;

    if (!flag_control_accel_) ctrl_lin = kff_*sp_vel_b + kp_*pos_err_b - kd_*vel_body_;
    else ctrl_lin = kff_*setpoint_.accel + kp_*pos_err_w + kd_*(setpoint_.vel-vel_world_) - kd_*0.5*vel_world_;
    ctrl_lin.z() *= z_scale_;

    // Desired yaw quaternion (rotation about Z axis)
    Eigen::Quaterniond q_setpoint_(Eigen::AngleAxisd(setpoint_.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_err = q_.conjugate() * q_setpoint_;
    double yaw_err = std::atan2(2.0 * (q_err.w()*q_err.z() + q_err.x()*q_err.y()),
                                1.0 - 2.0 * (q_err.y()*q_err.y() + q_err.z()*q_err.z()));
    // Saturation for yaw rate
    double yaw_cmd = kp_yaw_ * yaw_err + kff_yaw_ * setpoint_.yaw_dot;
    if (yaw_cmd > max_yaw_cmd) yaw_cmd = max_yaw_cmd;
    if (yaw_cmd < -max_yaw_cmd) yaw_cmd = -max_yaw_cmd;

    if (!flag_control_accel_){
      // Publish Twist
      cmd_vel.linear.x = ctrl_lin.x();
      cmd_vel.linear.y = ctrl_lin.y();
      cmd_vel.linear.z = ctrl_lin.z();
      cmd_vel.angular.z = yaw_cmd;  // yaw control feedforward
      cmd_vel_pub_->publish(cmd_vel);
    } else {
      // Publish Accel
      cmd_accel.linear.x = ctrl_lin.x();
      cmd_accel.linear.y = ctrl_lin.y();
      cmd_accel.linear.z = ctrl_lin.z();
      cmd_accel.angular.z = yaw_cmd;  // yaw control feedforward
      cmd_accel_pub_->publish(cmd_accel);
    }

    sp_received_ = false;
  }

  // Cached parameters
  double kff_, kp_, kd_, z_scale_, control_rate_, max_pos_err_, kp_yaw_, kff_yaw_, max_yaw_cmd, tolerance_;
  bool odom_in_body_frame_, flag_control_accel_, odom_received_{false}, sp_received_{false};

  // State
  Eigen::Quaterniond q_{1, 0, 0, 0};
  Eigen::Vector3d pos_world_{0, 0, 0};
  Eigen::Vector3d vel_meas_{0, 0, 0};
  Eigen::Vector3d vel_body_{0, 0, 0};
  Eigen::Vector3d vel_world_{0, 0, 0};
  Setpoint setpoint_{};

  // ROS2 interfaces
  rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr cmd_accel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

