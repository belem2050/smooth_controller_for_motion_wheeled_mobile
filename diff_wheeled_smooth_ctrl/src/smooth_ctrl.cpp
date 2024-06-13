#include "smooth_ctrl.hpp"
#include <cmath>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

SmoothCtrl::SmoothCtrl(const std::string& name)
: Node(name)
{
  this->k1_ = this->declare_parameter<float>("k1_", 1);
  this->k2_ = this->declare_parameter<float>("k2_", 3);
  this->beta_ = this->declare_parameter<float>("beta_", 0.4);
  this->v_max_ = this->declare_parameter<float>("v_max_", 0.5);
  this->active_ = this->declare_parameter<bool>("active_", false);
  this->home_pose_ = geometry_msgs::msg::Pose2D();

  cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_mux/input/teleop", 1);
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::QoS(1).best_effort(),
    std::bind(&SmoothCtrl::pose_subscriber_callback, this, std::placeholders::_1)
  );
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "move_base_simple/goal",
    rclcpp::QoS(1).best_effort(),
    std::bind(&SmoothCtrl::goal_subscriber_callback, this, std::placeholders::_1)
  );
  cmd_vel_publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SmoothCtrl::cmd_vel_publisher_timer_callback, this));

}

geometry_msgs::msg::Pose2D 
SmoothCtrl::pose_relative(geometry_msgs::msg::Pose2D &home_pose, geometry_msgs::msg::Pose2D &pose)
{
  geometry_msgs::msg::Pose2D pose_relative;

  float ca = cos(pose.theta);
  float sa = sin(pose.theta);
  float px = home_pose.x - pose.x;
  float py = home_pose.y - pose.y;
  float pa = home_pose.theta - pose.theta;
 
  pose_relative.x = ca*px + sa *py;
  pose_relative.y = -sa*px + ca*py;
  pose_relative.theta = this->mod_angle(pa);

  return pose_relative;
}

void 
SmoothCtrl::pose_subscriber_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  this->current_pose_ = geometry_msgs::msg::Pose2D();
  current_pose_.x = msg.pose.pose.position.x;
  current_pose_.y = msg.pose.pose.position.y;
  current_pose_.theta = this->mod_angle(tf2::getYaw(msg.pose.pose.orientation));
}

void 
SmoothCtrl::goal_subscriber_callback(const geometry_msgs::msg::PoseStamped & msg)
{
  home_pose_.x = msg.pose.position.x;
  home_pose_.y = msg.pose.position.y;
  home_pose_.theta = this->mod_angle(tf2::getYaw(msg.pose.orientation));
  active_ = true;
  RCLCPP_INFO(this->get_logger(), "home_pose.x = %f, home_pose.y = %f, home_pose.yaw = %f", home_pose_.x, home_pose_.y, home_pose_.theta);
}

void 
SmoothCtrl::cmd_vel_publisher_timer_callback()
{
  if(!active_)
    return;
  
  geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
  
  geometry_msgs::msg::Pose2D pose_relative = this->pose_relative(home_pose_, current_pose_);

  float dr = std::hypot(pose_relative.x, pose_relative.y);

  float sigma = mod_angle(std::atan2(pose_relative.y, pose_relative.x));
  float theta = mod_angle(sigma - pose_relative.theta);

  float part1 = k2_ * (-sigma - std::atan(k1_*theta));
  float part2 = (1 + k1_ / (1 + std::pow((k1_ * theta), 2)) * sin(-sigma));
  
  float v = 0.0;
  float w = 0.0;
  float k = 0.0;

  if (dr != 0.0) // keep 0 as linear vel if no distance to travel
  {
    k = (part1 + part2) / -dr;
    v = v_max_ / (1 + beta_ * std::pow(std::abs(k), lambda_));
  }

  if (pose_relative.theta != 0.0) // keep 0 as angular vel if no distance to travel
  {
    w = k *v;
  }

  RCLCPP_INFO(this->get_logger(), "dr = %f, sigma = %f, theta = %f, k = %f, v = %f, w = %f", dr, sigma, theta, k, v, w);

  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w;
  cmd_vel_publisher->publish(cmd_vel);
}

float 
SmoothCtrl::mod_angle(float angle)
{
  angle = fmod(angle, 2*M_PI);
  
  if(angle >= M_PI) 
    angle = angle - 2* M_PI;
    
  return angle;
}