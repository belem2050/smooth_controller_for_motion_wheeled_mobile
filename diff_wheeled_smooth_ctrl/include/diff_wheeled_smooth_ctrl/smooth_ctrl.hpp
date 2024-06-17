#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"


class  SmoothCtrl: public rclcpp::Node
{
  
  public:
    SmoothCtrl(const std::string& name);

  private:
    float k1_;
    float k2_;
    float beta_;
    float lambda_;
    float v_max_;
    bool active_ = false;
    geometry_msgs::msg::Pose2D current_pose_;
    geometry_msgs::msg::Pose2D home_pose_ = geometry_msgs::msg::Pose2D();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    std::shared_ptr<rclcpp::TimerBase> cmd_vel_publisher_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activation_service_;

    geometry_msgs::msg::Pose2D pose_relative(geometry_msgs::msg::Pose2D &home_pose, geometry_msgs::msg::Pose2D &pose);

    void pose_subscriber_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
    void goal_subscriber_callback(const geometry_msgs::msg::PoseStamped & msg);
    void cmd_vel_publisher_timer_callback();
    void activation_service_callback(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    float mod_angle(float angle);
  };