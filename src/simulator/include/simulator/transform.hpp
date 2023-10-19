#ifndef __SIMULATOR_TRANSFORM_HPP__
#define __SIMULATOR_TRANSFORM_HPP__

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "rclcpp/timer.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace simulator {

using std::placeholders::_1;    
using namespace std::literals::chrono_literals;
class Transform : public rclcpp::Node
{
public:
    Transform(/* args */);
    ~Transform();

protected:
    void updateLoop();
    void updateOdomFromVel(geometry_msgs::msg::Twist vel, rclcpp::Duration time_diff);
    void getTfFromOdom(nav_msgs::msg::Odometry odom);
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::TransformStamped odom_baselink_trans_;
    geometry_msgs::msg::TransformStamped map_odom_trans_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Time last_vel_moment_;
    rclcpp::Time last_update_moment_;
    rclcpp::Time measure_time_;
    bool message_received_ = false;
    
    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::string velocity_topic_;
    std::string odometry_topic_;
    std::string robot_frame_id_, odom_frame_id_, world_frame_id_;
    
    rclcpp::TimerBase::SharedPtr loop_timer_;
    double th_ = 0.0;
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_;
};

} // namespace simulator

#endif // __SIMULATOR_TRANSFORM_HPP__