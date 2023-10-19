#include "simulator/transform.hpp"

namespace simulator
{
Transform::Transform() : rclcpp::Node("robot"){
    declare_parameter("velocity_topic", "cmd_vel");
    declare_parameter("odometry_topic", "odom");
    declare_parameter("robot_frame_id", "base_link");
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("world_frame_id", "map");
    declare_parameter("pose_x", 0.0);
    declare_parameter("pose_y", 0.0);
    declare_parameter("pose_yaw", 0.0);

    double yaw;
    initial_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    get_parameter("velocity_topic", velocity_topic_);
    get_parameter("odometry_topic", odometry_topic_);
    get_parameter("robot_frame_id", robot_frame_id_);
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("world_frame_id", world_frame_id_);
    get_parameter("pose_x", initial_pose_->pose.pose.position.x);
    get_parameter("pose_y", initial_pose_->pose.pose.position.y);
    get_parameter("pose_yaw", yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    initial_pose_->pose.pose.orientation = tf2::toMsg(q);
    initial_pose_->header.frame_id = world_frame_id_;
    initial_pose_->header.stamp = now();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(velocity_topic_, 5,
                         std::bind(&Transform::velCallback, this, _1));

    last_update_moment_ = now();
    last_vel_moment_ = last_update_moment_ - rclcpp::Duration(0.1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    odom_.header.stamp = last_update_moment_;
    
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                            "initialpose", 5, std::bind(&Transform::initPoseCallback, this, _1));
    map_odom_trans_.header.frame_id = world_frame_id_;
    map_odom_trans_.header.stamp = last_update_moment_;
    map_odom_trans_.child_frame_id = odom_frame_id_;
    map_odom_trans_.transform.rotation.w = 1.0;
    
    loop_timer_ = this->create_wall_timer(20ms, std::bind(&Transform::updateLoop, this));

    initPoseCallback(initial_pose_);
}

Transform::~Transform(){}

void Transform::updateLoop(){
    last_update_moment_ = now();
    if (!message_received_)
    {
        odom_.header.stamp = last_update_moment_;
        odom_baselink_trans_.header.stamp = last_update_moment_;
    }
    
    odom_pub_->publish(odom_);
    getTfFromOdom(odom_);
    tf_broadcaster_->sendTransform(odom_baselink_trans_);
    message_received_ = false;
    map_odom_trans_.header.stamp = last_update_moment_;
    tf_broadcaster_->sendTransform(map_odom_trans_);    
}

void Transform::updateOdomFromVel(geometry_msgs::msg::Twist vel, rclcpp::Duration time_diff){
    double dt = time_diff.seconds();
    double delta_x = (vel.linear.x * cos(th_) - vel.linear.y * sin(th_)) * dt;
    double delta_y = (vel.linear.x * sin(th_) + vel.linear.y * cos(th_)) * dt;
    double delta_th = vel.angular.z * dt;
    odom_.pose.pose.position.x += delta_x;
    odom_.pose.pose.position.y += delta_y;
    th_ += delta_th;
    odom_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), th_));
    odom_.twist.twist = vel;
    odom_.header.stamp = last_update_moment_;
    odom_.header.frame_id = odom_frame_id_;
}
void Transform::getTfFromOdom(nav_msgs::msg::Odometry odom){
    geometry_msgs::msg::TransformStamped odom_tmp;
    
    odom_tmp.header = odom.header;
    odom_tmp.header.frame_id = odom_frame_id_;
    odom_tmp.child_frame_id = robot_frame_id_;
    odom_tmp.transform.translation.x = odom.pose.pose.position.x;
    odom_tmp.transform.translation.y = odom.pose.pose.position.y;
    odom_tmp.transform.translation.z = 0.0;
    odom_tmp.transform.rotation = odom.pose.pose.orientation;
    odom_baselink_trans_ = odom_tmp;  
}
void Transform::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    last_vel_moment_ = now();
    message_received_ = true;
    updateOdomFromVel(*msg, last_vel_moment_ - last_update_moment_);
}
void Transform::initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    if (msg->header.frame_id.find(world_frame_id_) == msg->header.frame_id.npos)
    {
        RCLCPP_ERROR(get_logger(), "Initial pose not specified in map frame, ignoring");
        return;
    }
    RCLCPP_INFO(get_logger(), "Received pose estimate of mobile base");

    tf2::Transform tx_odom_tf2, pose_old;
    tf2::impl::Converter<true, false>::convert(odom_baselink_trans_.transform, tx_odom_tf2);
    tf2::impl::Converter<true, false>::convert(msg->pose.pose, pose_old);
    tf2::Transform pose_new = pose_old * tx_odom_tf2.inverse();
    
    map_odom_trans_.header.stamp = now();
    map_odom_trans_.header.frame_id = world_frame_id_;
    map_odom_trans_.child_frame_id = odom_frame_id_;
    map_odom_trans_.transform.translation.x = pose_new.getOrigin().x();
    map_odom_trans_.transform.translation.y = pose_new.getOrigin().y();
    map_odom_trans_.transform.translation.z = pose_new.getOrigin().z();
    map_odom_trans_.transform.rotation.x = pose_new.getRotation().x();
    map_odom_trans_.transform.rotation.y = pose_new.getRotation().y();
    map_odom_trans_.transform.rotation.z = pose_new.getRotation().z();
    map_odom_trans_.transform.rotation.w = pose_new.getRotation().w();
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simulator::Transform>());
  rclcpp::shutdown();
  return 0;
}
