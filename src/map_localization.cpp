#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

class MapLocalization : public rclcpp::Node
{
public:
  MapLocalization() : Node("map_localization")
  {
    flag_tf_publish = true;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&MapLocalization::poseCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/map_odom_pose", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool flag_tf_publish;

  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

      RCLCPP_INFO(this->get_logger(), "Translation from map to odom frame - x: %f y: %f z: %f",
                  transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z);
      RCLCPP_INFO(this->get_logger(), "Orientation - x: %f y: %f z: %f w: %f",
                  transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                  transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);

      geometry_msgs::msg::PoseStamped map_pose;
      tf2::doTransform(odom_pose, map_pose, transform_stamped);

      // 创建发布的消息
      auto map_pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      map_pose_msg->header.stamp = this->now();
      map_pose_msg->header.frame_id = "map";
      map_pose_msg->pose.pose = map_pose.pose;

      pub_->publish(*map_pose_msg);

      RCLCPP_INFO(this->get_logger(), "Base_footprint pose in map frame - x: %f y: %f z: %f",
                  map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Orientation - x: %f y: %f z: %f w: %f \n",
                  map_pose.pose.orientation.x, map_pose.pose.orientation.y,
                  map_pose.pose.orientation.z, map_pose.pose.orientation.w);

      if(flag_tf_publish)
      {
        geometry_msgs::msg::TransformStamped tmp_map_to_odom;
        tmp_map_to_odom.header = map_pose_msg->header;
        tmp_map_to_odom.child_frame_id = "new_odom_link";
        tmp_map_to_odom.transform.translation.x = map_pose.pose.position.x;
        tmp_map_to_odom.transform.translation.y = map_pose.pose.position.y;
        tmp_map_to_odom.transform.translation.z = map_pose.pose.position.z;
        tmp_map_to_odom.transform.rotation.x = map_pose.pose.orientation.x;
        tmp_map_to_odom.transform.rotation.y = map_pose.pose.orientation.y;
        tmp_map_to_odom.transform.rotation.z = map_pose.pose.orientation.z;
        tmp_map_to_odom.transform.rotation.w = map_pose.pose.orientation.w;
        tf_broadcaster_->sendTransform(tmp_map_to_odom);
      }
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapLocalization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
