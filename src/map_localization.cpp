#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

class MapLocalization
{
public:
  MapLocalization() : tf_buffer_(ros::Duration(2.0)), tf_listener_(tf_buffer_)
  {
    sub_ = nh_.subscribe("/odom_combined", 10, &MapLocalization::poseCallback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/map_odom_pose", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = msg->header;
    odom_pose.pose = msg->pose.pose;

    try
    {
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform("map", "odom_combined", ros::Time(0));

      ROS_INFO_STREAM("Translation from map frame to odom_combined frame - x: " << transform_stamped.transform.translation.x
                      << " y: " << transform_stamped.transform.translation.y
                      << " z: " << transform_stamped.transform.translation.z);
      ROS_INFO_STREAM("Orientation - x: " << transform_stamped.transform.rotation.x
                      << " y: " << transform_stamped.transform.rotation.y
                      << " z: " << transform_stamped.transform.rotation.z
                      << " w: " << transform_stamped.transform.rotation.w);

      geometry_msgs::PoseStamped map_pose;
      tf2::doTransform(odom_pose, map_pose, transform_stamped);

      // 创建发布的消息
      geometry_msgs::PoseWithCovarianceStamped map_pose_msg;
      map_pose_msg.header.stamp = ros::Time::now();
      map_pose_msg.header.frame_id = "map";
      map_pose_msg.pose.pose = map_pose.pose;

      pub_.publish(map_pose_msg);

      ROS_INFO_STREAM("Base_footprint pose in map frame - x: " << map_pose.pose.position.x
                      << " y: " << map_pose.pose.position.y
                      << " z: " << map_pose.pose.position.z);
      ROS_INFO_STREAM("Orientation - x: " << map_pose.pose.orientation.x
                      << " y: " << map_pose.pose.orientation.y
                      << " z: " << map_pose.pose.orientation.z
                      << " w: " << map_pose.pose.orientation.w
                      << "\n");

      if(1)
      {
        geometry_msgs::TransformStamped tmp_transform_stamped;
        tmp_transform_stamped.header = map_pose_msg.header;
        tmp_transform_stamped.child_frame_id = "new_odom_combined_pose";
        tmp_transform_stamped.transform.translation.x = map_pose_msg.pose.pose.position.x;
        tmp_transform_stamped.transform.translation.y = map_pose_msg.pose.pose.position.y;
        tmp_transform_stamped.transform.translation.z = map_pose_msg.pose.pose.position.z;
        tmp_transform_stamped.transform.rotation.x = map_pose_msg.pose.pose.orientation.x;
        tmp_transform_stamped.transform.rotation.y = map_pose_msg.pose.pose.orientation.y;
        tmp_transform_stamped.transform.rotation.z = map_pose_msg.pose.pose.orientation.z;
        tmp_transform_stamped.transform.rotation.w = map_pose_msg.pose.pose.orientation.w;

        tf_broadcaster_.sendTransform(tmp_transform_stamped);
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Failed to lookup transform: " << ex.what());
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_localization");

  MapLocalization map_localization;

  ros::spin();

  return 0;
}
