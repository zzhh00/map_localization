# Map_localization
## Input : 
- Topic: /odom (nav_msgs::msg::Odometry)
- TF: map->odom (geometry_msgs::msg::TransformStamped)


## Output:
- Topic: /map_odom_pose (geometry_msgs::msg::PoseWithCovarianceStamped)
- TF: map->base_footprint (geometry_msgs::msg::TransformStamped)

## Function : 
- Transform odometry from odom frame into map frame to improve the localization frequency (using transformation between map frame and odom frame published by AMCL);
- Broadcaster the tf between map frame and new odom frame to check whether the transformation is right;