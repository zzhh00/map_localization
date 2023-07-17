# Map_localization
## Input : 
- Topic: /odom_combined (nav_msgs::Odometry)
- TF: map->odom_combined (geometry_msgs::TransformStamped)


## Output:
- Topic: /map_odom_pose (geometry_msgs::PoseWithCovarianceStamped)
- TF: map->base_footprint (geometry_msgs::TransformStamped)

## Function : 
- Transform odometry from odom frame into map frame to improve the localization frequency (using transformation between map frame and odom frame published by AMCL);
- Broadcaster the tf between map frame and new odom frame to check whether the transformation is right;