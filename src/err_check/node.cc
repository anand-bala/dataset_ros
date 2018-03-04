#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

void depthCb (const sensor_msgs::ImageConstPtr &depth) {
  static int count = 0;
  int zero = 0;
  float empty_percent = 0.0;

  if(count >= 1000) {
    ROS_INFO("Processed 1000 more /camera/depth_registered/image_raw");
    count = 0;
  }

  for (auto it = depth->data.begin(); it != depth->data.end(); it++) {
    if (*it == 0)
      zero++;
  }

  count++;
  empty_percent = float(zero)/float(depth->data.size()); 
  if (empty_percent > 0.80) {
    ROS_ERROR("Most of the Depth Image is Empty!");
    ros::shutdown();
  }
}

void velodyneCb (const sensor_msgs::PointCloud2ConstPtr &cloud) {
  static int count = 0;
  int zero = 0;
  float empty_percent = 0.0;

  if(count >= 1000) {
    ROS_INFO("Processed 1000 more /velodyne_points");
    count = 0;
  }

  for (auto it = cloud->data.begin(); it != cloud->data.end(); it++) {
    if (*it == 0)
      zero++;
  }
  
  count++;
  empty_percent = float(zero)/float(cloud->data.size()); 
  if (empty_percent > 0.80) {
    ROS_ERROR("Most of the Depth Image is Empty!");
    ros::shutdown();
  }

}


ros::Subscriber depthSub;
ros::Subscriber velodyneSub;

int main (int argc, char **argv) {
  ros::init(argc, argv, "err_check");
  ros::NodeHandle nh;

  ROS_INFO("Begin ErrCheckNode");

  depthSub = nh.subscribe("/camera/depth_registered/image_raw", 10,
      &depthCb);
  velodyneSub = nh.subscribe("/velodyne_points", 10,
      &velodyneCb);
  ros::spin();
  return 0;
}

