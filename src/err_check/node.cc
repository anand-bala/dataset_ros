#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <manual_measurements/WifiInfo.h>

ros::Subscriber depthSub;
ros::Subscriber velodyneSub;
ros::Subscriber ap1Sub;
ros::Subscriber ap2Sub;

ros::Timer ap1Timer;
ros::Timer ap2Timer;
ros::Time last_ap1;
ros::Time last_ap2;

void depthCb (const sensor_msgs::ImageConstPtr &depth) {
  static int count = 0;
  int zero = 0;
  float empty_percent = 0.0;

  if(count >= 500) {
    ROS_INFO("Processed 500 more /camera/depth_registered/image_raw");
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
  }
}

void velodyneCb (const sensor_msgs::PointCloud2ConstPtr &cloud) {
  static int count = 0;
  int zero = 0;
  float empty_percent = 0.0;

  if(count >= 500) {
    ROS_INFO("Processed 500 more /velodyne_points");
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
  }
}

void apInfoCb (const manual_measurements::WifiInfo &wifi) {
  last_ap1 = ros::Time::now();
	static int count = 0;
  if (wifi.accesspoint.size() < 3) {
    ROS_ERROR("[APInfo] Something is up! Received only %lu AP's", wifi.accesspoint.size());
  }
  if (count > 20) {
	ROS_INFO("[APInfo] Got 20 more");
	count = 0;
  }
  count++;
}

void apInfo2Cb (const manual_measurements::WifiInfo &wifi) {
  last_ap2 = ros::Time::now();
  static int count = 0;
  if (wifi.accesspoint.size() < 3) {
    ROS_ERROR("[APInfo2] Something is up! Received only %lu AP's", wifi.accesspoint.size());
  }
  if (count > 5) {
	ROS_INFO("[APInfo] Got 5 more");
	count = 0;
  }
  count++;
} 

void ap1TimerCb (const ros::TimerEvent& e) {
  ros::Time cur = e.current_real;
  ros::Duration diff(2.0);
  if(cur - last_ap1 >= diff) {
    ROS_ERROR("Haven't recieved an APInfo for more than 2 second!");
  }
}

void ap2TimerCb (const ros::TimerEvent& e) {
  ros::Time cur = e.current_real;
  ros::Duration diff(5.0);
  if(cur - last_ap2 > diff) {
    ROS_ERROR("Haven't recieved an APInfo2 for more than 2 second!");
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "err_check");
  ros::NodeHandle nh;

  ROS_INFO("Begin ErrCheckNode");

  depthSub = nh.subscribe("/camera/depth_registered/image_raw", 10,
      &depthCb);
  velodyneSub = nh.subscribe("/velodyne_points", 10,
      &velodyneCb);

  ap1Sub = nh.subscribe("/APInfo", 5, &apInfoCb);
  ap2Sub = nh.subscribe("/APInfo2", 5, &apInfo2Cb);

  ap1Timer = nh.createTimer(ros::Duration(1.0), ap1TimerCb);
  ap2Timer = nh.createTimer(ros::Duration(2.0), ap2TimerCb);

  ros::spin();
  return 0;
}

