#include "dataset_ros/err_check_nodelet.h"


namespace dataset_ros {

  void ErrCheckNodelet::onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    depthSub = nh.subscribe("/camera/depth_registered/image_raw", 10,
        &ErrCheckNodelet::depthCb, this);
    velodyneSub = nh.subscribe("/velodyne_points", 10,
        &ErrCheckNodelet::velodyneCb, this);
  }
  
  void ErrCheckNodelet::depthCb (const sensor_msgs::ImageConstPtr &depth) {
    int zero = 0;
    for (auto it = depth->data.begin(); it != depth->data.end(); it++) {
      if (*it == 0)
        zero++;
    }
    float empty_percent = float(zero)/float(depth->data.size()); 
    if (empty_percent > 0.80) {
      NODELET_ERROR("Most of the Depth Image is Empty!");
      ros::shutdown();
    }
  }

  void ErrCheckNodelet::velodyneCb (const sensor_msgs::PointCloud2ConstPtr &cloud) {
    int zero = 0;
    for (auto it = cloud->data.begin(); it != cloud->data.end(); it++) {
      if (*it == 0)
        zero++;
    }
    float empty_percent = float(zero)/float(cloud->data.size()); 
    if (empty_percent > 0.80) {
      NODELET_ERROR("Most of the Depth Image is Empty!");
      ros::shutdown();
    }

  }


} // namespace dataset_ros


