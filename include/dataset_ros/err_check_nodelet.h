#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#ifndef __DATASET_ROS_ERR_CHECK_NODELET_H__
#define __DATASET_ROS_ERR_CHECK_NODELET_H__

namespace dataset_ros {
  class ErrCheckNodelet : public nodelet::Nodelet {
    public:
      ErrCheckNodelet() {};

    private:
      virtual void onInit();
      void depthCb    (const sensor_msgs::ImageConstPtr &depth);
      void velodyneCb (const sensor_msgs::PointCloud2ConstPtr &cloud);

      ros::NodeHandle nh, private_nh;
      ros::Subscriber depthSub;
      ros::Subscriber velodyneSub;
  };
}

#endif //__DATASET_ROS_ERR_CHECK_NODELET_H__

