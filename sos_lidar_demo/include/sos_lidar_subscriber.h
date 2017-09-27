#ifndef SOS_LIDAR_SUBSCRIBER_H_
#define SOS_LIDAR_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <libv4l2.h>  // -lv4l2
#include <linux/videodev2.h>
#include <fcntl.h>


class SOSlidar_sub
 {
 public:

	SOSlidar_sub();

  ~SOSlidar_sub() {};

 void imageCallback(const sensor_msgs::ImageConstPtr& msg);

 private:

  //------------------------ Video subscriber ---------------------------------//
  ros::NodeHandle nh;
  image_transport::Subscriber sub;
  //--------------------------------------------------------------------------//

  //--------------------------- lidar ------------------------------//
    ros::NodeHandle nh_;   // ROS NodeHandle
    ros::Publisher laser_pub_;  // ROS Topic Publishers
    std::string frame_id_;
    sensor_msgs::LaserScan scan_;     // lidar 거리값처럼 계산해서 plot, slam

 //----------------------------------------------------------------//

  };


#endif
