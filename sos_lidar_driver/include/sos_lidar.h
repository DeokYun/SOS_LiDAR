#ifndef SOS_LIDAR_H_
#define SOS_LIDAR_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>

#include <libv4l2.h>  // -lv4l2
#include <linux/videodev2.h>
#include <fcntl.h>

class SOSlaser
 {
 public:

	SOSlaser();

  ~SOSlaser();

  void poll();

 private:
  //--------------------------- lidar ------------------------------//
  ros::NodeHandle nh;   // ROS NodeHandle
  ros::Publisher laser_pub_;  // ROS Topic Publishers
  std::string frame_id_;      // LiDAR farme
  sensor_msgs::LaserScan scan_;
  int port_;
  //----------------------------------------------------------------//


  //------------------------ Video ---------------------------------//
  cv::VideoCapture Camera;
  int fd;     // v4l2 camera setting
  //----------------------------------------------------------------//
  };


#endif
