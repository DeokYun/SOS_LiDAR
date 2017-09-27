#ifndef SOS_LIDAR_H_
#define SOS_LIDAR_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <iostream>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <libv4l2.h>  // -lv4l2
#include <linux/videodev2.h>
#include <fcntl.h>

//class LFCDLaser
class SOSlaser
 {
 public:

	SOSlaser();

  ~SOSlaser();   // 여기에 fd 닫는거 추가해야할 듯

  void poll();    // 거리값 data 받아오기

 private:
  //--------------------------- lidar ------------------------------//
  ros::NodeHandle nh_;   // ROS NodeHandle
  ros::Publisher laser_pub_;  // ROS Topic Publishers
  std::string frame_id_;
  sensor_msgs::LaserScan scan_;     // lidar 거리값처럼 계산해서 plot, slam
  int port_;
  //----------------------------------------------------------------//


  //------------------------ Video ---------------------------------//
  cv::VideoCapture Camera;
/*
  ros::NodeHandle nh;
  sensor_msgs::ImagePtr msg;
  image_transport::Publisher pub;
  //------------------------ Video ---------------------------------//

  //------------------------ Video bin_img ---------------------------------//
  ros::NodeHandle nh2;
  sensor_msgs::ImagePtr msg2;
  image_transport::Publisher pub2;
  //------------------------ Video ---------------------------------//
*/
  int fd;     // 카메라 설정
  };


#endif
