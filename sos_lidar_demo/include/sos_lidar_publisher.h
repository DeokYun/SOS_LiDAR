#ifndef SOS_LIDAR_PUBLISHER_H_
#define SOS_LIDAR_PUBLISHER_H_

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


class SOSlidar
 {
 public:

	SOSlidar();

  ~SOSlidar();   // 여기에 fd 닫는거 추가해야할 듯

  void camera_publisher();    // 거리값 data 받아오기

 private:

  //------------------------ Video ---------------------------------//
  cv::VideoCapture cap;

  ros::NodeHandle nh;
  sensor_msgs::ImagePtr msg;  // sensor_msgs(package 이름)::ImagePtr 방식으로 메세지를 선언
  image_transport::Publisher pub;
  
  //------------------------ Video ---------------------------------//

  int fd;     // 카메라 설정
  };


#endif
