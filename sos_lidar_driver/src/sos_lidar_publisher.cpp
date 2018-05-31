#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <sstream>
#include <iostream>
#include "sos_lidar.h"

#include <libv4l2.h>  // -lv4l2
#include <linux/videodev2.h>
#include <fcntl.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "sos_lidar_publisher");

  SOSlaser lidar;

  while (ros::ok())
  {
    lidar.poll();
  }

  return 0;
}
