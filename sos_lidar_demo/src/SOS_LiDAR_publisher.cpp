#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>

//------------------------lv4l2-----------------------//
#include <libv4l2.h>  // -lv4l2
#include <linux/videodev2.h>
#include <fcntl.h>
//----------------------------------------------------//

#include "sos_lidar_publisher.h"

using namespace cv;
using namespace std;

SOSlidar::SOSlidar()
{
  //생성자, initialization
  while (1)
  {
    cap.open(1);    // defalut : 0
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);   //설정하는거 while 안에 넣으면 딜레이생김
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    // cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);   //설정하는거 while 안에 넣으면 딜레이생김
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    if(!cap.isOpened())  // check if we succeeded
    {
      //cout << "error to initialize camera" << endl;
      ROS_ERROR("error to initialize camera");
      continue;
    }
    else
    {
      ROS_INFO_STREAM("A camera is opend!!");
      break;
    }
  }


  //------------------------ Video ---------------------------------//
  image_transport::ImageTransport it(nh);
  pub = it.advertise("camera/image", 100);

  //------------------------ Video ---------------------------------//


  //------------------------------v4l2 setup----------------------//
  //int fd;

  fd = open("/dev/video1", O_RDWR);     // 1번 카메라

  v4l2_control c;

  c.id = V4L2_CID_EXPOSURE_AUTO;
  c.value = V4L2_EXPOSURE_MANUAL;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
  c.value = 0;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  c.value = 150;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_AUTO_WHITE_BALANCE;
  c.value = 0;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  c.id = V4L2_CID_GAMMA;
  c.value = 72;
  v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

  //------------------------------v4l2 setup----------------------//
}

SOSlidar::~SOSlidar()
{
  close(fd);
}


void SOSlidar::camera_publisher()
{
  Mat frame;
  cap >> frame;

  //-----------------debug------------------//
  //노트북 캠이 1920 1080을 지원 안하는듯
  //cv::Size s = frame.size();
  //ROS_INFO_STREAM("height : " << s.height);
  //ROS_INFO_STREAM("width : " << s.width);
  //----------------------------------------//

  if(!frame.empty())
  {
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);

  }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "SOS_LiDAR_publisher");

  SOSlidar Camera;

  ros::Rate loop_rate(5); // loop 주기 설정


  while (ros::ok())
  {
    Camera.camera_publisher();
    //ros::spinOnce();  //?
    //loop_rate.sleep();    // 5Hz가 될때까지 기다림

  }
  return 0;
}
