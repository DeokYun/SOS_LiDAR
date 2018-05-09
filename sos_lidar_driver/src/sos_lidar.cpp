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

const float PI = 3.14159265;

SOSlaser::SOSlaser()
{
    //initialization
    ros::NodeHandle nh_("~");
    nh_.param("video_port", port_, 1);
    nh_.param("frame_id", frame_id_, std::string("base_scan"));

    //------------------------------------------------------//
    scan_.header.frame_id = frame_id_;
    scan_.angle_min = -50.0*PI/180;          // 40 deg to 140 deg fov
    scan_.angle_max = 50.0*PI/180;         // 40 deg to 140 deg fov
    scan_.angle_increment = 0.1*(PI/180);    // angular resolution 0.1 deg
    scan_.range_min = 0.15;
    scan_.range_max = 3.0;
    scan_.ranges.resize(1001);
    //------------------------------------------------------//

    laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

    //-------------------------------------------------//
    std::stringstream video_setting_num;
    video_setting_num << "/dev/video" << port_;
    std::string video_setting = video_setting_num.str();
    //-------------------------------------------------//


    //------------------ camera open ----------------------------//
    if(!Camera.open(port_))
    {
      throw std::runtime_error("Couldn't open " + video_setting);
      ROS_ERROR("Error to initialize camera");
      exit(0);
    }
    else
    {
      cout << "Opened " << video_setting << "!!!" << endl;
      ROS_INFO_STREAM("A camera is Opend!!");
    }
    //------------------ camera open ----------------------------//


    //-------------------Camera resoultion setup-----------------------//
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    //-----------------------------------------------------------------//

    //-------------------Camera pixel format setup---------------------//
    Camera.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
    // Camera.set(CAP_PROP_FPS, 30.0);
    //-----------------------------------------------------------------//

    //------------------------------v4l2 setup----------------------//
    // fd = open("/dev/video0", O_RDWR);
    fd = open(video_setting.c_str(), O_RDWR);

    v4l2_control c;

    c.id = V4L2_CID_EXPOSURE_AUTO;
    c.value = 0;
    v4l2_ioctl(fd, VIDIOC_ENUM_FMT, &c);

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

    c.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    c.value = 2800;
    v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);

    c.id = V4L2_CID_GAMMA;
    c.value = 72;
    v4l2_ioctl(fd, VIDIOC_S_CTRL, &c);
    //------------------------------v4l2 setup----------------------//
}


SOSlaser::~SOSlaser()
{
  close(fd);
}



void SOSlaser::poll()
{
  //--------------------------set the default variables-----------------------------//
  //--------------------------------------------------------------------------------//

  //------------------------------------ ROI ---------------------------------------//
  int c_start =195;
  int r_start = 0;
  int c_length = 1685;
  int r_length = 658;
  Rect ROI(c_start, r_start, c_length, r_length);

  //-------------------------number of datas for each frame----------------------------//
  int c_size=1001;

  //--------------------------laser beam threshold value----------------------------//
  int threshold_value = 170;

  //-----------------------------define the variables------------------------------//
  Mat tempFrame1;
  Mat sub_img;
  Mat gray_img;
  Mat resized_img;
  Mat bin_img;
  Mat distorted_pts = cv::Mat(1, c_length, CV_32FC2);
  Mat undistorted_pts = cv::Mat(1, c_length, CV_32FC2);

  float radius;
  float theta;
  //--------------------------------------------------------------------------------//

  //---------------------------calbration coefficient-------------------------------//
  Mat_<float> camMat(3,3);
  Mat_<float> distCoeffs(1,4);
  double fx = 1036.2; double fy = 1039.5; double cx = 966.6125; double cy = 542.7185;
  float laser_angle = -0.55*(PI/180);
  float dist_calib_a = -49200; float dist_calib_b = -527.1; float dist_calib_c = 17.3;
  camMat << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  distCoeffs << -0.3112, 0.071369, 0.0005339, 0.00136279;

  //--------------------------------------------------------------------------------//
  //---------------------------------------------------------------------------------//

  Camera >> tempFrame1 ;
  // cout << tempFrame1.size().width << endl;
  // cout << tempFrame1.size().height << endl;


    if(!tempFrame1.empty())
    {
      //------------------------------------- laser beam extraction -------------------------------------------//
      sub_img = tempFrame1(ROI);

      cvtColor(sub_img, gray_img, CV_RGB2GRAY);
      resize(gray_img, resized_img, Size(c_size, r_length));
      threshold(resized_img, bin_img, threshold_value, 255, THRESH_TOZERO);

      for (int c = 0; c < c_size; c++)
      {
        float num = 0;
        float den = 0;

        for (int  r = 0; r < r_length; r++)
        {
          num = num + (r+1)*bin_img.at<uchar>(r, c);
          den = den + bin_img.at<uchar>(r, c);
        }

        distorted_pts.at<Vec2f>(0, c)[0] = c*(c_length-1)/(c_size-1)+c_start;
        distorted_pts.at<Vec2f>(0, c)[1] = r_start+num/den-1;
      }
      //--------------------------------------------------------------------------------------------------------//

      undistortPoints(distorted_pts, undistorted_pts, camMat, distCoeffs);	// camera calibration

      //---------------------------------------- Radius calculation --------------------------------------------//
      for (int c = 0; c < c_size; c++)
      {
        theta = (-50 + c/10)*PI/180; // angular position //
        if (distorted_pts.at<Vec2f>(0, c)[1] == r_start-1)
          radius = 10000; // maximum distance //
        else
        {
          float x_undist = fx*undistorted_pts.at<Vec2f>(0, c_size-1-c)[0]+cx;
          float y_undist = fy*undistorted_pts.at<Vec2f>(0, c_size-1-c)[1]+cy;
          float y_undist2 = -(x_undist-cx)*sin(laser_angle)+(y_undist-cy)*cos(laser_angle)+cy;
          radius = ((dist_calib_a/(y_undist2+dist_calib_b)+dist_calib_c)/cos(theta));
        }
        scan_.ranges[c_size-1-c] = radius/1000.0;
      }
      //--------------------------------------------------------------------------------------------------------//

      scan_.time_increment = (1/10)/1000;
      scan_.scan_time = (1/10);
      scan_.header.stamp = ros::Time::now();

      //------------------------------------------------------------//

      laser_pub_.publish(scan_);
    }
}
