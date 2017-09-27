#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include "opencv2/opencv.hpp"
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
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
  //생성자, initialization
  nh_.param("video_port", port_, 0);
  nh_.param("frame_id", frame_id_, std::string("laser"));

  //--------------------1920*1080-----------------------//
  scan_.header.frame_id = frame_id_;
  scan_.angle_min = 40.0*PI/180;          // 40도에서 140도 fov
  scan_.angle_max = 140.0*PI/180;         // 라디안 단위
  scan_.angle_increment = (PI/180);    // 라디안 분해능  1도
  scan_.range_min = 0.12;
  scan_.range_max = 10.0;
  scan_.ranges.resize(101);
  //------------------------------------------------------//

  // //----------------------640*480---------------------//
  // scan_.header.frame_id = frame_id_;
  // scan_.angle_min = 40.0*PI/180;          // 40도에서 140도 fov
  // scan_.angle_max = 118.0*PI/180;         // 라디안 단위
  // scan_.angle_increment = (PI/180);    // 라디안 분해능  1도
  // scan_.range_min = 0.12;
  // scan_.range_max = 3.0;
  // scan_.ranges.resize(79);
  //   //----------------------640*480---------------------//
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 100);

  Camera.open(port_);
  //--------------------1920*1080-----------------------//
  Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
  //------------------------------------------------------//

  //----------------------640*480---------------------//
  // Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  // Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  //----------------------640*480---------------------//


  //------------------------------v4l2 setup----------------------//
  fd = open("/dev/video0", O_RDWR);

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


SOSlaser::~SOSlaser()
{
  close(fd);
}



void SOSlaser::poll()
{
   //------ 1920 * 1080 ----------//
    int c_start =237;
    int r_start = 0;
    int c_length = 1649;
    int r_length = 700;


    float theta[1649] = {0};
    float radius[1649] = {0};
    float value_out[1649] = {1080};

    int c_size=101;
    //------ 1920 * 1080 ----------//


    // // ------------  640 * 480 --------------//
    // int c_start = 118;
    // int r_start = 0;
    // int c_length = 522;
    // int r_length = 350;
    //
    // float theta[1649] = {0};
    // float radius[1649] = {0};
    // float value_out[1649] = {480};
    //
    // int c_size=79;
    // // ------------  640 * 480 --------------//

    Mat tempFrame1;
    Mat frame1;
    Mat sub_img;
    Mat gray_img;
    Mat resized_img;
    Mat bin_img;
    Mat distorted_pts = cv::Mat(1, c_length, CV_32FC2);
    Mat undistorted_pts = cv::Mat(1, c_length, CV_32FC2);

    Mat_<float> camMat(3,3);
    Mat_<float> distCoeffs(1,4);

    //calbration coefficient
    Rect ROI(c_start, r_start, c_length, r_length);   // 관심영역

     //------ 1920 * 1080 ----------//
    camMat << 1079.167, 0, 964.587, 0, 1091.033, 638.202, 0, 0, 1;
    distCoeffs << -0.307362, 0.06706, 0.000422, 0.005664;
    double fx = 1079.167; double fy = 1091.033; double cx = 964.587; double cy = 638.202;
     //------ 1920 * 1080 ----------//

    // // ------------  640 * 480 --------------//
    // camMat << 1079.167, 0, 964.587, 0, 1091.033, 668.202, 0, 0, 1;
    // distCoeffs << -0.257362, 0.06706, 0.000422, 0.005664;
    // double fx = 1079.167; double fy = 1091.033; double cx = 964.587; double cy = 668.202;
    // // ------------  640 * 480 --------------//
  //------------------------------------------------------------------------------------------------//

  Camera >> tempFrame1 ;
  imshow("LIDAR", tempFrame1);
  waitKey(10);

  //ROS_INFO_STREAM("size : " << tempFrame1.size());

      if(!tempFrame1.empty())
       {

        sub_img = tempFrame1(ROI);

        cvtColor(sub_img, gray_img, CV_RGB2GRAY);
        resize(gray_img, resized_img, Size(c_size, r_length));
        threshold(resized_img, bin_img, 150, 255, THRESH_TOZERO);            // 단순히 threshold로 라인레이져 잡은거 --> 다른 필터로 바꿀 수 있을듯
                                                                             // HSV로...???
        for (int c = 0; c < c_size; c++)
        {
            float num = 0;
            float den = 0;
            int count = 0;

            for (int  r = 0; r < r_length; r++) {
              int value = bin_img.at<uchar>(r, c);
              if (value != 0) {
                count = count+1;
                num = num + (r+1)*value;
                den = den + value;
              }
            }
            if (count == 0) {
              value_out[c] = 1;
            }
            else {
              value_out[c] = num/den;
            }
             //------ 1920 * 1080 ----------//
            distorted_pts.at<Vec2f>(0, c)[0] = c*(c_length-1)/(c_size-1)+c_start;
            distorted_pts.at<Vec2f>(0, c)[1] = r_start+value_out[c]-1;
             //------ 1920 * 1080 ----------//

            // ------------  640 * 480 --------------//
            // distorted_pts.at<Vec2f>(0, c)[0] = 2*(c*(c_length-1)/(c_size-1)+c_start);
            // distorted_pts.at<Vec2f>(0, c)[1] = 2*(r_start+value_out[c]-1);
            // ------------  640 * 480 --------------//
          }     // 레이져 부분만 찾아서 캘리브레이션

        undistortPoints(distorted_pts, undistorted_pts, camMat, distCoeffs);			// calibration

        //-------------------------------------- 거리값 계산하는 부분 --------------------------------------------//
        for (int c = 0; c < c_size; c++)
        {
          //theta[c] = (-50 + c*100/(c_size-1))*PI/180;
          theta[c] = (-50 + c)*PI/180;
          if (value_out[c_size-1-c] == 1)
            radius[c] = 60000;
          else
           {
           //------ 1920 * 1080 ----------//
            radius[c] = ((-31440/((fy*undistorted_pts.at<Vec2f>(0, c_size-1-c)[1]+cy)/1.5-432.1)+56.36)/cos(theta[c])); //for undistorted points
           //------ 1920 * 1080 ----------//

	          // ------------  640 * 480 --------------//
            // radius[c] = ((-31440/((2*fy*undistorted_pts.at<Vec2f>(0, c_size-1-c)[1]+cy)/1.5-432.1)+56.36)/cos(theta[c]));
	           // ------------  640 * 480 --------------//
	         }

          if (radius[c] > 3000)
              radius[c] = 0;		// 3m 넘으면 그냥 0으로

          scan_.ranges[c] = radius[c]/1000.0;      // 왼쪽에서붙터라 100-c 로 바꿔줘야할듯

        }
        //----------------------------------------------------------------------------------------------------//

        scan_.time_increment = (1/10)/100;   // 1sec / 28scan/ 100beam
        scan_.scan_time = (1/10);        // 28HZ scan time
        scan_.header.stamp = ros::Time::now();


        //--------------------------debug-----------------------------//
        // ROS_INFO_STREAM("0 : " << scan_.ranges[0]);
        // ROS_INFO_STREAM("5 : " << scan_.ranges[5]);
        // ROS_INFO_STREAM("50 : " << scan_.ranges[50]);
        // ROS_INFO_STREAM("55 : " << scan_.ranges[55]);
        // ROS_INFO_STREAM("95 : " << scan_.ranges[95]);
        // ROS_INFO_STREAM("100 : " << scan_.ranges[100]);
        //------------------------------------------------------------//


        laser_pub_.publish(scan_);

        // ros::spinOnce();

      }

}
