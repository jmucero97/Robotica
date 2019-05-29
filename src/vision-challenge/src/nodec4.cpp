


#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include "student.h"
#include <sstream>
#include <string.h>
    using namespace std;
string salida = "";
  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    cv::Mat rgb_frame; //Input image in matrix form
    cv::Mat out_frame; //Output image
    cv::Mat circle_frame; //Output image

    cv_bridge::CvImagePtr cv_ptr_rgb; //cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actually use it
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //make a copy of the ROS message data. //bgra8: BGR color image with an alpha channel
                                                         //Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    rgb_frame = cv_ptr_rgb->image; //Here we have the current frame in OpenCV Mat format

    processImageChallenge(rgb_frame,out_frame);
  rgb_frame.copyTo(circle_frame);
    processImageCircle_c4(out_frame,circle_frame);
   
cv::imshow("input_image", rgb_frame);    
cv::imshow("output_image", out_frame);
cv::imshow("circle_show", circle_frame);

cv::Moments moment = cv::moments(out_frame);
	double x = moment.m10/moment.m00;
	double y = moment.m01/moment.m00;

std::string xd;
std::stringstream xs;
xs << x;
xd = xs.str();

std::string yd;
std::stringstream ys;
ys << y;
yd = ys.str();

float sec=msg->header.stamp.sec;
float nsec=msg->header.stamp.nsec;
std::string sd;
std::stringstream ss;
ss << sec;
sd = ss.str();

std::string nd;
std::stringstream ns;
ns << nsec;
nd = ns.str();

/*
	std::string xd = to_string(x);
	std::string yd = to_string(y);
	std::string sec = to_string(msg->header.stamp.sec);
	std::string  nsec= to_string(msg->header.stamp.nsec);
*/
if(!isnan(x)){
	salida.append(sd);
	salida.append("\t");
	salida.append(nd);
	salida.append("\t");
	salida.append(xd);
	salida.append("\t");
	salida.append(yd);
	salida.append("\n");
}
    cv::waitKey(3); //Wait for 3 milliseconds
  }
    
 

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "edp2_c4"); 

  ros::NodeHandle nh_;
  
  image_transport::ImageTransport it_(nh_);

  image_transport::Subscriber image_sub_;

  image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb); //subscribes to the Kinect video frames

  ros::spin();


//IMPRIMIMO
	
	ofstream myfile("groundtruth.txt");
	if(myfile.is_open())
	{
	myfile << salida;
	 
	}else{
	cout <<"no esta";
	}
	myfile.close();
  return 0;
}
