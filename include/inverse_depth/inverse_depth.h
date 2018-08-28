#ifndef INVERSE_DEPTH_H 
#define INVERSE_DEPTH_H 

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/Marker.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Empty.h"

#include "std_msgs/String.h"
#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <gazebo_msgs/GetModelState.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Scalar.h>


using namespace cv;
using namespace std;

class InverseDNode {
 public:

  	InverseDNode();
	~InverseDNode();

	//***Functions***

	//Drone position functions
	void getPose(void);
	void vectorDirection(const geometry_msgs::Pose pose, const tf2::Vector3 distortion);

	//Callbacks
	void FrontImage(const sensor_msgs::ImageConstPtr &msg);

	//Corner Tracking (Shi-Tomasi)
	void cornerDetection(void);

	//Calculations
	void calculateDistortionParameters(vector<Point2f> corners);

	//Ray creation
	void generateRay(void);
	

	//***Variable Init***

	//ROS
	ros::NodeHandle nh_;

	//Image capture
	Mat front_image;
	cv_bridge::CvImagePtr cv_ptr_1;
	image_transport::ImageTransport it_;
  	image_transport::Subscriber front_image_;

	//Position
	ros::ServiceClient gls_client;
	geometry_msgs::Pose pose;
	gazebo_msgs::GetModelState gls_request;

	//Angles Azimuth and elevation
	tf2::Quaternion q;
	tf2::Matrix3x3 rotation;
	tf2::Vector3 distortion;
	tf2::Vector3 hworld;
	float u, v;
	


	//Corner Tracking (Shi-Tomasi)
	int maxCorners;
	int maxTrackbar;
	Mat image_gray, copy;
	vector<Point2f> corners;
	double qualityLevel;
  	double minDistance;
  	int blockSize;
  	bool useHarrisDetector;
  	double k;
	int r;

	//Parameter tracking
	float Cx;
	float Cy;
	float fx;
	float fy;
	float Hx, Hy, Hz;

	//Ray creation
	bool init_depth = true;
	float inverse_d;
	float theta;
	float phi;
	vector<Point3f> m;
	geometry_msgs::Pose initial_pose;	

};

#endif
