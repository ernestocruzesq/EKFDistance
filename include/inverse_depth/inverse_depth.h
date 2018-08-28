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

using namespace std;

class InverseDNode {
 public:
	int position;
  	InverseDNode();
	~InverseDNode();
	void positionCallback(const geometry_msgs::Point msg);
	void printPosition(void);

 private:

  	ros::NodeHandle nh_;
  	
	ros::Subscriber gazebo_position;

	ros::ServiceClient gls_client;

	gazebo_msgs::GetModelState gls_request;

};

#endif
