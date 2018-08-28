#include "inverse_depth.h"


InverseDNode::InverseDNode() {

	ros::NodeHandle params("~");
	gls_request.request.model_name="quadrotor";
	gls_client = nh_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	gls_client.call(gls_request);
}

InverseDNode::~InverseDNode()
{

}


void InverseDNode::getPosition()
{	
	pose.position.x = gls_request.response.pose.position.x;
	pose.position.y = gls_request.response.pose.position.y;
	pose.position.z = gls_request.response.pose.position.z;
	pose.orientation.x = gls_request.response.pose.orientation.x;
	pose.orientation.y = gls_request.response.pose.orientation.y;
	pose.orientation.z = gls_request.response.pose.orientation.z;
	ROS_INFO("X: %f Y: %f Z: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z);
}


void InverseDNode::getRotation()
{	

}
