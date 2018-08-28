#include "inverse_depth.h"

void InverseDNode::getPose()
{	

	gls_request.request.model_name="quadrotor";
	gls_client = nh_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	gls_client.call(gls_request);

	pose.position.x = gls_request.response.pose.position.x;
	pose.position.y = gls_request.response.pose.position.y;
	pose.position.z = gls_request.response.pose.position.z;

	pose.orientation.x = gls_request.response.pose.orientation.x;
	pose.orientation.y = gls_request.response.pose.orientation.y;
	pose.orientation.z = gls_request.response.pose.orientation.z;
	pose.orientation.w = gls_request.response.pose.orientation.w;

	//ROS_INFO("X: %f Y: %f Z: %f", pose.position.x, pose.position.y, pose.position.z);
	//ROS_INFO("rotX: %f rotY: %f rotZ: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z);

}


void InverseDNode::vectorDirection(const geometry_msgs::Pose pose, const tf2::Vector3 distortion)
{
	q = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	rotation = tf2::Matrix3x3(q);
	hworld = rotation * distortion;
	theta = tf2Atan2(hworld[0],hworld[2]);
	phi = tf2Atan2(-hworld[1],tf2Sqrt((tf2Pow(hworld[0],2))+(tf2Pow(hworld[2],2))));
	ROS_INFO("T: %f", theta);
	ROS_INFO("P: %f", phi);

}
