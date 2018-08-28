#include "inverse_depth.h"


InverseDNode::InverseDNode() {

	ros::NodeHandle params("~");
	//gazebo_position = nh_.subscribe("gazebo/LinkStates", 3, &InverseDNode::positionCallback, this);
	gls_client = nh_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
}

InverseDNode::~InverseDNode()
{

}


/*void InverseDNode::positionCallback(const geometry_msgs::Point msg)
{

  position = msg.x;
  
}


void InverseDNode::printPosition()
{
  ROS_INFO("Reconfigure Request: %f", position);
}
*/


