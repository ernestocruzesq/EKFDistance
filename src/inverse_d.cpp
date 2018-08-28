#include "inverse_depth.h"


int main(int argc, char *argv[]) {

	ros::init(argc, argv, "inverse_depth");

	InverseDNode Node;

	Node.getPosition();
	
	Node.getRotation();

	ros::spin();
	
	return 0;

}

