#include "inverse_depth.h"


int main(int argc, char *argv[]) {

	ros::init(argc, argv, "inverse_depth");

	InverseDNode Node;

	ros::spin();
	
	return 0;

}

