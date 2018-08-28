#include "inverse_depth.h"


InverseDNode::InverseDNode():
	it_(nh_) {

	ROS_INFO("Inverse Depth Initialization");
	ros::NodeHandle params("~");
	front_image_ = it_.subscribe("/ardrone/image_raw", 1, &InverseDNode::FrontImage, this);
}

InverseDNode::~InverseDNode()
{

}

void InverseDNode::getPosition()
{	

	gls_request.request.model_name="quadrotor";
	gls_client = nh_.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	gls_client.call(gls_request);

	pose.position.x = gls_request.response.pose.position.x;
	pose.position.y = gls_request.response.pose.position.y;
	pose.position.z = gls_request.response.pose.position.z;

	//ROS_INFO("X: %f Y: %f Z: %f", pose.position.x, pose.position.y, pose.position.z);
}

void InverseDNode::getRotation()
{	
	pose.orientation.x = gls_request.response.pose.orientation.x;
	pose.orientation.y = gls_request.response.pose.orientation.y;
	pose.orientation.z = gls_request.response.pose.orientation.z;

	//ROS_INFO("rotX: %f rotY: %f rotZ: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void InverseDNode::cornerDetection()
{
	copy = front_image.clone();
	maxCorners = 3;
	RNG rng(12345);	

	cvtColor(front_image, image_gray, CV_BGR2GRAY);

	goodFeaturesToTrack( image_gray,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );

	for( int i = 0; i < corners.size(); i++ )
     	{ 
		circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 );
		//ROS_INFO("Px: %f Py: %f", corners[i].x, corners[i].y); 
	} 

}

void InverseDNode::calculateParameters(void)
{
	
}

void InverseDNode::FrontImage(const sensor_msgs::ImageConstPtr &msg)
{	
	try
	{
		cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	front_image = cv_ptr_1->image;	
  
	cornerDetection();
	
	namedWindow( "front_image", CV_WINDOW_AUTOSIZE );

	imshow("front_image", copy);

	waitKey(1);
		
}
