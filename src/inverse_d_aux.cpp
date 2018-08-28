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

void InverseDNode::cornerDetection()
{
	copy = front_image.clone();
	maxCorners = 3;
	RNG rng(12345);	
	qualityLevel = 0.01;
	minDistance = 10;
	blockSize = 3;
	useHarrisDetector = false;
	k = 0.04;
	r = 4;

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
	
	getPose();
		
	calculateDistortionParameters(corners);

	vectorDirection(pose, distortion);

	//Initialization
	generateRay();

}

void InverseDNode::calculateDistortionParameters(vector<Point2f> corners)
{
	Cx = 319.595296;
	Cy = 179.600265;
	fx = 374.784581;
	fy = 374.801490;
	
	Hx = (corners[0].x - Cx)/fx;
	Hy = (corners[0].y - Cy)/fy;
	//ROS_INFO("Hx: %f Hy: %f", Hx, Hy); 

	u = Cx - (fx*Hx); //Is it in pixels or mm
	v = Cy - (fy*Hy);
	distortion = tf2::Vector3(u, v, 1);
}


void InverseDNode::generateRay()
{
	if(init_depth){
		initial_pose = pose;
		init_depth = false;
	}

	
	
		
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
