#include <ros/ros.h>
#include "detector.h"

typedef ns_detector::BoundaryDetector BoundaryDetector;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"cone_detector_node");
	ros::NodeHandle nh;
	BoundaryDetector boundary_detector(nh);
	ros::Rate timer_hz(50);
	while(ros::ok())
	{
		boundary_detector.runAlgorithm();
		boundary_detector.sendMsg();
		timer_hz.sleep();
		ros::spinOnce();
	}
	return 0;

}