#include <ros/ros.h>
#include <control.hpp>
#include <chrono>

typedef ns_control::Control Control;

int main(int argc, char **argv) 
{
	ros::init(argc,argv,"track_control_node");
	ros::NodeHandle nh;
	ros::Rate timer_100hz(100);

	Control control(nh);

	while(ros::ok())
	{
		control.runAlgorithm();
		control.sendMsg();
		ros::spinOnce();
		timer_100hz.sleep();
	}

	ros::AsyncSpinner spinner(6);
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
