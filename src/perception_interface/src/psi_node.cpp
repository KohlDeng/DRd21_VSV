#include "perception_slam_interface/psi.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "psi_node");

	ros::NodeHandle nh("");
	PSI PSI(&nh);

	ros::spin();
}
