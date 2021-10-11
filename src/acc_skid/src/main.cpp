#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"acc_skid_pub");
	ros::NodeHandle nh;

	ros::Publisher acc_pub = nh.advertise<geometry_msgs::Point>("/planning/end_point",5);
	ros::Publisher skip_pub = nh.advertise<std_msgs::Float64MultiArray>("/transform_matrix",5);
	
	geometry_msgs::Point end_point;
	end_point.x = 75.0;
	end_point.y = 0.0;
	end_point.z = 0.0;
	std_msgs::Float64MultiArray transform_matrix;
	for(int i=0;i<16;i++)
	{
		transform_matrix.data.push_back(0);
	}
	transform_matrix.data[0]=1;
	transform_matrix.data[5]=1;
	transform_matrix.data[10]=1;
	transform_matrix.data[15]=1;
	ROS_INFO("work starting...");
	ros::Rate timer_10hz(10);	
	while(ros::ok())
	{
		acc_pub.publish(end_point);
		skip_pub.publish(transform_matrix);
		timer_10hz.sleep();
		ros::spinOnce();

	}
	return 0;
}