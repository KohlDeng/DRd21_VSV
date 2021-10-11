#include <iostream>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "fsd_common_msgs/Map.h"

class VisualNode
{
public:
	VisualNode(ros::NodeHandle *nh);
	void mapCallback(const fsd_common_msgs::Map::ConstPtr &msg);
	void localmapCallback(const fsd_common_msgs::Map::ConstPtr &msg);
	void sendMsg();
private:
	ros::NodeHandle *nh_;
	ros::Publisher mapPub;
	ros::Publisher localmapPub;
	ros::Subscriber mapSub;
	ros::Subscriber localmapSub;

	visualization_msgs::MarkerArray visualMap;
	visualization_msgs::MarkerArray visualLocalmap;

};

VisualNode::VisualNode(ros::NodeHandle* nh)
{
	nh_=nh;
	mapPub = nh_->advertise<visualization_msgs::MarkerArray>(
		"visual/map/map",3);
	localmapPub = nh_->advertise<visualization_msgs::MarkerArray>(
		"visual/base/localmap",3);
	mapSub = nh_->subscribe("/map",3,&VisualNode::mapCallback,this);
	localmapSub = nh_->subscribe("/localmap",3,&VisualNode::localmapCallback,this);
}

void VisualNode::mapCallback(const fsd_common_msgs::Map::ConstPtr &msg)
{
	visualMap.markers.clear();
	visualization_msgs::Marker markerRed;
	visualization_msgs::Marker markerBlue;

   	markerRed.header.frame_id = "/map";
    markerRed.header.stamp = ros::Time::now();
    markerRed.ns = "red";
    markerRed.action = visualization_msgs::Marker::ADD;
    markerRed.type = visualization_msgs::Marker::SPHERE;
    markerRed.scale.x = 0.5;
    markerRed.scale.y = 0.5;
    markerRed.scale.z = 0.5;
    markerRed.color.a = 0.5;
    markerRed.color.r = 1.0;
    markerRed.color.g = 0;
    markerRed.color.b = 0;
    markerRed.color.a = 1.0;
    
    int i=0;
    markerRed.points.clear();
    for(const auto &iter: msg->cone_red) {
        markerRed.pose.position.x = iter.position.x;
        markerRed.pose.position.y = iter.position.y;
        markerRed.id = i;
        i++;
        visualMap.markers.push_back(markerRed);
        if(i>1000) break;
    }

   	markerBlue.header.frame_id = "/map";
    markerBlue.header.stamp = ros::Time::now();
    markerBlue.ns = "blue";
    markerBlue.action = visualization_msgs::Marker::ADD;
    markerBlue.type = visualization_msgs::Marker::SPHERE;
    markerBlue.scale.x = 0.5;
    markerBlue.scale.y = 0.5;
    markerBlue.scale.z = 0.5;
    markerBlue.color.a = 0.5;
    markerBlue.color.r = 0;
    markerBlue.color.g = 0;
    markerBlue.color.b = 1.0;
    markerBlue.color.a = 1.0;
    
    i=0;
    markerBlue.points.clear();
    for(const auto &iter: msg->cone_blue) {
        markerBlue.pose.position.x = iter.position.x;
        markerBlue.pose.position.y = iter.position.y;
        markerBlue.id = i;
        i++;
        visualMap.markers.push_back(markerBlue);
        if(i>1000) break;
    }
    ROS_INFO("visualMap Successful");
}

void VisualNode::localmapCallback(const fsd_common_msgs::Map::ConstPtr &msg)
{
	visualLocalmap.markers.clear();
	visualization_msgs::Marker markerRed;
	visualization_msgs::Marker markerBlue;

   	markerRed.header.frame_id = "/base_link";
    markerRed.header.stamp = ros::Time::now();
    markerRed.ns = "red";
    markerRed.action = visualization_msgs::Marker::ADD;
    markerRed.type = visualization_msgs::Marker::SPHERE;
    markerRed.scale.x = 0.5;
    markerRed.scale.y = 0.5;
    markerRed.scale.z = 0.5;
    markerRed.color.a = 0.5;
    markerRed.color.r = 1.0;
    markerRed.color.g = 0;
    markerRed.color.b = 0;
    markerRed.color.a = 1.0;
    
    int i=0;
    markerRed.points.clear();
    for(const auto &iter: msg->cone_red) {
        markerRed.pose.position.x = iter.position.x;
        markerRed.pose.position.y = iter.position.y;
        markerRed.id = i;
        i++;
        visualLocalmap.markers.push_back(markerRed);
        if(i>1000) break;
    }

   	markerBlue.header.frame_id = "/base_link";
    markerBlue.header.stamp = ros::Time::now();
    markerBlue.ns = "blue";
    markerBlue.action = visualization_msgs::Marker::ADD;
    markerBlue.type = visualization_msgs::Marker::SPHERE;
    markerBlue.scale.x = 0.5;
    markerBlue.scale.y = 0.5;
    markerBlue.scale.z = 0.5;
    markerBlue.color.a = 0.5;
    markerBlue.color.r = 0;
    markerBlue.color.g = 0;
    markerBlue.color.b = 1.0;
    markerBlue.color.a = 1.0;
    
    i=0;
    markerBlue.points.clear();
    for(const auto &iter: msg->cone_blue) {
        markerBlue.pose.position.x = iter.position.x;
        markerBlue.pose.position.y = iter.position.y;
        markerBlue.id = i;
        i++;
        visualLocalmap.markers.push_back(markerBlue);
        if(i>1000) break;
    }
    ROS_INFO("visual Localmap Successful");
}


void VisualNode::sendMsg()
{
	mapPub.publish(visualMap);
	localmapPub.publish(visualLocalmap);
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"visualization");
	ros::NodeHandle nh;
	VisualNode visual_node(&nh);
	ros::Rate timer_hz(1);
	while(ros::ok())
	{
		visual_node.sendMsg();
		timer_hz.sleep();
		ros::spinOnce();
	}
	return 0;

}