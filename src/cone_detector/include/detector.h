#pragma once

#include "ros/ros.h"
#include <iostream>
#include <map>
#include <cmath>

#include "fsd_common_msgs/Cone.h"
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include <type.hpp>
#include <visual_path.hpp>

namespace ns_detector
{

using ConePos = FSD::ConePos;
using PathPoint = FSD::PathPoint;
using SearchTree = FSD::SearchTree;
using Cost_index = FSD::Cost_index;
using Cost_n = FSD::Cost_n;
using Cost_p = FSD::Cost_p;

class BoundaryDetector
{
public:
	BoundaryDetector(ros::NodeHandle &nh);
	void runAlgorithm();
	void sendMsg();


private:
	ros::NodeHandle nh_;
	fsd_common_msgs::Map map_current;
  	fsd_common_msgs::Map map;
  	visualization_msgs::Marker visualTriangles;
  	visualization_msgs::MarkerArray visualTree;
  	visualization_msgs::MarkerArray visualBoundary;
  	visualization_msgs::Marker visualPath;
  	
  	fsd_common_msgs::Map boundaryDetections;
  	fsd_common_msgs::CarState carstate;
  	fsd_common_msgs::Map coneInMap;

  	double max_beam_cost_;
  	int max_iter_num_, max_search_num_;
  	Cost_n beam_weight_;
  	Cost_p path_weight_;


  	//subscriber and publisher
  	ros::Subscriber localMapSub;
  	ros::Subscriber poseSub;
  	ros::Publisher boundaryDetectionsPub;
  	ros::Publisher visualTrianglesPub;
  	ros::Publisher visualBoundaryPub;
  	ros::Publisher visualTreePub;
  	ros::Publisher visualPathPub;
  	ros::Publisher coneInMapPub;		//发布在map坐标系下的装桶点，由control接收



  	void loadParameters();
  	void localMapCallback(const fsd_common_msgs::Map &msg);
  	void poseCallback(const fsd_common_msgs::CarState &state);
  	void getConeInMap();
  	bool filter(fsd_common_msgs::Map &init_map);
  	void initSet(fsd_common_msgs::Map map, cv::Subdiv2D &coneSet, 
  					std::map<ConePos, char> &colorMap);
  	void getMidPoint(cv::Subdiv2D coneSet, std::map<ConePos, char> colorMap, 
  					std::map<int, PathPoint> &MidSet);
  	void searchPath(std::map<int, PathPoint> MidSet, SearchTree &Path);
  	void selectBestPath(SearchTree Path, std::vector<PathPoint> &BestPath);
  	void generateBoundary(std::vector<PathPoint> BestPath, 
  					fsd_common_msgs::Map &Boundary);

};

}