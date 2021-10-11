#pragma once

#include "ros/ros.h"
#include <iostream>
#include <string>

#include <std_msgs/Int16.h>

#include "Utils/types.h"
#include "Utils/param.h"
#include "Utils/visual.h"

#include "Track/track_base.h"
#include "Track/trackdrive_track.h"
#include "Solver/solver_base.h"
#include "Solver/mpc_solver.h"
#include "Solver/pure_pursuit_solver.h"

namespace ns_control 
{
class Control
{
public:
	Control(ros::NodeHandle &nh);
	void loadParameters();
	void subscribeToTopics();
  	void publishToTopics();
  	void runAlgorithm();
  	void sendMsg();
  	bool Check();

  	visualization_msgs::MarkerArray RefPath_;
  	visualization_msgs::MarkerArray PrePath_;

private:
 	void localMapCallback(const fsd_common_msgs::Map &map);
 	void carStateCallback(const fsd_common_msgs::CarState &msg);
 	void lapCountCallback(const std_msgs::Int16 &lap);

private:
	ros::Subscriber localMapSubscriber_;
	ros::Subscriber carStateSubscriber_;
	//lapCount node
	ros::Subscriber lapCountSubscriber_;

	ros::Publisher cmdPublisher_;
	ros::Publisher refPathPublisher_;
	ros::Publisher prePathPublisher_;
	//topic name
	std::string car_state_topic_name_;
	std::string map_topic_name_;
  	std::string ctrl_cmd_topic_name_;
  	std::string predict_path_topic_name_;
  	std::string ref_path_topic_name_;

  	ros::NodeHandle nh_;

  	//  global varibles in class member
  	Autox_Track track_;

  	Solver *solver_;
  	MPC_Solver mpc_solver_;
  	Pure_Pursuit_Solver pure_pursuit_solver_;

  	fsd_common_msgs::Map local_map_;
  	fsd_common_msgs::CarState car_state_;
  	std_msgs::Int16 lap_count_;
  	//publish
  	fsd_common_msgs::ControlCommand cmd_;

  	Trajectory trajectory_;
  	Trajectory refline_;

  	bool is_init = false;

};




} //namespace ns_control