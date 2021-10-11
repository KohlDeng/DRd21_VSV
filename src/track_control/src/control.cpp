#include <control.hpp>
#include <Utils/param.h>

struct Param param_={0};

namespace ns_control
{

Control::Control(ros::NodeHandle &nh)
{
	nh_=nh;
  param_.getParams(nh);                   //from Utils/param.h
	loadParameters();
  subscribeToTopics();
  publishToTopics();
  lap_count_.data = 0;

}

void Control::loadParameters() {
  ROS_INFO("loading handle parameters");
  //subscribe
  //car_state_topic_name_="/slam/carstate";
  car_state_topic_name_="/estimation/slam/state";
  map_topic_name_="/map";
  //map_topic_name_="/planning/map/localmap";
  //publish
  ctrl_cmd_topic_name_="/control/pure_pursuit/control_command";
  ref_path_topic_name_="/visual/ref_path";
  predict_path_topic_name_="/visual/pre_path";
}

void Control::subscribeToTopics() 
{
  ROS_INFO("subscribe to topics");
  localMapSubscriber_ =
      nh_.subscribe(map_topic_name_, 10, &Control::localMapCallback, this);
  carStateSubscriber_ =
      nh_.subscribe(car_state_topic_name_, 10, &Control::carStateCallback, this);
  //lapCount node
  lapCountSubscriber_ = 
  	  nh_.subscribe("/slam/lapcount", 10, &Control::lapCountCallback, this);
}

void Control::publishToTopics()
{
  ROS_INFO("publish to topics");
  cmdPublisher_ = nh_.advertise<fsd_common_msgs::ControlCommand>(ctrl_cmd_topic_name_, 1);
  refPathPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>(ref_path_topic_name_, 1);
  prePathPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>(predict_path_topic_name_, 1);
}

void Control::carStateCallback(const fsd_common_msgs::CarState &state) {
  car_state_=state;
}

void Control::localMapCallback(const fsd_common_msgs::Map &map) {
  local_map_=map;
}

void Control::lapCountCallback(const std_msgs::Int16 &lap)
{
  lap_count_ = lap;
}

void Control::sendMsg() 
{
  cmdPublisher_.publish(cmd_);
  refPathPublisher_.publish(RefPath_);
  prePathPublisher_.publish(PrePath_);
}

bool Control::Check() {

  if (local_map_.cone_red.empty() || local_map_.cone_blue.empty()) {
    ROS_WARN_STREAM("Local Map Empty !");
    //ROS_INFO("velocity is %lf",param_.desire_vel);
    return false;
  }

}

void Control::runAlgorithm()
{
  if (!Check()) {
  ROS_WARN_STREAM("Check Error");
  return;
  }

  if(lap_count_.data<1)//raw is <2
  {
    param_.desire_vel = param_.desire_pp_vel;
    track_.setMap(local_map_);    
    track_.genTraj();
    track_.setState(VehicleState(car_state_, cmd_));
    track_.CalculateTraj(refline_);

    solver_ = &pure_pursuit_solver_;
    solver_->setState(VehicleState(car_state_, cmd_));
    solver_->setTrajectory(refline_);
    solver_->solve();
    cmd_ = solver_->getCmd();
    std::vector<float> color_ref = {1, 0, 0};
    std::vector<float> color_pre = {0, 1, 0};
    std::vector<float> color_init = {0, 0, 1};

    visual_trajectory(refline_, RefPath_, "/base_link", color_ref,
                      car_state_.header, true);
    std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
    std::cout << "throttle: " << cmd_.throttle.data << std::endl;
  }
  else if(lap_count_.data<3)
  {
    param_.desire_vel = param_.desire_mpc_vel;
    track_.setMap(local_map_);
    track_.genTraj();
    track_.setState(VehicleState(car_state_, cmd_));
    track_.CalculateTraj(refline_);
    solver_ = &mpc_solver_;
    solver_->setState(VehicleState(car_state_, cmd_));
    solver_->setTrajectory(refline_);
    solver_->solve();

    cmd_ = solver_->getCmd();

    std::vector<float> color_ref = {1, 0, 0};
    std::vector<float> color_pre = {0, 1, 0};
    std::vector<float> color_init = {0, 0, 1};

    visual_trajectory(refline_, RefPath_, "/base_link", color_ref,
                      car_state_.header, true);
    visual_trajectory(solver_->getTrajectory(), PrePath_, "/base_link",
                      color_pre, car_state_.header, true);

    std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
    std::cout << "throttle: " << cmd_.throttle.data << std::endl;
  }
  else{
    cmd_.throttle.data = -1.0;
    cmd_.steering_angle.data = 0.0;
    std::cout << "steering: " << cmd_.steering_angle.data << std::endl;
    std::cout << "throttle: " << cmd_.throttle.data << std::endl;
  }

}





}// namespace ns_control