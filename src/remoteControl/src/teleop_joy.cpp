#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fsd_common_msgs/ControlCommand.h>


class TeleopKey
{
public:
  TeleopKey();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;

};


TeleopKey::TeleopKey()
{

  nh_.param("axis_linear", linear_, 8);
  nh_.param("axis_angular", angular_, 0);
  nh_.param("scale_angular", a_scale_, 0.00003051851);
  nh_.param("scale_linear", l_scale_, 0.00003051851);

  vel_pub = nh_.advertise<fsd_common_msgs::ControlCommand>("/control/pure_pursuit/control_command", 5);
  joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopKey::joyCallback, this);

}

void TeleopKey::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  fsd_common_msgs::ControlCommand cmd;
  cmd.steering_angle.data = a_scale_*joy->axes[angular_]*10000;
  cmd.throttle.data = -l_scale_*joy->axes[linear_]*10000;
  vel_pub.publish(cmd);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopKey teleop_key;

  ros::spin();
}