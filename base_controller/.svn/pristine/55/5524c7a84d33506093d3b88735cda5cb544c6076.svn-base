#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <boost/thread.hpp>


#include <sensor_msgs/Range.h>


class FrontSonarSafeCtrl
{
public:
	FrontSonarSafeCtrl();

private:
	void callback_sonar(const sensor_msgs::Range::ConstPtr& msg);
	void callback_vel(const geometry_msgs::Twist::ConstPtr& msg);
	void callback_laser(const std_msgs::Float32::ConstPtr& msg);


	ros::NodeHandle nh_, private_nh;
	ros::Publisher vel_pub;
	ros::Subscriber sub_sonar, sub_vel, sub_laser;
	double safe_dist_, sonar_echo, laser_echo;
};


FrontSonarSafeCtrl::FrontSonarSafeCtrl() : private_nh("~")
{
  private_nh.param("safe_dist", safe_dist_, 0.3); 

  vel_pub = nh_.advertise<geometry_msgs::Twist>("yocs_cmd_vel_mux/safe/cmd_vel", 5);
  sub_sonar = nh_.subscribe("sonar_front", 5, &FrontSonarSafeCtrl::callback_sonar, this);
  sub_vel = nh_.subscribe("yocs_cmd_vel_mux/output/cmd_vel", 5, &FrontSonarSafeCtrl::callback_vel, this);
  sub_laser = nh_.subscribe("min_dis", 1, &FrontSonarSafeCtrl::callback_laser, this);
}



void FrontSonarSafeCtrl::callback_sonar(const sensor_msgs::Range::ConstPtr& msg)
{
  sonar_echo = msg->range;
	
}

void FrontSonarSafeCtrl::callback_laser(const std_msgs::Float32::ConstPtr& msg)
{
  laser_echo = msg->data;
}	
void FrontSonarSafeCtrl::callback_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist safe_vel_ = *msg;

//	ROS_INFO("safe_dist_: %f", safe_dist_);
//	if((msg->linear.x != 0 || msg->angular.z != 0) && sonar_echo < safe_dist_ ){
	if(sonar_echo < safe_dist_ || laser_echo < (safe_dist_ - 0.06)){
		safe_vel_.linear.x = 0.0;
		safe_vel_.angular.z = 0.0;
		
	}
	vel_pub.publish(safe_vel_);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "cmd_safe_node");

  FrontSonarSafeCtrl front_sonar_safe_ctrl;
  
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  
//  ros::spin();
  
  return 0;
}
