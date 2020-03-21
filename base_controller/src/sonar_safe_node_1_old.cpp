#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


#include <sensor_msgs/Range.h>


class SonarSafeCtrl
{
public:
	SonarSafeCtrl();

private:
	void callback_sonar(const sensor_msgs::Range::ConstPtr& msg);
	void callback_vel(const geometry_msgs::Twist::ConstPtr& msg);


	ros::NodeHandle nh_, private_nh;
	ros::Publisher vel_pub;
	ros::Subscriber sub_sonar, sub_vel;
	double safe_dist_, sonar_echo;
};


SonarSafeCtrl::SonarSafeCtrl() : private_nh("~")
{
	private_nh.param("safe_dist", safe_dist_, 0.3); 

    vel_pub = nh_.advertise<geometry_msgs::Twist>("yocs_cmd_vel_mux/input/safe", 10);
	sub_sonar = nh_.subscribe("sonar_back", 5, &SonarSafeCtrl::callback_sonar, this);
	sub_vel = nh_.subscribe("cmd_vel", 5, &SonarSafeCtrl::callback_vel, this);
}



void SonarSafeCtrl::callback_sonar(const sensor_msgs::Range::ConstPtr& msg)
{
	sonar_echo = msg->range;
	
}

void SonarSafeCtrl::callback_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	geometry_msgs::Twist safe_vel_ = *msg;

//	ROS_INFO("safe_dist_: %f", safe_dist_);
	if((msg->linear.x < 0 || msg->angular.z != 0) && sonar_echo < safe_dist_ ){
//	if(sonar_echo < safe_dist_ ){
		safe_vel_.linear.x = 0.0;
		safe_vel_.angular.z = 0.0;
		
//	vel_pub.publish(safe_vel_);
	}
	vel_pub.publish(safe_vel_);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "sonar_safe_node_1");

  SonarSafeCtrl sonar_safe_ctrl;

  
  ros::spin();
  
//  return 0;
}
