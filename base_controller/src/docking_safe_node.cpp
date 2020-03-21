#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <sensor_msgs/Range.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "base_controller/pid.h"


class DockingSafeCtrl
{
public:
	DockingSafeCtrl();

private:
	void callback_sonar(const sensor_msgs::Range::ConstPtr& msg);
	void callback_vel(const geometry_msgs::Twist::ConstPtr& msg);
	void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);


	ros::NodeHandle nh_, private_nh;
	ros::Publisher vel_pub;
	ros::Subscriber sub_sonar, sub_vel , sub_imu;
	double safe_dist_, sonar_echo_, twist_offset_;

	boost::mutex mutex_;

	float spec_theta_, cur_theta_;
	float cur_ang_vel_;

	ros::Time go_back_time_; 

	PID_t theta_pid_;
	double dt_min_, kp_, ki_, kd_, integral_limit_, output_limit_;
};


DockingSafeCtrl::DockingSafeCtrl() : private_nh("~")
{
	private_nh.param("safe_dist", safe_dist_, 0.3); 
	private_nh.param("dt_min", dt_min_, 0.05); 
	private_nh.param("kp", kp_, 0.8); 
	private_nh.param("ki", ki_, 2.0); 
	private_nh.param("kd", kd_, 0.0); 
	private_nh.param("integral_limit", integral_limit_, 0.5); 
	private_nh.param("output_limit", output_limit_, 0.5); 


    vel_pub = nh_.advertise<geometry_msgs::Twist>("yocs_cmd_vel_mux/input/safe", 5);
	sub_sonar = nh_.subscribe("sonar_back", 1, &DockingSafeCtrl::callback_sonar, this);
	sub_vel = nh_.subscribe("cmd_vel", 5, &DockingSafeCtrl::callback_vel, this);
	sub_imu = nh_.subscribe("imu/data", 5, &DockingSafeCtrl::callback_imu, this);

	pid_init(&theta_pid_, dt_min_);

	pid_set_parameters(&theta_pid_, kp_, ki_, kd_, integral_limit_, output_limit_);
}

void DockingSafeCtrl::callback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
	boost::mutex::scoped_lock(mutex_);

	cur_ang_vel_ = msg->angular_velocity.z;

	cur_theta_ = tf::getYaw(msg->orientation);
	ROS_INFO("cur_theta_: %f", cur_theta_);

}


void DockingSafeCtrl::callback_sonar(const sensor_msgs::Range::ConstPtr& msg)
{
	boost::mutex::scoped_lock(mutex_);
	sonar_echo_ = msg->range;
	
}

void DockingSafeCtrl::callback_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	boost::mutex::scoped_lock(mutex_);
	
	if((ros::Time::now() - go_back_time_).toSec() > 2.0)
	{
      spec_theta_ = cur_theta_;
	}

	ros::Duration pid_duration = ros::Time::now() - go_back_time_;
	float pid_dt = pid_duration.toSec();

    twist_offset_ = pid_calculate(&theta_pid_, spec_theta_, cur_theta_, 1, pid_dt);
    ROS_INFO("spec_theta: %f", spec_theta_);
    ROS_INFO("cur_theta: %f", cur_theta_);
    ROS_INFO("pid calculate twist: %f", twist_offset_);

	go_back_time_ = ros::Time::now();

	geometry_msgs::Twist safe_vel_ = *msg;

	if((msg->linear.x < 0 || msg->angular.z != 0) && sonar_echo_ <= safe_dist_ ){

	  safe_vel_.linear.x = 0.0;
	  safe_vel_.angular.z = 0.0;

	}

	if((msg->linear.x < 0) && sonar_echo_ > safe_dist_ ){

		safe_vel_.angular.z = twist_offset_;
		
	}
	vel_pub.publish(safe_vel_);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "docking_safe_node");

  DockingSafeCtrl docking_safe_ctrl;

  
  ros::spin();
  
//  return 0;
}