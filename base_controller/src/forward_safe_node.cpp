#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

class ForwardSafeCtrl
{
public:
  ForwardSafeCtrl();

  void update(const ros::TimerEvent& te)
  {
  	std_msgs::String st;

  	if((sonar_echo_ <= safe_dist_)|| (laser_echo_ <= safe_dist_))
  	{	  
  	  st.data = "true";
  	}
  	else
  	{
      st.data = "false";
  	}
  	obs_state_pub.publish(st);
  }

  void run()
  {
  	updateTimer = ros::Timer(nh_.createTimer(ros::Duration(0.1/rate_),
                                                &ForwardSafeCtrl::update,
                                                this));
  }

private:
  void callback_sonar(const sensor_msgs::Range::ConstPtr& msg);
  void callback_vel(const geometry_msgs::Twist::ConstPtr& msg);
  void callback_laser(const std_msgs::Float32::ConstPtr& msg);
  void callback_ctrl_mode(const std_msgs::String::ConstPtr& msg);

  ros::NodeHandle nh_, private_nh;
  ros::Publisher vel_pub, obs_state_pub;
  ros::Subscriber sub_sonar, sub_vel, sub_laser, sub_ctrl_mode;
  double safe_dist_, sonar_echo_, laser_echo_;
  std::string ctrl_mode_;

  boost::mutex obs_mutex_;

  ros::Timer updateTimer;
  int rate_;
};

ForwardSafeCtrl::ForwardSafeCtrl() : ctrl_mode_("auto"), private_nh("~")
{
  private_nh.param("safe_dist", safe_dist_, 0.3); 
  private_nh.param("rate", rate_, 10); 

  vel_pub = nh_.advertise<geometry_msgs::Twist>("yocs_cmd_vel_mux/safe/cmd_vel", 5);
  obs_state_pub = nh_.advertise<std_msgs::String>("detect_obs", 2);

  sub_sonar = nh_.subscribe("sonar_front", 5, &ForwardSafeCtrl::callback_sonar, this);
  sub_vel = nh_.subscribe("yocs_cmd_vel_mux/output/cmd_vel", 5, &ForwardSafeCtrl::callback_vel, this);
  sub_laser = nh_.subscribe("min_dis", 1, &ForwardSafeCtrl::callback_laser, this);
  sub_ctrl_mode = nh_.subscribe("control_mode", 1, &ForwardSafeCtrl::callback_ctrl_mode, this);
}

void ForwardSafeCtrl::callback_sonar(const sensor_msgs::Range::ConstPtr& msg)
{
  boost::mutex::scoped_lock(obs_mutex_);
  sonar_echo_ = msg->range;	

}

void ForwardSafeCtrl::callback_laser(const std_msgs::Float32::ConstPtr& msg)
{
  boost::mutex::scoped_lock(obs_mutex_);
  laser_echo_ = msg->data;
}

void ForwardSafeCtrl::callback_ctrl_mode(const std_msgs::String::ConstPtr& msg)
{
  ctrl_mode_ = msg->data;
}

void ForwardSafeCtrl::callback_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist safe_vel_ = *msg;
//	ROS_INFO("safe_dist_: %f", safe_dist_);
//	if((msg->linear.x != 0 || msg->angular.z != 0) && sonar_echo_ < safe_dist_ ){

  //传感器检测到障碍物距离小于阈值时：如果控制模式为自动，则停车；如果控制模式为手动，则手动控制可控(可以后退，不可以前进)；
  if(sonar_echo_ < safe_dist_ || laser_echo_ < (safe_dist_ - 0.06)){
    if(ctrl_mode_ == "manual")
	{
	  if(safe_vel_.linear.x >= 0.0)
	  {
        safe_vel_.linear.x = 0.0;
	    safe_vel_.angular.z = 0.0;		
	  }
	  else
	  {
        safe_vel_.linear.x = -0.1;
	    safe_vel_.angular.z *= 0.5;		
	  }
	}
	else
	{
      safe_vel_.linear.x = 0.0;
	  safe_vel_.angular.z = 0.0;		
	}
  }
  vel_pub.publish(safe_vel_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "forward_safe_node");

  ForwardSafeCtrl forward_safe_ctrl;
  
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  ros::waitForShutdown();
//  ros::spin();
  
  return 0;
}
