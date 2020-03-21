#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"

ros::Publisher pub;

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	float x, y;
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;

	std_msgs::String msg_pub;

	if((x >= 9.2 && x <= 15 && y >= 1.8 && y <= 6.0) || (x >= 12.8 && x <= 14.0 && y >= 6.0 && y <= 17.7))
	{
		ROS_INFO("Robot is at the reasonable place.");
	//	msg_pub.data = "Status: ok! Robot is at the reasonable place.";
		msg_pub.data = "1";
	}
	else
	{
		ROS_ERROR("Robot is out of the reasonable place.");
	//	msg_pub.data = "Status: wrong! Robot is out of the reasonable place.";
		msg_pub.data = "0";
	}

	pub.publish(msg_pub);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_check_node");//初始化节点
    ros::NodeHandle n;  //定义节点进程句柄

    ros::Subscriber pos_sub = n.subscribe("/amcl_pose", 1, callback);

    pub = n.advertise<std_msgs::String>("/pose_safe", 1);

    ros::spin();
    return 0;
}
