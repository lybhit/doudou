/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#define PI 3.141592


  int s, nbytes[2];
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame_rev;
  struct can_filter rfilter;
  
  ros::Time current_time, last_time;
  
  
  union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

  
class SubscribeAndPublish
{
	public:
	SubscribeAndPublish():x(0),y(0),vx(0),vth(0),th(0)
	{
		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
		odom_sub_0 = n.subscribe("/imu/data", 3, &SubscribeAndPublish::callback, this);
             current_time = ros::Time::now();
             last_time = ros::Time::now();    
		
	}
	
	void callback(const sensor_msgs::Imu & cmd_input)
  {
    //yaw = tf::getYaw(cmd_input.orientation);
    vth = cmd_input.angular_velocity.z;

    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

    nav_msgs::Odometry odom;//定义里程计对象

    geometry_msgs::Quaternion odom_quat;//四元数变量

    //定义covariance矩阵，作用为解决位置和速度的不同测量的不确定性
    float covariance[36] = {0.0001, 0, 0, 0, 0, 0, //covariance on gps_x
                      0, 0.0001, 0, 0, 0, 0, //covariance on gps_y
                      0, 0, 9999, 0, 0, 0, //covariance on gps_z
                      0, 0, 0, 9999, 0, 0, //large covariance on rot x
                      0, 0, 0, 0, 9999, 0, //large covariance on rot y
                      0, 0, 0, 0, 0, 0.01};//large covariance on rot z
  //载入covariance矩阵
    for(int i=0; i<36; i++)
    {
      odom.pose.covariance[i] = covariance[i];
    }

      
    nbytes[0] = read(s, &frame_rev, sizeof(frame_rev)); //接收报文


    //串口接收的数据长度正确就处理并发布里程计数据消息
    vel_linear.data[0] = frame_rev.data[4];
    vel_linear.data[1] = frame_rev.data[5];
    vel_angular.data[0] = frame_rev.data[6];
    vel_angular.data[1] = frame_rev.data[7];

    if(std::abs(vel_linear.d) == 3)
    {
      vel_linear.d = 0;
    }

    if(vel_angular.d == 20)
    {
      vel_angular.d = 0;
    }

    // ROS_INFO("Linear_vel data[0] = %d",  vel_linear.data[0]);
    // ROS_INFO("Linear_vel data[1] = %d",  vel_linear.data[1]);

    // ROS_INFO("Angular_vel data[0] = %d",  vel_angular.data[0]);
    // ROS_INFO("Angular_vel data[1] = %d",  vel_angular.data[1]);

    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double delta_x = vel_linear.d * cos(th) * dt * 0.001;
    double delta_y = vel_linear.d * sin(th) * dt * 0.001;
    
    if(std::abs(vth) <= 0.001 || std::abs(vel_angular.d * 0.001) <= 0.001){
      vth = 0.0;
    }
    double delta_th = vth * dt;


    x += delta_x;
    y += delta_y;

    th += delta_th;
    
    vx = vel_linear.d * 0.001;

    if(vth == 0.0 || std::abs(vel_angular.d * 0.001) <= 0.001)
    {
      vth = 0.0;
    }else{
      vth = vel_angular.d * 0.001;
    }
    

    //将x 、y坐标，线速度缩小1000倍       
        
    //里程计的偏航角需要转换成四元数才能发布
    odom_quat = tf::createQuaternionMsgFromYaw(th);//将偏航角转换成四元数
          
    //载入里程计时间戳
    odom.header.stamp = ros::Time::now();
    //里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
    //载入线速度和角速度
    odom.twist.twist.linear.x = vx;

    odom.twist.twist.angular.z = vth;
    //发布里程计
    odom_pub.publish(odom);

    last_time = ros::Time::now();
  }
	
	private:
	ros::NodeHandle n;
	ros::Publisher odom_pub;
	ros::Subscriber odom_sub_0;
	
	float x, y, vx, vth, th;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");//初始化串口节点

  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  //定义接收规则，只接收表示符等于0x181的报文
  rfilter.can_id = 0x181;
  rfilter.can_mask = CAN_SFF_MASK;
	
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}


