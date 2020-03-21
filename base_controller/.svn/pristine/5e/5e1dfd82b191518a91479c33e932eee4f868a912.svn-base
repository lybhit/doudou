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


  int s, nbytes;
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
	SubscribeAndPublish():x(0),y(0),vx(0),vth(0)
	{
		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
		odom_sub_0 = n.subscribe("/imu/data", 3, &SubscribeAndPublish::callback, this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();    
		
	}
	
	void callback(const sensor_msgs::Imu & cmd_input)
    {
       yaw = tf::getYaw(cmd_input.orientation);

    


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
		
 
		
          
        nbytes = read(s, &frame_rev, sizeof(frame_rev)); //接收报文


        //串口接收的数据长度正确就处理并发布里程计数据消息
            vel_linear.data[0] = frame_rev.data[0];
            vel_linear.data[1] = frame_rev.data[1];
            vel_angular.data[0] = frame_rev.data[2];
            vel_angular.data[1] = frame_rev.data[3];

            current_time = ros::Time::now();

            double dt = (current_time - last_time).toSec();
            double delta_x = vel_linear.d * cos(yaw) * dt * 0.001;
            double delta_y = vel_linear.d * sin(yaw) * dt * 0.001;


            x += delta_x;
            y += delta_y;
            
            vx = vel_linear.d * 0.001;
            vth = vel_angular.d * 0.001;

            //将x 、y坐标，线速度缩小1000倍       
                
            //里程计的偏航角需要转换成四元数才能发布
            odom_quat = tf::createQuaternionMsgFromYaw(yaw);//将偏航角转换成四元数
            
            
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
            ROS_INFO("location: %f, %f, %f", x, y, yaw);
            ROS_INFO("velocity: %d, %d", vel_linear.d, vel_angular.d);
        


     last_time = ros::Time::now();
    }
	
	private:
	ros::NodeHandle n;
	ros::Publisher odom_pub;
	ros::Subscriber odom_sub_0;
	
	float x, y, yaw, vx, vth;
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

  //定义接收规则，只接收表示符等于0x83的报文
  rfilter.can_id = 0x83;
  rfilter.can_mask = CAN_SFF_MASK;
	
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}


