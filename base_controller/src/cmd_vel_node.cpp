/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
#include "cruiser_tx2_can.h"
using namespace std;

bool stop_flag = false;

bool rev_flag_1 = false;
bool rev_flag_2 = false;

int s, nbytes;
struct can_frame frame[2];

void recv_manager(struct can_frame input_frame, int sockfd)
{
    struct can_frame frame = input_frame;
    int fd = sockfd;
    fd_set rdfs;
    int ret;
    struct timeval timeout;

    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    if ((ret = select((fd)+1, &rdfs, NULL, NULL, &timeout)) <= 0) //如果连接成功就跳过
    {
    //flag_c = true;
     perror("select");
     rev_flag_1 = false;
     rev_flag_2 = false;
    
//     flag_c = true;
     //ROS_INFO("Flag_c %d", flag_c);
     return;
    }
    
    if (FD_ISSET(fd, &rdfs)) 
    {
        ret = recv_frame(&frame, fd);

        if(ret < 0)
        {
            rev_flag_1 = false;
            rev_flag_2 = false;
        }
        else
        {
            if(frame.can_id == 0x181)
            {
              rev_flag_1 = true;

            }
            if(frame.can_id == 0x182)
            {
              rev_flag_2 = true;
            }
        }
        
        
    }
    fflush(stdout);
}
// union shortint_char
// {
//   short int d;
//   unsigned char data[2];
// }linear_vel,angular_vel,vel_linear,vel_angular;

union int_char
{
  int d;
  unsigned char data[4];
}linear_vel, angular_vel, left_vel, right_vel;


void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
//  ROS_INFO("enter callback");
  float r = 0.065;//半径
  float L = 0.3;  //轮距

  double vel_linear = cmd_input.linear.x;
  double vel_angular = cmd_input.angular.z;

  if(vel_linear > 0.5)
  {
    vel_linear = 0.5;
  }

  if(vel_linear < -0.2)
  {
    vel_linear = -0.2;
  }

  if(vel_angular > 0.6)
  {
    vel_angular = 0.6;
  }

  if(vel_angular < -0.6)
  {
    vel_angular = -0.6;
  }
  
  // 1m/s --> 147r/min ---- 对应的是1470 * 0.1r/min
  // if(!electric_alarm){
   if(stop_flag)
   {
     right_vel.d = 0;
     left_vel.d  = 0;
   }
   else
   {
     right_vel.d = -1 * (vel_linear + vel_angular * L / 2) * 1470;//右轮需要取反
     left_vel.d  = (vel_linear - vel_angular * L / 2) * 1470;
   }
  
  // ROS_INFO("Right_vel = %d", right_vel.d);
  // ROS_INFO("Left_vel = %d", left_vel.d);

  frame[0].data[0] = right_vel.data[0];
  frame[0].data[1] = right_vel.data[1];
  frame[0].data[2] = right_vel.data[2];
  frame[0].data[3] = right_vel.data[3];
  // ROS_INFO("frame[0]: %d, %d, %d, %d", right_vel.data[3], right_vel.data[2], right_vel.data[1], right_vel.data[0]);

  frame[1].data[0] = left_vel.data[0];
  frame[1].data[1] = left_vel.data[1];
  frame[1].data[2] = left_vel.data[2];
  frame[1].data[3] = left_vel.data[3];
  // ROS_INFO("frame[1]: %d, %d, %d, %d", left_vel.data[3], left_vel.data[2], left_vel.data[1], left_vel.data[0]);  

  recv_manager(frame[0], s);
  recv_manager(frame[1], s);

  if(rev_flag_1 == true && rev_flag_2 == true)
  {
    // ROS_INFO("cmd_vel_node send data");
    send_frame(&frame[0], s);
    // ROS_INFO("cmd_vel_node send data");
    send_frame(&frame[1], s);
  }  

//	close(s);
}

void callback_1(const std_msgs::String & input)//订阅/ros_status主题回调函数
{
  if(input.data == "alarm")
  {
    stop_flag = true;
    ROS_INFO("motor alarm");
  }
//  else
//  {
//     stop_flag = false;
//  }
}


int main(int argc, char** argv)
{

  struct sockaddr_can addr;
  struct ifreq ifr;
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  frame[0].can_id = 0x401;
  frame[0].can_dlc = 4;

  frame[1].can_id = 0x402;
  frame[1].can_dlc = 4;

  ros::init(argc, argv, "cmd_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄

  ros::Subscriber sub = n.subscribe("/cmd_vel", 3, callback);
  ros::Subscriber sub_1 = n.subscribe("/motor_status", 5, callback_1);
    
  ros::Rate loop_rate(20);    

  while (ros::ok())
 {
 
    ros::spinOnce();//周期执行
    loop_rate.sleep();//周期休眠
        
 } 

 if(!ros::ok())
  {
    for(int i = 0; i < 5; ++i)
    {
      for(int j = 0; j < 4; ++j)
      {
        frame[0].data[j] = 0; 
        frame[1].data[j] = 0; 
      }

      send_frame(&frame[0], s);
      send_frame(&frame[1], s);
      ROS_INFO("Stop the car!!!");
    }
  }
// return 0;
}


