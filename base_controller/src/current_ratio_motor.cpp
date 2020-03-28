/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"

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

//电机报警后是，电机速度置0,这种状态下过流警告会消失，如果电机恢复速度，然后继续过流报警，这样子会进入死循环的情况，这需要怎么处理会好呢？？？
//报警一次，一直持续报警？

extern bool electric_alarm;

int s, nbytes;
struct can_frame frame[2];
struct can_filter rfilter;

union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

// union int_char
// {
//   int d;
//   unsigned char data[4];
// }linear_vel, angular_vel, left_vel, right_vel;


void ask_motor_para()
{
  ROS_INFO("goto ask motor");

  frame[0].data[0] = 0xE8;
  frame[0].data[1] = 0x02;
  frame[0].data[2] = 0x50;
  frame[0].data[3] = 0x00;
  frame[0].data[4] = 0x00;
  frame[0].data[5] = 0x00;
  frame[0].data[6] = 0x00;
  frame[0].data[7] = 0x00;


  send_frame(&frame[0], s);

//	close(s);
}


int main(int argc, char** argv)
{
  int threshold_;
  float duration_;
  bool switch_;

  bool alarm_flag = false;

  struct sockaddr_can addr;
  struct ifreq ifr;
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  frame[0].can_id = 0x601;
  frame[0].can_dlc = 8;

  rfilter.can_id = 0x581;
  rfilter.can_mask = CAN_SFF_MASK;
	
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  ros::init(argc, argv, "current_ratio_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("threshold", threshold_, 500);
  private_nh.param<float>("duration", duration_, 2.0);
  private_nh.param<bool>("switch", switch_, false);

  ros::Publisher motor_pub = n.advertise<std_msgs::String>("motor_status", 10);

  ros::Time time_mark;
    
  ros::Rate loop_rate(10);    

  std_msgs::String str;

  while (ros::ok())
 {
    ask_motor_para();

    read(s, &frame[1], sizeof(frame[1]));

    // ROS_INFO("ID:%x  data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", 
    //         frame[1].can_id, frame[1].data[0], frame[1].data[1],
    //         frame[1].data[2],frame[1].data[3],frame[1].data[4],frame[1].data[5], frame[1].data[6], frame[1].data[7]);

    uint16_t val_1 = (0xff & frame[1].data[4]) | ((0xff & frame[1].data[5]) << 8);
    uint16_t val_2 = (0xff & frame[1].data[6]) | ((0xff & frame[1].data[7]) << 8);

    // ROS_INFO("val_1 = %d", val_1);
    // ROS_INFO("val_2 = %d", val_2);

    str.data = "ok";


    //如果开关打开，则只要警报一次，就一直会报警；否则，当电机过载率降低，消除报警信号
    if(!switch_){
      if(val_1 >= threshold_ || val_2 >= threshold_)
      {
      }
      else
      {
          time_mark = ros::Time::now();
      }

      if((ros::Time::now() - time_mark).toSec() >= duration_)
      {
        str.data = "alarm";
      }

    }else{
      if(val_1 >= threshold_ || val_2 >= threshold_)
      {
      }
      else
      {
        if(!alarm_flag){
          time_mark = ros::Time::now();
        }   
      }

      if((ros::Time::now() - time_mark).toSec() >= duration_)
      {
        str.data = "alarm";
        alarm_flag = true;
      }
    }
    
    motor_pub.publish(str);

    ros::spinOnce();//周期执行
    loop_rate.sleep();//周期休眠
        
 }  
 return 0;
}


