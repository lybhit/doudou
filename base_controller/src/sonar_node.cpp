/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>

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

using namespace std;

#define MAX_DISTANCE 2.5

int can_write_to_sonar()
{
  int s, nbytes;
  struct can_frame frame;
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can1");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  frame.can_id = 0x0601;
  frame.can_dlc = 3;
  
  frame.data[0] = 0xb3;
  frame.data[1] = 0x10;
  frame.data[2] = 0xff;

  nbytes = write(s, &frame, sizeof(frame));
  close(s);
}

int cal(unsigned short int a, unsigned short int b)
{
    unsigned short int c = a/16;
    unsigned short int d = a%16;
    unsigned short int e = b/16;
    unsigned short int f = b%16;
    unsigned int g = c*1000 + d*100 + e*10 + f;
    return g;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄
  ros::Publisher sonar_pubs[4];
  for(int i = 0; i < 4; ++i){
  	stringstream ss;
  	ss << "sonar_" << i;
  	sonar_pubs[i] = n.advertise<sensor_msgs::Range>(ss.str(),10);
  }

  int s, nbytes[2];
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame_rev[2];
  struct can_filter rfilter[2];
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can1");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can1 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

	
  //定义接收规则，只接收表示符等于0x83的报文
  rfilter[0].can_id = 0x611;
  rfilter[0].can_mask = CAN_SFF_MASK;

  rfilter[1].can_id = 0x612;
  rfilter[1].can_mask = CAN_SFF_MASK;
	
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  sensor_msgs::Range range;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.min_range = 0.0;
  range.max_range = MAX_DISTANCE;

  float dis[4];
  std::string frame("sonar_");

  ros::Rate loop_rate(20);    

  can_write_to_sonar();

  while (ros::ok())
  {
      nbytes[0] = read(s, &frame_rev[0], sizeof(frame_rev[0])); //接收报文
      nbytes[1] = read(s, &frame_rev[1], sizeof(frame_rev[1])); //接收报文

      if (nbytes == 0)
        can_write_to_sonar();
      else{
          if(frame_rev[0].can_id = 0x611)
          {
            dis[0] = cal((unsigned short int)frame_rev[0].data[2], (unsigned short int)frame_rev[0].data[3]) * 0.001;
            dis[1] = cal((unsigned short int)frame_rev[0].data[4], (unsigned short int)frame_rev[0].data[5]) * 0.001;
          }

          if(frame_rev[1].can_id = 0x612)
          {
            dis[2] = cal((unsigned short int)frame_rev[1].data[0], (unsigned short int)frame_rev[1].data[1]) * 0.001;
            dis[3] = cal((unsigned short int)frame_rev[1].data[2], (unsigned short int)frame_rev[1].data[3]) * 0.001;
          }
          //ROS_INFO("Distance: %f ", a);

          for(int j = 0; j<4; j++)
          {
            if(dis[j] > MAX_DISTANCE || dis[j] < 0)
            dis[j] = MAX_DISTANCE;
          }
      
        for (int i = 0; i < 4; ++i) 
        {

          range.header.stamp = ros::Time::now();
          range.header.frame_id = frame + std::to_string(i);
          range.field_of_view = 0.5;
          range.range = dis[i];
          sonar_pubs[i].publish(range);
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
   }
      return 0;
}


