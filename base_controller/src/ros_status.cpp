#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
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
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int s;
ros::Time time_mark;
uint16_t val_1 = 0;
uint16_t val_2 = 0;

void recv_manager(int sockfd)
{
    struct can_frame frame;
    int fd = sockfd;
    fd_set rdfs;
    int ret;

    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    if ((ret = select((fd)+1, &rdfs, NULL, NULL, NULL)) <= 0) 
    {
    //    perror("select");
        running = 0;
    }
    ROS_INFO("Start Receive data --1--");
    if (FD_ISSET(fd, &rdfs)) 
    {
      ROS_INFO("Start Receive data --2--");

      ret = recv_frame(&frame, fd);

      if(frame.can_id == 0x181)
      {
        val_1 = (0xff & frame.data[4]) | ((0xff & frame.data[5]) << 8);
      }

      if(frame.can_id == 0x182)
      {
        val_2 = (0xff & frame.data[4]) | ((0xff & frame.data[5]) << 8);
      }        
    }
    fflush(stdout);
} 

void can_read_from_motor()
{
  uint16_t threshold = 400;
	
  recv_manager(s);
  ROS_INFO("over current ratio_1 = %d", val_1);
  ROS_INFO("over current ratio_2 = %d", val_2);
  if(val_1 >= threshold || val_2 >= threshold)
  {
      
  }
  else
  {
  //  clear_flag = true;
    time_mark = ros::Time::now();
  }  
}

void can_write_to_motor()
{
  int s, nbytes;
  struct can_frame frame[2];
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
	
  frame[0].data[0] = 0;
  frame[0].data[1] = 0;
  frame[0].data[2] = 0;
  frame[0].data[3] = 0;

  frame[1].can_id = 0x402;
  frame[1].can_dlc = 4;

  frame[1].data[0] = 0;
  frame[1].data[1] = 0;
  frame[1].data[2] = 0;
  frame[1].data[3] = 0;

  send_frame(&frame[0], s);
  send_frame(&frame[1], s);

  close(s);
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_status");

  ros::NodeHandle n;

  ros::Publisher motor_pub = n.advertise<std_msgs::String>("motor_status", 1000);

  can_write_to_motor();
  ros::Rate loop_rate(20);

  if(can_conect(&s, "can0") < 0)
  {
    exit(0);
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  std_msgs::String str;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

//    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    
    can_read_from_motor();

    str.data = "ok";
    
    if((ros::Time::now() - time_mark).toSec() >= 3.0)
    {
      str.data = "alarm";
    }
    
    motor_pub.publish(str);
    ros::spinOnce();

    loop_rate.sleep();
  }
  
  if(!ros::ok())
  {
    for(int i = 0; i < 5; ++i)
    {
      can_write_to_motor();
    }
  }

  return 0;
}
