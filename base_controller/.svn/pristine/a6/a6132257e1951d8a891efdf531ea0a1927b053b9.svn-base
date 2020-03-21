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
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int can_read_from_motor()
{
  int a, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame_rev;
  struct can_filter rfilter;
	
  a = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  stpcpy(ifr.ifr_name, "can0");
  ioctl(a, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(a, (struct sockaddr*)&addr, sizeof(addr));
	
  rfilter.can_id = 0x83;
  rfilter.can_mask = CAN_SFF_MASK;
	
  nbytes = read(a, &frame_rev, sizeof(frame_rev));
  
  return nbytes;
}

int can_write_to_motor()
{
  int s, nbytes;
  struct can_frame frame;
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  frame.can_id = 0x82;
  frame.can_dlc = 8;
	
  frame.data[0] = 0;
  frame.data[1] = 0;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  nbytes = write(s, &frame, sizeof(frame));
  close(s);
}
 
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "status_monitor");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher motor_pub = n.advertise<std_msgs::String>("motor_status", 1000);

  can_write_to_motor();
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
	std::stringstream ss;
	
	
	int b = can_read_from_motor();
	if (b == -1)
	{
		ss << "motor is offline ";
		msg.data = ss.str();
		can_write_to_motor();
	}
	else
	{
		ss << "motor is working ";
		msg.data = ss.str();
	}

//    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    motor_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
