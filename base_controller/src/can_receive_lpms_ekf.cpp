/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
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

#define PI 3.141592


int s, nbytes[2];
struct sockaddr_can addr;
int right_rot_vel = 0;
int left_rot_vel = 0;


uint16_t val_1;
uint16_t val_2;

//  struct ifreq ifr;
//  struct can_frame frame_rev[2];
//  struct can_filter rfilter[2];
  
  ros::Time current_time, last_time;

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
//    ROS_INFO("Start Receive data --1--");
    if (FD_ISSET(fd, &rdfs)) 
    {
//      ROS_INFO("Start Receive data --2--");
        ret = recv_frame(&frame, fd);
        if(frame.can_id == 0x181)
        {
          right_rot_vel = (int)frame.data[0] | (((int)frame.data[1]) << 8) | (((int)frame.data[2]) << 16) | (((int)frame.data[3]) << 24);

          val_1 = (0xff & frame.data[4]) | ((0xff & frame.data[5]) << 8);

        }
       if(frame.can_id == 0x182)
        {
          left_rot_vel = (int)frame.data[0] | (((int)frame.data[1]) << 8) | (((int)frame.data[2]) << 16) | (((int)frame.data[3]) << 24);

          val_2 = (0xff & frame.data[4]) | ((0xff & frame.data[5]) << 8);

        }        
    }
    fflush(stdout);
}
  
  
  union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

  
class SubscribeAndPublish
{
	public:
	SubscribeAndPublish():x(0),y(0),vx(0),vth(0),th(0),private_nh("~")
	{
    private_nh.param<float>("offset_imu", offset_imu_, 0.01);
    private_nh.param<float>("offset_odom", offset_odom_, 0.002);
    private_nh.param<int>("threshold_current", threshold_current_, 500);

		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);
    motor_pub = n.advertise<std_msgs::String>("motor_status", 10);
		odom_sub_0 = n.subscribe("/imu/data", 3, &SubscribeAndPublish::callback, this);

    current_time = ros::Time::now();
    last_time = ros::Time::now();    
		time_mark = ros::Time::now();
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

          
//        nbytes[0] = read(s, &frame_rev[0], sizeof(frame_rev[0])); //接收报文

//        nbytes[1] = read(s, &frame_rev[1], sizeof(frame_rev[1])); //接收报文
//        recv_frame(&frame_rev[0], s);

//        ROS_INFO("Receive data 0: %d, %d, %d, %d, %d, %d", frame_rev[0].data[5],frame_rev[0].data[4],frame_rev[0].data[3],frame_rev[0].data[2],frame_rev[0].data[1],frame_rev[0].data[0]);

//        recv_frame(&frame_rev[1], s);
//        ROS_INFO("Receive data 1: %d, %d, %d, %d, %d, %d", frame_rev[1].data[5],frame_rev[1].data[4],frame_rev[1].data[3],frame_rev[1].data[2],frame_rev[1].data[1],frame_rev[1].data[0]);

        double r = 0.065;
        double L = 0.3;

//        int right_rot_vel = 0;
//        int left_rot_vel = 0; 
        
//        if(frame_rev[0].can_id == 181)
//        int  right_rot_vel = (int)frame_rev[0].data[0] | (((int)frame_rev[0].data[1]) << 8) | (((int)frame_rev[0].data[2]) << 16) | (((int)frame_rev[0].data[3]) << 24);
        
//        if(frame_rev[1].can_id == 182)
//        int left_rot_vel = (int)frame_rev[1].data[0] | (((int)frame_rev[1].data[1]) << 8) | (((int)frame_rev[1].data[2]) << 16) | (((int)frame_rev[1].data[3]) << 24);
  
        recv_manager(s);   

        if(val_1 >= threshold_current_ || val_2 >= threshold_current_)
        {    
        }
        else
        {
        //  clear_flag = true;
          time_mark = ros::Time::now();
        }

        std_msgs::String str;

        str.data = "ok";
  
        if((ros::Time::now() - time_mark).toSec() >= 3.0)
        {
          str.data = "alarm";
        }
        
        motor_pub.publish(str);
               
        if(right_rot_vel == 0 || right_rot_vel == -1)
           right_rot_vel = 0;
  
        if(left_rot_vel == 0 || left_rot_vel == -1)
           left_rot_vel = 0;

        // ROS_INFO("Right_rot_vel: = %d", right_rot_vel);
        // ROS_INFO("Left_rot_vel: = %d", left_rot_vel);
       
        double left_linear = left_rot_vel * 2 * PI * r / 60;
        double right_linear = -right_rot_vel * 2 * PI * r / 60;//右轮速度取反，沿着出线的方向看过去，顺时针速度为正

        double v_linear =  (left_linear + right_linear) * 0.5;
        double v_angular =  (right_linear - left_linear)/ L;


        //串口接收的数据长度正确就处理并发布里程计数据消息
            // vel_linear.data[0] = frame_rev.data[0];
            // vel_linear.data[1] = frame_rev.data[1];
            // vel_angular.data[0] = frame_rev.data[2];
            // vel_angular.data[1] = frame_rev.data[3];

            current_time = ros::Time::now();

            double dt = (current_time - last_time).toSec();
            // double delta_x = vel_linear.d * cos(th) * dt * 0.001;
            // double delta_y = vel_linear.d * sin(th) * dt * 0.001;

            // double delta_th = vth * dt;

            if(std::abs(vth) < offset_imu_ || std::abs(v_angular) < offset_odom_)
            {
              vth = 0;
            }

            double delta_x = v_linear * cos(th) * dt;
            double delta_y = v_linear * sin(th) * dt;
            double delta_th = vth * dt;


            x += delta_x;
            y += delta_y;

            th += delta_th;
            
            // vx = vel_linear.d * 0.001;
//            vth = vel_angular.d * 0.001;

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
            // odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.x = v_linear;
//            odom.twist.twist.angular.z = vel_angular.d * 0.001;
            // odom.twist.twist.angular.z = vth;
            odom.twist.twist.angular.z = vth;
            //发布里程计
            odom_pub.publish(odom);
//            ROS_INFO("location: %f, %f, %f", x, y, th);
//            ROS_INFO("velocity: %d, %d", vel_linear.d, vel_angular.d);
        

     last_time = ros::Time::now();
    }
	
	private:
	ros::NodeHandle n, private_nh;
	ros::Publisher odom_pub, motor_pub;
	ros::Subscriber odom_sub_0;

  float offset_imu_, offset_odom_;

	int threshold_current_;
	float x, y, vx, vth, th;
  ros::Time time_mark;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");//初始化串口节点

  if(can_conect(&s, "can0") < 0)
  {
    exit(0);
  }

//  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
//  strcpy(ifr.ifr_name, "can0");
//  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
//  addr.can_family = AF_CAN;
//  addr.can_ifindex = ifr.ifr_ifindex;
//  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  //定义接收规则，只接收表示符等于0x83的报文
//  rfilter[0].can_id = 0x181;//右轮
//  rfilter[0].can_mask = CAN_SFF_MASK;

//  rfilter[1].can_id = 0x182;//左轮
//  rfilter[1].can_mask = CAN_SFF_MASK;
	
//  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}


