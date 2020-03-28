#include "cruiser_tx2_can.h"
//#include "cruiser_bringup_config.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#define PI 3.141592

//通过can0读取数据统一处理
//1.订阅/imu/data,里程计数据合成，发布odom话题

//2.电机有效转矩率 %， 发布motor_status话题

//3.急停开关断开恢复，发送数据设置PDO，初始化

union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

class CanManage
{
private:
    ros::NodeHandle n;

    ros::Publisher motor_pub, odom_pub;

    ros::Subscriber imu_sub;

    float x, y, vx, vth, th;

    std_msgs::String str;


    ros::Time current_time, last_time, time_mark;
    int sockfd;
    bool debug;

    void  recv_manager(void);
    void  callbackThread(void);
    void  callback_imu(const sensor_msgs::Imu & cmd_input);


    bool flag_a, flag_b, flag_c;

    int frame_data_1st[11][8] = {{0x2F,0x00,0x16,0x00,0x00,0x00,0x00,0x00},
                                 {0x23,0x00,0x16,0x01,0x10,0x00,0x40,0x60},
                                 {0x2F,0x00,0x16,0x00,0x01,0x00,0x00,0x00},
                                 {0x2F,0x01,0x16,0x00,0x00,0x00,0x00,0x00},
                                 {0x23,0x01,0x16,0x01,0x20,0x00,0x83,0x60},
                                 {0x23,0x01,0x16,0x02,0x20,0x00,0x84,0x60},
                                 {0x2F,0x01,0x16,0x00,0x02,0x00,0x00,0x00},
                                 {0x2F,0x02,0x16,0x00,0x00,0x00,0x00,0x00},
                                 {0x23,0x02,0x16,0x01,0x20,0x00,0xFF,0x60},
                                 {0x2F,0x02,0x16,0x00,0x01,0x00,0x00,0x00},
                                 {0x2F,0x60,0x60,0x00,0x03,0x00,0x00,0x00}};

    int frame_data_2nd[8] = {0xb8,0x0b,0x00,0x00,0xb8,0x0b,0x00,0x00};

    int frame_data_3rd[10][8] = {{0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00},
                                 {0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00},
                                 {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00},
                                 {0x2F,0x00,0x1a,0x00,0x00,0x00,0x00,0x00},
                                 {0x23,0x00,0x1a,0x01,0x10,0x00,0x6C,0x60},
                                 {0x23,0x00,0x1a,0x02,0x10,0x00,0x02,0x50},
                                 {0x2F,0x00,0x1a,0x00,0x02,0x00,0x00,0x00},
                                 {0x2F,0x00,0x18,0x05,0x32,0x00,0x00,0x00},
                                 {0x2F,0x00,0x18,0x03,0x32,0x00,0x00,0x00},
                                 {0x2F,0x00,0x18,0x02,0xfe,0x00,0x00,0x00}
                                };

    int right_rot_vel, left_rot_vel;
    int16_t val_1, val_2;
    uint16_t threshold;

public:
    CanManage();
    ~CanManage();
    void send_stop();
    void send_init(int16_t id_1, int16_t id_2);
};

CanManage::CanManage():
    flag_a(false),
    flag_b(false),
    flag_c(false),
    threshold(400),
    x(0),y(0),vx(0),vth(0),th(0)
{
    
    if(can_conect(&sockfd, tx2_can[0]) < 0)
    {
        exit(0);
    }
    
    motor_pub = n.advertise<std_msgs::String>("motor_status", 10);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);

    imu_sub = n.subscribe("/imu/data", 3, &CanManage::callback_imu, this);

    current_time = ros::Time::now();
    last_time = ros::Time::now(); 


    // boost::thread chatter_thread(&CanManage::callbackThread);
    //boost::thread chatter_thread(boost::bind(&CanManage::callbackThread, this));
    
    ros::Rate loop(20);
    while(ros::ok())
    {
      recv_manager();
      // if(running == 0 )
      // {
      //     break;
      // }

      
      if(flag_a == true && flag_b == true)
      {
          ROS_INFO("FLAG : %d, %d, %d", flag_a, flag_b, flag_c);
          ROS_INFO("go to send data");
          send_init( 0x601, 0x301);
          send_init( 0x602, 0x302);

          flag_a = false;
          flag_b = false;
          flag_c = false;
      }

      if(val_1 >= threshold || val_2 >= threshold)
      {

      }
      else
      {
    //  clear_flag = true;
        time_mark = ros::Time::now();
      }

      str.data = "ok";

      if((ros::Time::now() - time_mark).toSec() >= 3.0)
      {
        str.data = "alarm";
      }

      motor_pub.publish(str);


      ros::spinOnce();
      loop.sleep();
    }

}

CanManage::~CanManage()
{
    send_stop();
}

void CanManage::send_init(int16_t id_1, int16_t id_2)
{
    ROS_INFO("START SEND DATA");
    struct can_frame start_frame;
    start_frame.can_id = 0;
    start_frame.can_dlc = 2;
    start_frame.data[0] = 0x01;
    start_frame.data[1] = 0x00;
    send_frame(&start_frame, sockfd);
    ROS_INFO("send data: %d, %d", start_frame.data[0], start_frame.data[1]);

    ROS_INFO("Send data");
    struct can_frame init_frame[2];

    init_frame[0].can_id  = id_1;
    init_frame[0].can_dlc = 8;

    init_frame[1].can_id  = id_2;
    init_frame[1].can_dlc = 8;

    for(int i = 0; i < 11; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            init_frame[0].data[j] = frame_data_1st[i][j];
        }
        ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[0].data[0], init_frame[0].data[1], init_frame[0].data[2],
                     init_frame[0].data[3], init_frame[0].data[4], init_frame[0].data[5], init_frame[0].data[6], init_frame[0].data[7]);

        send_frame(&init_frame[0], sockfd);
    }

    for(int i = 0; i < 8; ++i)
    {
        init_frame[1].data[i] = frame_data_2nd[i];
        
    }
    ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[1].data[0], init_frame[1].data[1], init_frame[1].data[2],
                     init_frame[1].data[3], init_frame[1].data[4], init_frame[1].data[5], init_frame[1].data[6], init_frame[1].data[7]);

    send_frame(&init_frame[1], sockfd);

    for(int i = 0; i < 10; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            init_frame[0].data[j] = frame_data_3rd[i][j];
        }
        ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[0].data[0], init_frame[0].data[1], init_frame[0].data[2],
                     init_frame[0].data[3], init_frame[0].data[4], init_frame[0].data[5], init_frame[0].data[6], init_frame[0].data[7]);

        send_frame(&init_frame[0], sockfd);
    }

}

void CanManage::send_stop()
{
    struct can_frame stop_frame[2];

    stop_frame[0].can_id  = 0x401;
    stop_frame[0].can_dlc = 4;

    stop_frame[1].can_id  = 0x402;
    stop_frame[1].can_dlc = 4;
    
    for(int i=0;i<stop_frame[0].can_dlc;i++)
    {
        stop_frame[0].data[i] = 0x00;
    }

    for(int i=0;i<stop_frame[1].can_dlc;i++)
    {
        stop_frame[1].data[i] = 0x00;
    }

    ros::Rate loop(100);
    for(int i=0;i<10;i++)
    {
        ROS_INFO("send stop");
        send_frame(&stop_frame[0], sockfd);
        send_frame(&stop_frame[1], sockfd);

        loop.sleep();
    }
}

void CanManage::recv_manager(void)
{
    struct can_frame frame;
    int fd = sockfd;
    fd_set rdfs;
    int ret;
    struct timeval timeout;

    timeout.tv_sec = 5;
    timeout.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    if ((ret = select((fd)+1, &rdfs, NULL, NULL, NULL)) <= 0) 
    {
    //flag_c = true;
     perror("select");
    
//     flag_c = true;
     ROS_INFO("Flag_c %d", flag_c);
     return;
    }
    
    if (FD_ISSET(fd, &rdfs)) 
    {
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

        if(frame.can_id == 0x701)
        {
            if(frame.data[0] == 0)
            {
             flag_a = true;
            }
            ROS_INFO("Flag_a %d", flag_a);
        }

        if(frame.can_id == 0x702)
        {
            if(frame.data[0] == 0)
            {
              flag_b = true;
            }
                
            ROS_INFO("Flag_b %d", flag_b);
        }

        // ROS_INFO("ID:%x  data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", frame.can_id, frame.data[0], frame.data[1],
        //         frame.data[2],frame.data[3],frame.data[4],frame.data[5], frame.data[6], frame.data[7]);
        
        
    }
    fflush(stdout);
}

void CanManage::callback_imu(const sensor_msgs::Imu & cmd_input)
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

  double r = 0.065;
  double L = 0.3; 
         
  if(right_rot_vel == 0 || right_rot_vel == -1)
     right_rot_vel = 0;

  if(left_rot_vel == 0 || left_rot_vel == -1)
     left_rot_vel = 0;

//   ROS_INFO("Right_rot_vel: = %d", right_rot_vel);
//   ROS_INFO("Left_rot_vel: = %d", left_rot_vel);
 
  double left_linear = left_rot_vel * 2 * PI * r / 60;
  double right_linear = -right_rot_vel * 2 * PI * r / 60;//右轮速度取反，沿着出线的方向看过去，顺时针速度为正

  double v_linear =  (left_linear + right_linear) * 0.5;
  double v_angular =  (right_linear - left_linear)/ L;

  current_time = ros::Time::now();

  double dt = (current_time - last_time).toSec();

  double delta_x = v_linear * cos(th) * dt;
  double delta_y = v_linear * sin(th) * dt;
  double delta_th = v_angular * dt;


  x += delta_x;
  y += delta_y;

  th += delta_th;
      
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
  //odom.twist.twist.angular.z = vel_angular.d * 0.001;
  // odom.twist.twist.angular.z = vth;
  odom.twist.twist.angular.z = v_angular;
  //发布里程计
  odom_pub.publish(odom);
//      ROS_INFO("location: %f, %f, %f", x, y, th);
//      ROS_INFO("velocity: %d, %d", vel_linear.d, vel_angular.d);

last_time = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_manager_node");
    CanManage CanManage;
    
    ros::spin();
    ros::shutdown();
    return 0;
}
