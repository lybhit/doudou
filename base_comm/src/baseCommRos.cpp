#include "base_comm/baseCommRos.h"

namespace doudou {

void sigterm(int signo)
{
     running_ = 0; 
}

BaseCommRos::BaseCommRos()
: motor_connect_stat_(false)
{
    ros::NodeHandle private_nh_("~");
    private_nh_.param("sensor_2_tx2_topic", sensor_2_tx2_topic_, std::string("sensorMsgToTx2"));
    private_nh_.param("tx2_2_sensor_topic", tx2_2_sensor_topic_, std::string("tx2MsgToSensor"));

    private_nh_.param("rate", rate_, 20);//消息上传频率

    private_nh_.param("motor_can_port", motor_can_port_, std::string("can0"));
    private_nh_.param("ultrasonic_can_port", ultrasonic_can_port_, std::string("can1"));

    if(motor_dev_.canConnect(motor_can_port_.c_str()))
    {
      std::cout << "can device connected!" << std::endl;
      motor_connect_stat_ = true;
      running_ = 1;
    }

    motor_read_thread_ = new boost::thread(boost::bind(&BaseCommRos::motorReadThread, this));

    msg_publisher_ = node_.advertise<carbot_msgs::DifferentialDriveCommRev>(sensor_2_tx2_topic_, 10);

    msg_subscriber_ = node_.subscribe(tx2_2_sensor_topic_, 3, &BaseCommRos::sensorMsgCallback, this);


}

BaseCommRos::~BaseCommRos()
{
    std::cout << "~BaseCommRos" << std::endl; 

    motor_read_thread_->interrupt();
    motor_read_thread_->join();
    
    delete motor_read_thread_;
}



void BaseCommRos::sendEvent(const ros::TimerEvent& te)
{
  carbot_msgs::DifferentialDriveCommRev sensorMsg;
  std::cout << "send msg!!!" << std::endl;
  {
    writeLock lockWrite(rwMutex_);
    
    sensorMsg.FromSensorData.resize(82);
    sensorMsg.FromSensorData[0] = 0xA5;
    sensorMsg.FromSensorData[1] = 0x5A;
    sensorMsg.FromSensorData[2] = 0x24;

    sensorMsg.FromSensorData[3] = counter_ ++;
    if(counter_ == 256){
        counter_ = 0;
    }

    sensorMsg.FromSensorData[4] = 0x0; //序列号
    sensorMsg.FromSensorData[5] = 0x0; //reserved
    sensorMsg.FromSensorData[6] = 0x0; //reserved
    sensorMsg.FromSensorData[7] = 0x0; //reserved

    sensorMsg.FromSensorData[8]  = l_motor_data_[0]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[9]  = l_motor_data_[1]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[10] = l_motor_data_[2]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[11] = l_motor_data_[3]; //左轮毂电机转速（rpm）

    sensorMsg.FromSensorData[12] = r_motor_data_[0]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[13] = r_motor_data_[1]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[14] = r_motor_data_[2]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[15] = r_motor_data_[3]; //右轮毂电机转速（rpm）

    sensorMsg.FromSensorData[16] = 0; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[17] = 0; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[18] = 0; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[19] = 0; //左轮毂电机转速（rpm）

    sensorMsg.FromSensorData[20] = 0; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[21] = 0; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[22] = 0; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[23] = 0; //右轮毂电机转速（rpm）

    sensorMsg.FromSensorData[24] = 0; //前左转向电机转速（rpm）
    sensorMsg.FromSensorData[25] = 0; //前左转向电机转速（rpm）

    sensorMsg.FromSensorData[26] = 0; //前右转向电机转速（rpm）
    sensorMsg.FromSensorData[27] = 0; //前右转向电机转速（rpm）

    sensorMsg.FromSensorData[28] = 0; //后左转向电机转速（rpm）
    sensorMsg.FromSensorData[29] = 0; //后左转向电机转速（rpm）

    sensorMsg.FromSensorData[30] = 0; //后右转向电机转速（rpm）
    sensorMsg.FromSensorData[31] = 0; //后右转向电机转速（rpm）

    sensorMsg.FromSensorData[32] = l_motor_stat_; //左轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[33] = r_motor_stat_; //右轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[34] = 0; //后左轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[35] = 0; //后右轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]

    sensorMsg.FromSensorData[36] = l_motor_err_; //左轮毂驱动器报警码
    sensorMsg.FromSensorData[37] = r_motor_err_; //右轮毂驱动器报警码
    sensorMsg.FromSensorData[38] = 0; //后左轮毂驱动器报警码
    sensorMsg.FromSensorData[39] = 0; //后右轮毂驱动器报警码

    sensorMsg.FromSensorData[40] = 0; //前左转向电机状态
    sensorMsg.FromSensorData[41] = 0; //前右转向电机状态
    sensorMsg.FromSensorData[42] = 0; //后左转向电机状态
    sensorMsg.FromSensorData[43] = 0; //后右转向电机状态

    sensorMsg.FromSensorData[44] = 0; //左轮毂驱动器报警码
    sensorMsg.FromSensorData[45] = 0; //右轮毂驱动器报警码
    sensorMsg.FromSensorData[46] = 0; //后左轮毂驱动器报警码
    sensorMsg.FromSensorData[47] = 0; //后右轮毂驱动器报警码

    sensorMsg.FromSensorData[48] = 20; //机箱温度（-127 ～ 127）
    sensorMsg.FromSensorData[49] = 22; //电池电量（0 ~ 100)

    sensorMsg.FromSensorData[50] = 20; //reserved
    sensorMsg.FromSensorData[51] = 22; //reserved

    sensorMsg.FromSensorData[52] = battery_volt_[0]; //电池电压（单位：100mV）
    sensorMsg.FromSensorData[53] = battery_volt_[1]; //电池电压（单位：100mV）

    sensorMsg.FromSensorData[54] = battery_current_[0]; //电池电流（单位：10mA）
    sensorMsg.FromSensorData[55] = battery_current_[1]; //电池电流（单位：10mA）

    sensorMsg.FromSensorData[56] = battery_temp_; //电池温度，1-报警，0-正常

    sensorMsg.FromSensorData[57] = ultrasonic_data_[0]; //超声波测距值(cm)
    sensorMsg.FromSensorData[58] = ultrasonic_data_[1]; //超声波测距值(cm)
    sensorMsg.FromSensorData[59] = ultrasonic_data_[2]; //超声波测距值(cm)
    sensorMsg.FromSensorData[60] = ultrasonic_data_[3]; //超声波测距值(cm)
    sensorMsg.FromSensorData[61] = ultrasonic_data_[4]; //超声波测距值(cm)
    sensorMsg.FromSensorData[62] = ultrasonic_data_[5]; //超声波测距值(cm)
    sensorMsg.FromSensorData[63] = ultrasonic_data_[6]; //超声波测距值(cm)
    sensorMsg.FromSensorData[64] = ultrasonic_data_[7]; //超声波测距值(cm)

    sensorMsg.FromSensorData[65] = obs_avoid_; //避障模块状态信息

    sensorMsg.FromSensorData[66] = comm_stat_[0]; //通信状态
    sensorMsg.FromSensorData[67] = comm_stat_[1]; //通信状态

    sensorMsg.FromSensorData[68] = fall_sensor_data_[0]; // 跌落传感器
    sensorMsg.FromSensorData[69] = fall_sensor_data_[1]; // 跌落传感器
    sensorMsg.FromSensorData[70] = fall_sensor_data_[2]; // 跌落传感器
    sensorMsg.FromSensorData[71] = fall_sensor_data_[3]; // 跌落传感器

    sensorMsg.FromSensorData[72] = l_motor_load_rate_[0]; //前左轮毂负载率（千分比）
    sensorMsg.FromSensorData[73] = l_motor_load_rate_[1];
    sensorMsg.FromSensorData[74] = r_motor_load_rate_[0]; //前右轮毂负载率
    sensorMsg.FromSensorData[75] = r_motor_load_rate_[1];

    sensorMsg.FromSensorData[76] = 0; //后左轮毂负载率
    sensorMsg.FromSensorData[77] = 0;
    sensorMsg.FromSensorData[78] = 0; //后右轮毂负载率
    sensorMsg.FromSensorData[79] = 0;
  }

  sensorMsg.FromSensorData[80] = crc_[0]; //crc校验
  sensorMsg.FromSensorData[81] = crc_[1]; //crc校验

  msg_publisher_.publish(sensorMsg);

}

void BaseCommRos::run()
{
  msg_send_timer_ = ros::Timer(node_.createTimer(ros::Duration(1./rate_),
                                                &BaseCommRos::sendEvent,
                                                this));
}

void BaseCommRos::sensorMsgCallback(const carbot_msgs::DifferentialDriveCommSend::ConstPtr msg)
{
  collision_bar_enable_         = (msg->ToSensorData[4] & 0x1)?1:0; //防撞条使能判断
  ultrasonic_enable_               = (msg->ToSensorData[4] & 0x2)?1:0; //超声波使能判断
  fall_sensor_enable_             = (msg->ToSensorData[4] & 0x4)?1:0; //防跌落使能判断
  emergency_stop_enable_ = (msg->ToSensorData[4] & 0x8)?1:0; //急停开关使能判断

  led_mode_ = msg->ToSensorData[4] & 0x70;

  motor_enable_ = msg->ToSensorData[5] & 0b00000011;

  l_motor_vel_send_.d = ((short)msg->ToSensorData[6]) << 8 | (short)msg->ToSensorData[7];
  r_motor_vel_send_.d = ((short)msg->ToSensorData[8]) << 8 | (short)msg->ToSensorData[9];

  std::cout << "l_motor_vel_send_ " << l_motor_vel_send_.d << std::endl;
  std::cout << "r_motor_vel_send_ " << r_motor_vel_send_.d << std::endl;

  struct can_frame send_2_motor[2];
  send_2_motor[0].can_id = 0x401;
  send_2_motor[0].can_dlc = 4;
  send_2_motor[0].data[0] = l_motor_vel_send_.data[0];
  send_2_motor[0].data[1] = l_motor_vel_send_.data[1];
  send_2_motor[0].data[2] = l_motor_vel_send_.data[2];
  send_2_motor[0].data[3] = l_motor_vel_send_.data[3];

  ROS_INFO("left can frame data: %d, %d, %d, %d", send_2_motor[0].data[0], send_2_motor[0].data[1], send_2_motor[0].data[2], send_2_motor[0].data[3]);

  send_2_motor[1].can_id = 0x402;
  send_2_motor[1].can_dlc = 4;
  send_2_motor[1].data[0] = r_motor_vel_send_.data[0];
  send_2_motor[1].data[1] = r_motor_vel_send_.data[1];
  send_2_motor[1].data[2] = r_motor_vel_send_.data[2];
  send_2_motor[1].data[3] = r_motor_vel_send_.data[3];

  ROS_INFO("left can frame data: %d, %d, %d, %d", send_2_motor[1].data[0], send_2_motor[1].data[1], send_2_motor[1].data[2], send_2_motor[1].data[3]);


  // motor_dev_.sendFrame(&send_2_motor[0]);
  // motor_dev_.sendFrame(&send_2_motor[1]);

}

void BaseCommRos::motorReadThread()
{
  ros::NodeHandle n;
  std::cout << "enter thead" << std::endl;
   struct can_frame reader;

  signal(SIGTERM, sigterm);
  signal(SIGHUP, sigterm);
  signal(SIGINT, sigterm);

   while(n.ok() && running_)
   {
    //  std::cout << "go to receive data process" << std::endl;
     if(motor_dev_.recvFrame(&reader))
     {
      //  std::cout << "receive data." << std::endl;
       if(reader.can_id == 0x181)
       {
         readLock lockRead(rwMutex_);
         l_motor_data_[0] = reader.data[3];
         l_motor_data_[1] = reader.data[2];
         l_motor_data_[2] = reader.data[1];
         l_motor_data_[3] = reader.data[0];
        //  std::cout << std::hex << reader.can_id << ": " << static_cast<int>(l_motor_data_[3]) << ' ' << static_cast<int>(l_motor_data_[2]) << ' ' << static_cast<int>(l_motor_data_[1]) << ' ' << static_cast<int>(l_motor_data_[0]) << std::endl;
       }

       if(reader.can_id == 0x182)
       {
         readLock lockRead(rwMutex_);
         r_motor_data_[0] = reader.data[3];
         r_motor_data_[1] = reader.data[2];
         r_motor_data_[2] = reader.data[1];
         r_motor_data_[3] = reader.data[0];
        //  std::cout << std::hex << reader.can_id << ": " << r_motor_data_[3] << ' ' << r_motor_data_[2] << ' ' << r_motor_data_[1] << ' ' << r_motor_data_[0] << std::endl;
       }
     }

     usleep(20000);
   }

}

}//namespace doudou

