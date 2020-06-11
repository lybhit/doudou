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
    private_nh_.param("debug", debug_, false);//消息上传频率

    private_nh_.param("motor_can_port", motor_can_port_, std::string("can0"));
    private_nh_.param("ultrasonic_can_port", ultrasonic_can_port_, std::string("can1"));

    if(motor_dev_.canConnect(motor_can_port_.c_str()))
    {
      std::cout << "can device connected!" << std::endl;
      

      motorInit(0x601, 0x301);
      motorInit(0x602, 0x302);

      usleep(1000);

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

    motorStop();

    motor_read_thread_->interrupt();
    motor_read_thread_->join();
    
    delete motor_read_thread_;
}



void BaseCommRos::sendEvent(const ros::TimerEvent& te)
{
  carbot_msgs::DifferentialDriveCommRev sensorMsg;
  // std::cout << "send msg!!!" << std::endl;
  // ROS_INFO_STREAM_THROTTLE(0.5, "send msg!!!");

  {
    writeLock lockWrite(rwMutex_);

    sensorMsg.header.stamp = ros::Time::now();
    
    sensorMsg.FromSensorData.resize(80);
    sensorMsg.FromSensorData[0] = 0xA5;
    sensorMsg.FromSensorData[1] = 0x5A;
    sensorMsg.FromSensorData[2] = 0x24;

    sensorMsg.FromSensorData[3] = counter_ ++;
    if(counter_ == 256)
    {
        counter_ = 0;
    }

    uint8_t car_type  = 0x02;
    uint8_t car_size  = 0x01;
    uint8_t soft_year = 0x00;
    uint8_t soft_mon  = 0x06;
    uint8_t soft_data = 0x08;

    sensorMsg.FromSensorData[4] = (car_type<<4)|(car_size); //序列号
    sensorMsg.FromSensorData[5] = 0x00; //reserved
    sensorMsg.FromSensorData[6] = (soft_year<<4)|soft_mon;
    sensorMsg.FromSensorData[7] = soft_data;

    sensorMsg.FromSensorData[8]  = l_motor_data_[0]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[9]  = l_motor_data_[1]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[10] = l_motor_data_[2]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[11] = l_motor_data_[3]; //左轮毂电机转速（rpm）

    sensorMsg.FromSensorData[12] = r_motor_data_[0]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[13] = r_motor_data_[1]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[14] = r_motor_data_[2]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[15] = r_motor_data_[3]; //右轮毂电机转速（rpm）

    sensorMsg.FromSensorData[16] = sensorMsg.FromSensorData[8]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[17] = sensorMsg.FromSensorData[9]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[18] = sensorMsg.FromSensorData[10]; //左轮毂电机转速（rpm）
    sensorMsg.FromSensorData[19] = sensorMsg.FromSensorData[11]; //左轮毂电机转速（rpm）

    sensorMsg.FromSensorData[20] = sensorMsg.FromSensorData[12]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[21] = sensorMsg.FromSensorData[13]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[22] = sensorMsg.FromSensorData[14]; //右轮毂电机转速（rpm）
    sensorMsg.FromSensorData[23] = sensorMsg.FromSensorData[15]; //右轮毂电机转速（rpm）

    sensorMsg.FromSensorData[24] = 0; //前左转向电机角度（rpm）
    sensorMsg.FromSensorData[25] = 0; //前左转向电机角度（rpm）

    sensorMsg.FromSensorData[26] = 0; //前右转向电机角度（rpm）
    sensorMsg.FromSensorData[27] = 0; //前右转向电机角度（rpm）

    sensorMsg.FromSensorData[28] = 0; //后左转向电机角度（rpm）
    sensorMsg.FromSensorData[29] = 0; //后左转向电机角度（rpm）

    sensorMsg.FromSensorData[30] = 0; //后右转向电机角度（rpm）
    sensorMsg.FromSensorData[31] = 0; //后右转向电机角度（rpm）

    sensorMsg.FromSensorData[32] = l_motor_stat_; //左轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[33] = r_motor_stat_; //右轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[34] = l_motor_stat_; //后左轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]
    sensorMsg.FromSensorData[35] = r_motor_stat_; //后右轮毂驱动器状态信息，bit0=1[使能]，bit1=1[报警]，bit2=1[故障]

    sensorMsg.FromSensorData[36] = l_motor_err_; //左轮毂驱动器报警码
    sensorMsg.FromSensorData[37] = r_motor_err_; //右轮毂驱动器报警码
    sensorMsg.FromSensorData[38] = l_motor_err_; //后左轮毂驱动器报警码
    sensorMsg.FromSensorData[39] = r_motor_err_; //后右轮毂驱动器报警码

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

    sensorMsg.FromSensorData[76] = l_motor_load_rate_[0]; //前左轮毂负载率（千分比）
    sensorMsg.FromSensorData[77] = l_motor_load_rate_[1];
    sensorMsg.FromSensorData[78] = r_motor_load_rate_[0]; //前右轮毂负载率
    sensorMsg.FromSensorData[79] = r_motor_load_rate_[1];
  }

  modbus_crc_.push_vector(sensorMsg.FromSensorData);

  if(counter_ % 10 == 0)
  {
    // ROS_INFO("crc16 data: %d, %d", sensorMsg.FromSensorData[80], sensorMsg.FromSensorData[81]);
  }
  
  msg_publisher_.publish(sensorMsg);

  double dur = (ros::Time::now() - latest_sub_time_).toSec();

  if(dur > 1.0)
  {
    // motorStop();
  }

}

void BaseCommRos::run()
{
  msg_send_timer_ = ros::Timer(node_.createTimer(ros::Duration(1./rate_),
                                                &BaseCommRos::sendEvent,
                                                this));
}

void BaseCommRos::sensorMsgCallback(const carbot_msgs::DifferentialDriveCommSend::ConstPtr msg)
{
  latest_sub_time_ = ros::Time::now();
  collision_bar_enable_         = (msg->ToSensorData[4] & 0x1)?1:0; //防撞条使能判断
  ultrasonic_enable_               = (msg->ToSensorData[4] & 0x2)?1:0; //超声波使能判断
  fall_sensor_enable_             = (msg->ToSensorData[4] & 0x4)?1:0; //防跌落使能判断
  emergency_stop_enable_ = (msg->ToSensorData[4] & 0x8)?1:0; //急停开关使能判断

  led_mode_ = msg->ToSensorData[4] & 0x70;

  motor_enable_ = msg->ToSensorData[5] & 0b00000011;

  // l_motor_vel_send_.d = ((short)msg->ToSensorData[6]) << 8 | (short)msg->ToSensorData[7];
  // r_motor_vel_send_.d = (((short)msg->ToSensorData[8]) << 8 | (short)msg->ToSensorData[9]);
  l_motor_vel_send_.d = -(short)((msg->ToSensorData[6]) << 8 | msg->ToSensorData[7]);
  r_motor_vel_send_.d = -(short)((msg->ToSensorData[8]) << 8 | msg->ToSensorData[9]);

  // std::cout << "l_motor_vel_send_ " << l_motor_vel_send_.d << std::endl;
  // std::cout << "r_motor_vel_send_ " << r_motor_vel_send_.d << std::endl;

  struct can_frame send_2_motor[2];
  send_2_motor[0].can_id = 0x401;
  send_2_motor[0].can_dlc = 4;
  send_2_motor[0].data[0] = l_motor_vel_send_.data[0];
  send_2_motor[0].data[1] = l_motor_vel_send_.data[1];
  send_2_motor[0].data[2] = l_motor_vel_send_.data[2];
  send_2_motor[0].data[3] = l_motor_vel_send_.data[3];

  // ROS_INFO("left can frame data: %x, %x, %x, %x", send_2_motor[0].data[0], send_2_motor[0].data[1], send_2_motor[0].data[2], send_2_motor[0].data[3]);

  send_2_motor[1].can_id = 0x402;
  send_2_motor[1].can_dlc = 4;
  send_2_motor[1].data[0] = r_motor_vel_send_.data[0];
  send_2_motor[1].data[1] = r_motor_vel_send_.data[1];
  send_2_motor[1].data[2] = r_motor_vel_send_.data[2];
  send_2_motor[1].data[3] = r_motor_vel_send_.data[3];

  // ROS_INFO("right can frame data: %x, %x, %x, %x", send_2_motor[1].data[0], send_2_motor[1].data[1], send_2_motor[1].data[2], send_2_motor[1].data[3]);

  if(debug_)
  {

  }else
  {
    motor_dev_.sendFrame(&send_2_motor[0]);
    motor_dev_.sendFrame(&send_2_motor[1]);
  }

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
         l_motor_data_[0] = reader.data[0];
         l_motor_data_[1] = reader.data[1];
         l_motor_data_[2] = reader.data[2];
         l_motor_data_[3] = reader.data[3];

         l_motor_load_rate_[0] = reader.data[5];
         l_motor_load_rate_[1] = reader.data[4];
        //  std::cout << std::hex << reader.can_id << ": " << static_cast<int>(l_motor_data_[3]) << ' ' << static_cast<int>(l_motor_data_[2]) << ' ' << static_cast<int>(l_motor_data_[1]) << ' ' << static_cast<int>(l_motor_data_[0]) << std::endl;
       }

       if(reader.can_id == 0x182)
       {
         readLock lockRead(rwMutex_);
         r_motor_data_[0] = reader.data[0]; 
         r_motor_data_[1] = reader.data[1];
         r_motor_data_[2] = reader.data[2];
         r_motor_data_[3] = reader.data[3]; // 右轮反向

         r_motor_load_rate_[0] = reader.data[5];
         r_motor_load_rate_[1] = reader.data[4];
        //  std::cout << std::hex << reader.can_id << ": " << r_motor_data_[3] << ' ' << r_motor_data_[2] << ' ' << r_motor_data_[1] << ' ' << r_motor_data_[0] << std::endl;
       }
     }

     usleep(1000);
   }

}

void BaseCommRos::motorInit(int16_t id_1, int16_t id_2)
{
    ROS_INFO("START SEND DATA");
    struct can_frame start_frame;
    start_frame.can_id = 0;
    start_frame.can_dlc = 2;
    start_frame.data[0] = 0x01;
    start_frame.data[1] = 0x00;
    motor_dev_.sendFrame(&start_frame);

    ROS_INFO("Send data");
    struct can_frame init_frame[2];

    init_frame[0].can_id  = id_1;
    init_frame[0].can_dlc = 8;

    init_frame[1].can_id  = id_2;
    init_frame[1].can_dlc = 8;

    uint8_t rpdo_size = motor_init_data_.rpdo_data_.size();

    for(int i = 0; i < rpdo_size; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            init_frame[0].data[j] = motor_init_data_.rpdo_data_[i][j];
        }
        // ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[0].data[0], init_frame[0].data[1], init_frame[0].data[2],
        //              init_frame[0].data[3], init_frame[0].data[4], init_frame[0].data[5], init_frame[0].data[6], init_frame[0].data[7]);
//        calculate(10);
       usleep(1000);

       motor_dev_.sendFrame(&init_frame[0]);
    }

    for(int i = 0; i < 8; ++i)
    {
         init_frame[1].data[i] = motor_init_data_.acc_data_[i];
    }
    // ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[1].data[0], init_frame[1].data[1], init_frame[1].data[2],
    //                  init_frame[1].data[3], init_frame[1].data[4], init_frame[1].data[5], init_frame[1].data[6], init_frame[1].data[7]);
     usleep(1000);
     motor_dev_.sendFrame(&init_frame[1]);

    uint8_t tpdo_size = motor_init_data_.tpdo_data_.size();
    for(int i = 0; i < tpdo_size; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            init_frame[0].data[j] = motor_init_data_.tpdo_data_[i][j];
        }
        // ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[0].data[0], init_frame[0].data[1], init_frame[0].data[2],
        //              init_frame[0].data[3], init_frame[0].data[4], init_frame[0].data[5], init_frame[0].data[6], init_frame[0].data[7]);
        usleep(1000);
        motor_dev_.sendFrame(&init_frame[0]);
    }

}

void BaseCommRos::motorStop()
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
      // ROS_INFO("send stop");
      motor_dev_.sendFrame(&stop_frame[0]);
      usleep(100);
      motor_dev_.sendFrame(&stop_frame[1]);

      loop.sleep();
  }
}


 

}//namespace doudou

