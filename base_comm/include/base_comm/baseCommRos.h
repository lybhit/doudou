#ifndef BASE_COMM_H
#define BASE_COMM_H

#include "ros/ros.h"
#include "canHandle.h"
#include "motor_can_data.h"
#include "motor_crc16.h"
#include "carbot_msgs/DifferentialDriveCommRev.h"
#include "carbot_msgs/DifferentialDriveCommSend.h"

#include <vector>
#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <signal.h>

namespace doudou {

int running_; // 用于退出线程

typedef boost::shared_mutex            WR_Mutex;
typedef boost::unique_lock<WR_Mutex>   writeLock;
typedef boost::shared_lock<WR_Mutex>   readLock;

class BaseCommRos
{
public:
    BaseCommRos();
    ~BaseCommRos();

    void sensorMsgCallback(const carbot_msgs::DifferentialDriveCommSend::ConstPtr msg);

    void sendEvent(const ros::TimerEvent& te);

    void run();

    void motorReadThread();

    void motorInit(int16_t id_1, int16_t id_2); // Set RPDO, TPDO, enable motor.

    void motorStop();

private:
    ros::NodeHandle node_;

    ros::Subscriber msg_subscriber_;

    ros::Publisher msg_publisher_;

    ros::Timer msg_send_timer_;//设置定时发送消息
    int rate_;

    // uint8_t l_motor_data_[4];
    // uint8_t r_motor_data_[4];
    uint8_t l_motor_stat_;
    uint8_t r_motor_stat_;

    uint8_t l_motor_err_;
    uint8_t r_motor_err_;

    union int_char
    {
      int d;
      unsigned char data[4];
    }l_motor_vel_send_, r_motor_vel_send_, l_motor_data_, r_motor_data_;

    uint8_t battery_volt_[2];
    uint8_t battery_current_[2];
    uint8_t battery_temp_;

    uint8_t ultrasonic_data_[8];

    uint8_t obs_avoid_;

    uint8_t fall_sensor_data_[4];

    uint8_t l_motor_load_rate_[2];
    uint8_t r_motor_load_rate_[2];

    uint8_t comm_stat_[2];

    uint8_t crc_[2];

    int counter_; //0~255之间变换
    doudou::CANHandle motor_dev_; //驱动器
    // doudou::CANHandle ultrasonic_dev_; //超声波

    std::string motor_can_port_;
    std::string ultrasonic_can_port_;

    std::string sensor_2_tx2_topic_;
    std::string tx2_2_sensor_topic_;

    WR_Mutex rwMutex_;

    boost::thread* motor_read_thread_;
    boost::condition_variable_any read_cond_;

    bool motor_connect_stat_;
    bool collision_bar_enable_;
    bool ultrasonic_enable_;
    bool fall_sensor_enable_;
    bool emergency_stop_enable_;

    uint8_t led_mode_;
    uint8_t motor_enable_;

    MotorCanData motor_init_data_;

    doudou::ModbusCrc16 modbus_crc_;

    ros::Time latest_sub_time_;

    bool debug_;

};

}//namespace doudou

#endif // BASE_COMM_H
