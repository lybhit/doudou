#ifndef MOTOR_HANDLE_H
#define MOTOR_HANDLE_H

#include "canHandle.h"
#include "motor_can_data.h"
#include "ros/ros.h"
#include <string>

namespace doudou{

class MotorHandle : public CANHandle
{
public:
    MotorHandle(std::string can_port):can_port_(can_port), can_connect_stat_(false)
    {
        const char* port = can_port_.c_str();
        if(canConnect(port))
        {
            can_connect_stat_ = true;
        }
    }

void init(int16_t id_1, int16_t id_2)
{
    ROS_INFO("START SEND DATA");
    struct can_frame start_frame;
    start_frame.can_id = 0;
    start_frame.can_dlc = 2;
    start_frame.data[0] = 0x01;
    start_frame.data[1] = 0x00;
    sendFrame(&start_frame);

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

       sendFrame(&init_frame[0]);
    }

    for(int i = 0; i < 8; ++i)
    {
         init_frame[1].data[i] = motor_init_data_.acc_data_[i];
    }
    // ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[1].data[0], init_frame[1].data[1], init_frame[1].data[2],
    //                  init_frame[1].data[3], init_frame[1].data[4], init_frame[1].data[5], init_frame[1].data[6], init_frame[1].data[7]);
     usleep(1000);
    sendFrame(&init_frame[1]);

    uint8_t tpdo_size = motor_init_data_.tpdo_data_.size();
    for(int i = 0; i < 10; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            init_frame[0].data[j] = motor_init_data_.tpdo_data_[i][j];
        }
        // ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[0].data[0], init_frame[0].data[1], init_frame[0].data[2],
        //              init_frame[0].data[3], init_frame[0].data[4], init_frame[0].data[5], init_frame[0].data[6], init_frame[0].data[7]);
        usleep(1000);
        sendFrame(&init_frame[0]);
    }
}


private:
    std::string  can_port_;
    bool can_connect_stat_;
    MotorCanData motor_init_data_;

};
}


#endif