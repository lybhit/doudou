#include "ros/ros.h"
#include "carbot_msgs/DifferentialDriveCommRev.h"
#include "carbot_msgs/DifferentialDriveCommSend.h"

class TestComm
{
    public:
    TestComm()
    : counter_(0)
    , private_nh_("~")
    {
        private_nh_.param("rate", rate_, 10);

        msg_publisher_ = nh_.advertise<carbot_msgs::DifferentialDriveCommSend>("msg_send", 2);

    }

    ~TestComm(){}

    void update(const ros::TimerEvent& te)
    {
        carbot_msgs::DifferentialDriveCommSend msg;

        msg.ToSensorData.resize(28);
        msg.ToSensorData[0] = 0x66;
        msg.ToSensorData[1] = 0xAA;
        msg.ToSensorData[2] = 0x19;
        msg.ToSensorData[3] = counter_++;

        if(counter_ == 256){
           counter_ = 0;
        }

        msg.ToSensorData[4] = 0b10100000;
        msg.ToSensorData[5] = 0b00000011;
        msg.ToSensorData[6] = 0x00;  // 左轮毂电机转速 高8位 （0.1rpm）
        msg.ToSensorData[7] = 0x64;  // 左轮毂电机转速 低8位 （0.1rpm）
        msg.ToSensorData[8] = 0x00;  // 右轮毂电机转速 高8位 （0.1rpm）
        msg.ToSensorData[9] = 0x32;  // 右轮毂电机转速 低8位 （0.1rpm）

        msg.ToSensorData[10] = 0x19;
        msg.ToSensorData[11] = 0x19;
        msg.ToSensorData[12] = 0x19;
        msg.ToSensorData[13] = 0x19;
        msg.ToSensorData[14] = 0x19;
        msg.ToSensorData[15] = 0x19;
        msg.ToSensorData[16] = 0x19;
        msg.ToSensorData[17] = 0x19;
        msg.ToSensorData[18] = 0x19;
        msg.ToSensorData[19] = 0x19;
        msg.ToSensorData[20] = 0x19;
        msg.ToSensorData[21] = 0x19;
        msg.ToSensorData[22] = 0x19;
        msg.ToSensorData[23] = 0x19;
        msg.ToSensorData[24] = 0x19;
        msg.ToSensorData[25] = 0x19;
        msg.ToSensorData[26] = 0x19;
        msg.ToSensorData[27] = 0x19;

        msg_publisher_.publish(msg);
    }

    void run()
    {
        update_timer_ = ros::Timer(nh_.createTimer(ros::Duration(1./rate_), &TestComm::update, this));
    }

    private:
    
    // Access to ROS node
    ros::NodeHandle nh_, private_nh_;
    ros::Timer update_timer_;
    ros::Publisher msg_publisher_;

    // Parameters
    int rate_;
    int counter_;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_comm_node");

    TestComm test_comm;
    test_comm.run();

    ros::spin();

    return 0;
}