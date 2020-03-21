#include "cruiser_tx2_can.h"
#include "ros/ros.h"
class CanManage
{
private:
    ros::NodeHandle node;
    int sockfd;
    bool debug;

    void  recv_manager(void);
    void  callbackThread(void);


    bool servo_recv_flag;
    bool hub_recv_flag;

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

public:
    CanManage();
    ~CanManage();
    void send_stop();
    void send_init(int16_t id_1, int16_t id_2);
};

CanManage::CanManage():
    flag_a(false),
    flag_b(false),
    flag_c(false)
{
    
    if(can_conect(&sockfd, tx2_can[0]) < 0)
    {
        ROS_ERROR("Open can0 failed!");
        exit(0);
    }

    // // boost::thread chatter_thread(&CanManage::callbackThread);
    // boost::thread chatter_thread(boost::bind(&CanManage::callbackThread, this));
    
    ros::Rate loop(50);
    while(running)
    {
        recv_manager();
        // if(running == 0 )
        // {
        //     break;
        // }
        ROS_INFO("FLAG : %d, %d, %d", flag_a, flag_b, flag_c);
        if(flag_a == true && flag_b == true && flag_c == true)
        {
            ROS_INFO("go to send data");
            send_init( 0x601, 0x301);
            send_init( 0x602, 0x302);

            flag_a = false;
            flag_b = false;
            flag_c = false;
        }
        ros::spinOnce();
        loop.sleep();
    }

//    chatter_thread.join();
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
        ROS_INFO("Data: %d, %d, %d, %d, %d, %d, %d, %d",init_frame[1].data[0], init_frame[1].data[1], init_frame[1].data[2],
                     init_frame[1].data[3], init_frame[1].data[4], init_frame[1].data[5], init_frame[1].data[6], init_frame[1].data[7]);
    }

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
    struct timeval timeout;
    int fd = sockfd;
    fd_set rdfs;
    int ret;

    timeout.tv_sec = 5;
    timeout.tv_usec = 0;


    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    if ((ret = select((fd)+1, &rdfs, NULL, NULL, &timeout)) <= 0) 
    {
        perror("select");
    //    running = 0;
        flag_c = true;
        ROS_INFO("Flag_c %d", flag_c);
        return;
    }
    
    if (FD_ISSET(fd, &rdfs)) 
    {
        ret = recv_frame(&frame, fd);

        if(frame.can_id == 0x701)
        {
            if(frame.data[0] == 0)
                flag_a = true;
            ROS_INFO("Flag_a %d", flag_a);
        }

        if(frame.can_id == 0x702)
        {
            if(frame.data[0] == 0)
                flag_b = true;
            ROS_INFO("Flag_b %d", flag_b);
        }

        ROS_INFO("ID:%x  data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", frame.can_id, frame.data[0], frame.data[1],
                frame.data[2],frame.data[3],frame.data[4],frame.data[5], frame.data[6], frame.data[7]);
        
        
    }
    fflush(stdout);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_manager_node");
    CanManage CanManage;
    
    ros::spin();
    ros::shutdown();
    return 0;
}
