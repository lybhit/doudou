#ifndef CAN_HANDLE_H
#define CAN_HANDLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

// #include <ros/ros.h>

namespace doudou{

    // void sigterm(int signo)
    // {
        
    // }

class CANHandle
{
public:
    CANHandle():filter_can_id(0xb5 | 0xb6), filter_can_mask(0xDFFFF000){}

    ~CANHandle()
    {
        std::cout << "~CANHandle" << std::endl;
        disconnectSocket();
        // close(s);
    }

    void setFilterPara(const int filter_id, const int filter_mask)
    {
        filter_can_id = filter_id;
        filter_can_mask = filter_mask;
    }

    bool canConnect(const char *device)
    {
        struct sockaddr_can addr;
        struct can_filter *filter;

        int i, ret;
        int enable_canfd = 1;
        // signal(SIGTERM, sigterm);
        // signal(SIGHUP, sigterm);
        // signal(SIGINT, sigterm);

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(s < 0)
        {
            perror("socket");
            return 0;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = if_nametoindex(device);

        filter = new struct can_filter;
        filter->can_id = filter_can_id & CAN_SFF_MASK;
        filter->can_mask = filter_can_mask & (~CAN_ERR_FLAG);

        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, filter, sizeof(struct can_filter));
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

        delete filter;

        if(bind(s, (struct sockaddr *)&addr, sizeof (addr)) < 0)
        {
            perror("bind");
            return 0;
        }

        return 1;
    }

    int sendFrame(struct can_frame *frame)
    {
        int ret;
        while((ret = send(s, frame, sizeof (struct can_frame), 0)) != sizeof (can_frame))
        {
            if(ret < 0)
            {
                if(errno != ENOBUFS)
                {
                    perror("send failed");
                    return -1;
                }
            }
            else {
    //            fprintf(stderr, "send returned %d, ret");
            }
        }

        return 0;
    }

    int recvFrame(struct can_frame *frame)
    {
        struct timeval timeout;
        int ret;
        int val;
        fd_set rdfs;

        timeout.tv_sec = 0;
        timeout.tv_usec = 10;

        FD_ZERO(&rdfs);
        FD_SET(s, &rdfs);

        if((ret = select(s+1, &rdfs, NULL, NULL, NULL)) <= 0)
        {
            perror("select");
            return 0;
        }

        if(FD_ISSET(s, &rdfs))
        {
            ret = recv(s, frame, sizeof(*frame), 0);

            if(ret != sizeof(*frame))
            {
                if(ret < 0)
                {
                    perror("recv failed");
                }else {
                    fprintf(stderr, "recv returned %d", ret);
                }

                return 0;
            }else {
                return 1;
            }
        }


    //#if 1
    //    ROS_INFO("frame id: 0x%02x", frame->can_id);

    //    for(int i = 0; i < frame->can_dlc; ++i)
    //    {
    //       printf("recv:%2d ~ 0x%2x\n", i, frame->data[i]);
    //    }
    //#endif
        return 0;
    }

    void disconnectSocket()
    {
        std::cout << "close s" << std::endl;
        close(s);
    }

private:
    int s;
    int filter_can_id;
    int filter_can_mask;
//    struct can_frame frame;

};

}//namespace doudou

#endif // CAN_HANDLE_H
