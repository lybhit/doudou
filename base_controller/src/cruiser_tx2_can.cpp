#include "cruiser_tx2_can.h"
#include "ros/ros.h"
//#include "cruiser_bringup_config.h"

void sigterm(int signo)
{
	running = 0;
}

int recv_frame(struct can_frame *frame, int sockfd)
{
	struct timeval timeout;
	int ret;
	int val;
	ROS_INFO("recv_frame");

	ret = recv(sockfd, frame, sizeof(*frame), 0);

	if (ret != sizeof(*frame)) 
	{
		if (ret < 0)
			perror("recv failed");
		else
			fprintf(stderr, "recv returned %d", ret);
		return -1;
	}
#if 1
	ROS_INFO("frame id:0x%02x", frame->can_id);

	for(int i=0;i<frame->can_dlc;i++)
	{
		printf("recv:%02d ~ 0x%02x\n", i, frame->data[i]);
	}
#endif
	return 0;
}

int send_frame(struct can_frame *frame, int sockfd)
{
    int ret;
	while ((ret = send(sockfd, frame, sizeof(struct can_frame), 0)) != sizeof(struct can_frame)) 
	{
		if (ret < 0) 
		{
			if (errno != ENOBUFS) {
				perror("send failed");
				return -1;
			} 
		} 
		else 
		{
			fprintf(stderr, "send returned %d", ret);
			return -1;
		}
	}
	return 0;
}

int can_conect(int *sockfd, const char *device)
{
    struct sockaddr_can addr;
    struct can_filter *filter;


    int i, ret;

    signal(SIGTERM, sigterm);
    signal(SIGHUP,  sigterm);
    signal(SIGINT,  sigterm);

    *sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if(*sockfd < 0)
    {
        perror("socket");
        return -1;
    }

    addr.can_family       = AF_CAN;
    addr.can_ifindex      = if_nametoindex(device);

    filter = new struct can_filter;

    filter->can_id   = filter_can_id & CAN_SFF_MASK;
    filter->can_mask = filter_can_mask & (~CAN_ERR_FLAG);

    ROS_INFO("can  mask:0x%08X can_id:0x%08X", filter->can_mask, filter->can_id);

    setsockopt(*sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, filter, sizeof(struct can_filter));
    setsockopt(*sockfd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

    delete [] filter;

    if (bind(*sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
    {
        perror("bind");
        return -1;
    }
    return 0;
}

double hub_round_trans(const int data, const int num)
{
	int value = data;
	double ret;

	double wheel_diameter;
	ros::param::get("wheel_diameter", wheel_diameter);

	if((data>>15)&0x01)
	{
		value = 0xFFFF0000 | value;
	}
	return ret;
}

double servo_trans(const int data)
{
	int value = data;
	double ret;
	if((data>>15)&0x01)
	{
		value = 0xFFFF0000 | value;
	}
	value = value;
	return ret;
}

