#ifndef CRUISER_TX2_CAN_H
#define CRUISER_TX2_CAN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <pthread.h>
#include <semaphore.h>

#ifndef SO_TIMESTAMPING
#define SO_TIMESTAMPING 37
#endif

#define TX2_CAN_EQUIPMENTS (2)
#define TX2_CAN_ENABLE     (1)

static volatile int running = 1;

const char tx2_can[TX2_CAN_EQUIPMENTS][5] = {"can0", "can1"};

const int filter_can_id   = 0xB5|0xB6;
const int filter_can_mask = 0xDFFFF000;//0xDFFFFF0E;
const int canfd_on = 1;

typedef struct can_pthread_frame_
{
    int sockfd;
    struct can_frame frame;
}can_pthread_frame;

// const int    hub_max_speed[4]   = {281, 278, 281, 279};
// const double linear_ratio[4][2] = {{1.09179, -0.092745}, {1.083820, -0.183026}, {1.181523, -0.037415}, {1.165698, -0.017485}};
// const double recv_linear_ration[4][2] = {{0.918947, 0.035581}, {0.915549, 0.019863}, {0.9198, -0.094039}, {0.916452, -0.039987}};

void sigterm(int signo);
int send_frame(struct can_frame *frame, int sockfd);
int recv_frame(struct can_frame *frame, int sockfd);
int can_conect(int *sockfd, const char *device);

double hub_round_trans(const int data, const int num);
double servo_trans(const int data);

#endif
