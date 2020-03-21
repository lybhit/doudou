#include "ros/ros.h"
#include <iostream>
#include <fstream>  
#include "std_msgs/Float32.h" 

#include <sensor_msgs/LaserScan.h>
#define PI 3.1415926


class LaserRead
{
public:
    LaserRead();
private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    //double threshold_;

};

LaserRead::LaserRead():private_nh_("~")
{
  //private_nh_.param("threshold", threshold_, 0.4);  
  sub_ = nh_.subscribe("/scan", 10, &LaserRead::laserCallback, this);

  pub_ = nh_.advertise<std_msgs::Float32>("min_dis", 5);
}


//std::ofstream myfile("/home/mm/record/laser.txt",std::ios::out);
void LaserRead::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> ranges=msg->ranges;
    float a = 0.7;
    for(int i=0;i<ranges.size();i++)
    {
        if(ranges[i] < a)
            a = ranges[i];
    }
//    myfile<<std::endl;
    std_msgs::Float32 b;
    b.data = a;
    pub_.publish(b);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");

    LaserRead laser_read;

    ros::spin();

    return 0;
}