#ifndef __READ_ENCODER__
#define __READ_ENCODER__

extern union floatDara
{
  float d;
  unsigned char data[4];
}position_x,position_y,orientation,vel_linear,vel_angular;

extern float wheel_interval;//    500.0f;        
extern float multiplier;           //倍频数
extern float deceleration_ratio;  //减速比
extern float wheel_diameter;     //轮子直径
extern float pi_1_2;                  //π/2
extern float pi;              //π
extern float pi_3_2;                  //π*3/2
extern float pi_2_1;                  //π*2
extern float dt;                 //采样时间间隔
extern float line_number;       //码盘线数
extern float orientation_interval;  //dt时间内方向变化值 
extern float sin_;        //角度计算值
extern float cos_; 
extern float delta_distance,delta_orientation;   //采样时间间隔内运动距离 
extern float const_frame,const_angle,distance_sum,distance_diff; 
extern float orientation_1;
 
extern unsigned int once;




void odometry(short int, short int);




#endif
