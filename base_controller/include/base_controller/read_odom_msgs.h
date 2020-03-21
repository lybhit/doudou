#ifndef __READ_ODOM__
#define __READ_ODOM__



/***********************************************  输出 *****************************************************************/
union floatData //union的作用为实现char数组和float之间的转换
 {
      float d;
      unsigned char data[4];
  }position_x,position_y,orientation,vel_linear,vel_angular;


/***********************************************  输入 *****************************************************************/

extern float odometry_right,odometry_left;//串口得到左右轮速度

/***********************************************  变量 *****************************************************************/

float wheel_interval= 500.0f;//    500.0f;        
float multiplier=4.0f;           //倍频数
float deceleration_ratio=90.0f;  //减速比
float wheel_diameter=130.0f;     //轮子直径
float pi_1_2=1.570796f;			 //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;			 //π*3/2
float pi_2_1=6.283186f;			 //π*2
float dt=0.005f;                 //采样时间间隔
float line_number=1400.0f;       //码盘线数
float orientation_interval=0;  //dt时间内方向变化值

float sin_=0;        //角度计算值
float cos_=0;

float delta_distance=0,delta_orientation=0;   //采样时间间隔内运动距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float orientation_1=0;

unsigned int once=1;

/****************************************************************************************************************/

//里程计计算函数
void odometry(short int right, short int left)
{	
    right = -right; //counterclockwise denote positive, important !!!

	if(once)  //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
		const_angle=const_frame/wheel_interval;
		once=0;
	}
    
	distance_sum = 0.5f*(right+left);//小车行驶路程为两轮速度和的一半
	distance_diff = right-left;//小车行驶角度为两轮速度差

	delta_distance = distance_sum;
	delta_orientation = distance_diff;

    
	orientation_interval = delta_orientation * const_angle;//采样时间内行走的角度
	orientation.d = orientation.d + orientation_interval;//计算出里程计的方向角
	orientation_1 = orientation.d + 0.5f * orientation_interval;//里程计方向角数据位数变化，用于三角函数计算
	
        sin_ = sin(orientation_1);//计算出采样时间内y坐标
	cos_ = cos(orientation_1);//计算出采样时间内x坐标
	
        position_x.d = position_x.d + delta_distance * cos_ * const_frame;//计算出x坐标
	position_y.d = position_y.d + delta_distance * sin_ * const_frame;//计算出y坐标
    
	vel_linear.d = delta_distance*const_frame / dt;//计算出里程计x坐标
	vel_angular.d = orientation_interval / dt;//计算出里程计y坐标
	
    //方向角角度校正
	if(orientation.d > pi)
	{
		orientation.d -= pi_2_1;
	}
	else
	{
		if(orientation.d < -pi)
		{
			orientation.d += pi_2_1;
		}
	}
}

#endif
