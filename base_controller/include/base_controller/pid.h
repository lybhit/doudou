#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <math.h>
#include <algorithm>

typedef struct{
  float dt_min;
  float kp;
  float ki;
  float kd;
  float integral;
  float integral_limit;
  float output_limit;
  float error_previous;
  float last_output;
}PID_t;

void pid_init(PID_t *pid, float dt_min)
{
  pid->dt_min = dt_min;
  pid->kp = 0.0f;
  pid->ki = 0.0f;
  pid->kd = 0.0f;
  pid->integral = 0.0f;
  pid->integral_limit = 0.0f;
  pid->output_limit = 0.0f;
  pid->error_previous = 0.0f;
  pid->last_output = 0.0f;
}

void pid_set_parameters(PID_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  
}

float pid_calculate(PID_t *pid, float sp, float val, float val_dot, float dt)
{
//   if(!isinf(sp) || !isinf(val) || !isinf(val_dot) || !isinf(dt)){
//     return pid->last_output;
//   }
  
  float i, d;
  
  //current error value
  float error = sp - val;
  
  //current error derivative
  d = (error - pid->error_previous) / std::max(dt, pid->dt_min);
  
  pid->error_previous = error;
  
  float output = (error * pid->kp) + (d * pid->kd);
  
  i = pid->integral + (error * dt);
  
  if((abs(output + (i * pid->ki)) <= pid->output_limit) && abs(i) <= pid->integral_limit)
  {
    pid->integral = i;
    output += pid->integral * pid->ki;
  }
  
  if(std::isfinite(output))
  {
    if(output > pid->output_limit){
      output = pid->output_limit;
    }else if(output < -pid->output_limit){
      output = -pid->output_limit;
    }
    
    pid->last_output = output;
  }
  
  return pid->last_output;
  
}

void pid_reset_integral(PID_t *pid)
{
  pid->integral = 0.0f;
}

#endif

































