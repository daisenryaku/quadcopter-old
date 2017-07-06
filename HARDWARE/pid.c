#include <pid.h>
#include <math.h>

float FilterGx(float in)  //输入采集信号 in，返回滤波器输出值 
{
static double w[5] = {0};  //输出移位寄存器 
static double y;  //输出值 
//输出的 N 级延时链 
w[0] = in;
y = 0.2 * w[0] + 0.2 * w[1] + 0.2 * w[2] + 0.2 * w[3] + 0.2 * w[4];
w[4] = w[3]; 
w[3] = w[2]; 
w[2] = w[1]; 
w[1] = w[0]; 
return  (float)y;   //返回滤波器输出值 
} 

void PID_Init(PID_Typedef* pid, const float desired, const float kp, const float ki, const float kd){
	pid->desired = desired;
	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->iLimit = 100;
	pid->deriv = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->outP = 0;
	pid->outI = 0;
	pid->outD = 0;
	pid->output = 0;
	pid->prevOutput = 0;
}	

float PID_Update(PID_Typedef* pid, const float measured, float desired)
{
  pid->desired = desired;                              //获取期望角度
  pid->error   = pid->desired - measured;              //偏差：期望-测量值
  pid->integ   += pid->error * IMU_UPDATE_DT;          //积分

  if (pid->integ > pid->iLimit)                        //积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }
  
  pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;     //微分,应该可用陀螺仪角速度代替:pid->deriv = -gyro;
  
  if(fabs(pid->error) > PIDdeadband )                             //pid死区
  {   
    pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	pid->output = pid->outP + pid->outI + pid->outD;
  }
  else
  {
    pid->output = pid->prevOutput;
  }
  pid->prevError = pid->error;                          //更新前一次偏差
  pid->prevOutput = pid->output;
  
  return pid->output;
}

