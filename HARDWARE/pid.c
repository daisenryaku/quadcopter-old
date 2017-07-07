#include <pid.h>
#include <math.h>

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

float PID_Update(PID_Typedef* pid, const float measured, float desired){
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

void aWind_Filter(short *ax, short*ay, short* az){
  static short ax_para[WIND_SIZE] = {0};
  static short ay_para[WIND_SIZE] = {0};
  static short az_para[WIND_SIZE] = {16384,16384,16384,16384,16384,16384,16384,16384,16384,16384};
 
  static short pos = 9;
  static long int ax_sum,ay_sum,az_sum = 16384*9;
 
  ax_para[pos] = *ax;
  ay_para[pos] = *ay;
  az_para[pos] = *az;
 
  pos = (pos+1)%WIND_SIZE;
 
  ax_sum-=ax_para[pos];
  ay_sum-=ay_para[pos];
  az_sum-=az_para[pos];
 
  ax_sum+=*ax;
  ay_sum+=*ay;
  az_sum+=*az;
 
  *ax = ax_sum/10.0;
  *ay = ay_sum/10.0;
  *az = az_sum/10.0;
  return;
}




