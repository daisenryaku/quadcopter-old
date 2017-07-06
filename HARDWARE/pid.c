#include <pid.h>
#include <math.h>

float FilterGx(float in)  //����ɼ��ź� in�������˲������ֵ 
{
static double w[5] = {0};  //�����λ�Ĵ��� 
static double y;  //���ֵ 
//����� N ����ʱ�� 
w[0] = in;
y = 0.2 * w[0] + 0.2 * w[1] + 0.2 * w[2] + 0.2 * w[3] + 0.2 * w[4];
w[4] = w[3]; 
w[3] = w[2]; 
w[2] = w[1]; 
w[1] = w[0]; 
return  (float)y;   //�����˲������ֵ 
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
  pid->desired = desired;                              //��ȡ�����Ƕ�
  pid->error   = pid->desired - measured;              //ƫ�����-����ֵ
  pid->integ   += pid->error * IMU_UPDATE_DT;          //����

  if (pid->integ > pid->iLimit)                        //��������
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }
  
  pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;     //΢��,Ӧ�ÿ��������ǽ��ٶȴ���:pid->deriv = -gyro;
  
  if(fabs(pid->error) > PIDdeadband )                             //pid����
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
  pid->prevError = pid->error;                          //����ǰһ��ƫ��
  pid->prevOutput = pid->output;
  
  return pid->output;
}

