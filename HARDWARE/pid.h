#ifndef __PID_H
#define __PID_H

//#define IMU_UPDATE_DT 0.004
#define IMU_UPDATE_DT 0.02

#define PIDdeadband 0.01
#define WIND_SIZE 10 

extern float pitch;
extern float roll;
extern float yaw;
typedef struct{
	float desired; 
	float error;  //比例
	float nextError;
	float prevError;
	float integ;  //积分
	float iLimit;
	float deriv;  //微分
	float kp;
	float ki;
	float kd;
	float outP;
	float outI;
	float outD;
	float output;
	float prevOutput;
} PID_Typedef;


void PID_Init(PID_Typedef* pid, const float desired, const float kp, const float ki, const float kd);
float PID_Update(PID_Typedef* pid, const float measured, float desired);
void aWind_Filter(short *ax, short*ay, short* az);
#endif

