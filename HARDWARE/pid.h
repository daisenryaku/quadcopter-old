#ifndef __PID_H
#define __PID_H

#define IMU_UPDATE_DT 0.004
#define PIDdeadband 0.01

extern float pitch;
extern float roll;
extern float yaw;
typedef struct{
	float desired; 
	float error;  //比例
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

float FilterGx(float in);
void PID_Init(PID_Typedef* pid, const float desired, const float kp, const float ki, const float kd);
float PID_Update(PID_Typedef* pid, const float measured, float desired);





#endif

