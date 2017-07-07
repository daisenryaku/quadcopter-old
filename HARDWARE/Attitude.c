#include "Attitude.h"
#define AtR 0.0174533l
#define RtA 57.2956l

float kp = 100.0f;              // ��������֧�������������ٶ�
float ki = 0.1f;                // �����ǻ�������֧����
//float halfT = 0.002f;           // ��������һ��
float halfT = 0.01f;           // ��������һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   //��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;  // ������С�������
extern float yaw, pitch, roll; 			    //��̬��

void Attitude(short gx, short gy, short gz, short ax, short ay, short az) 
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
	
	//������ٶȵ�λ�� 
    norm = sqrt(ax*ax + ay*ay + az*az);    
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;      

	//ת��Ϊ�������Ҿ������������Ԫ��
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //������������������������Ǻͼ��ٶ�֮������
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //���������
    exInt = exInt + ex*ki;
    eyInt = eyInt + ey*ki;
    ezInt = ezInt + ez*ki;

	//����������
    gx = gx + kp*ex + exInt;
    gy = gy + kp*ey + eyInt;
    gz = gz + kp*ez + ezInt;

	//һ�����������������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT; 
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

	//�淶��Ԫ��
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3); 
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
	
    //��Ԫ��תŷ����	
	roll  = asin(-2.f * q1 * q3 + 2 * q0* q2)* RtA; // Roll
	pitch = atan2(2.f *(q2*q3 + q0*q1),q0*q0-q1*q1-q2*q2+q3*q3)* RtA;
	yaw   = atan2(2.f *(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * RtA;              
}

