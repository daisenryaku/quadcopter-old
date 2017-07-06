#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "MPU6050.h"
#include "niming.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "pid.h"
#include "includes.h"

/*
7.6
DMP����
���� λ���� PID
��ֵ�˲�
*/


/* ************************************ �������� ************************************ */
// ��֤ȡֵ��ĳ����Χ��
#define inrange(x) ((x)<=1000 ? 1000 : (x)>1200 ? 1200 : (x))
#define anglerange(x) ((x)<=-90 ? 90 : (x)>90 ? 90 : (x))	

//��̬������ر���
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
float pitch, roll, yaw;    //ŷ����

// PID��ر���
int16_t motor1=1150, motor2=1150, motor3=1150, motor4=1150; //���ת��
int thr = 1150; //�����źţ���ʵӦ���Ǵ�ң��������
PID_Typedef* pid_Pitch;
PID_Typedef* pid_Yaw; 
PID_Typedef* pid_Roll; 

/* ************************************ �������� ************************************ */

//0 START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//1 ���ת������
#define MOTOR_TASK_PRIO       			4  //�����������ȼ�
#define MOTOR_STK_SIZE  		    	512 //���������ջ��С	
OS_STK MOTOR_TASK_STK[MOTOR_STK_SIZE];     //�����ջ
void MOTOR_task(void *pdata);             //������


//2 PID��������
#define PID_TASK_PRIO 					6   //�����������ȼ�
#define PID_STK_SIZE 					512  //���������ջ��С
OS_STK PID_TASK_STK[PID_STK_SIZE]; 			//�����ջ
void PID_task(void * pdata);      			//������
 			   

//3 GY86-��̬��������
#define GY86_TASK_PRIO       			8  //�����������ȼ�
#define GY86_STK_SIZE  					512 //���������ջ��С
OS_STK GY86_TASK_STK[GY86_STK_SIZE];       //�����ջ
void GY86_task(void *pdata);              //������

/* ************************************  ������ ************************************ */

int main(void)
{ 
	delay_init(100-1);		//��ʼ����ʱ����
    Motor_Init();	        //�����ʼ����ת
	//LED_Init();		    //��ʼ��LED�˿�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
    //uart_init(115200);	//���ڳ�ʼ��������Ϊ115200  
	uart_init(500000);      //���ڳ�ʼ��
	MPU_Init();             //MPU6050��ʼ��
	while( mpu_dmp_init())  //MPU6050 DMP��ʼ��
	{
		delay_ms(10);
	}
	float Kp=1,Ki=0.5,kd=0.01;
	pid_Pitch = (PID_Typedef*)malloc(sizeof(PID_Typedef));
	pid_Yaw = (PID_Typedef*)malloc(sizeof(PID_Typedef));
	pid_Roll = (PID_Typedef*)malloc(sizeof(PID_Typedef));
	PID_Init(pid_Pitch, 0, Kp, Ki , kd);
	PID_Init(pid_Yaw,   0, Kp, Ki , kd);
	PID_Init(pid_Roll , 0, Kp, Ki , kd);
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	
}

/* ************************************ ������ ************************************ */
 //��ʼ����
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
  	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)						   
 	// ����GY86��̬��������
	OSTaskCreate(GY86_task,(void *)0,(OS_STK*)&GY86_TASK_STK[GY86_STK_SIZE-1],GY86_TASK_PRIO);
	// ����PID��������
	 OSTaskCreate(PID_task, (void *)0,(OS_STK*)&PID_TASK_STK[PID_STK_SIZE-1],PID_TASK_PRIO);
    // �������ת������    
 	OSTaskCreate(MOTOR_task,(void *)0,(OS_STK*)&MOTOR_TASK_STK[MOTOR_STK_SIZE-1],MOTOR_TASK_PRIO);	
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
} 

//1 ���ת������
void MOTOR_task(void *pdata)
{	 	
	while(1)
	{
		TIM1->CCR1 = motor1; 
		TIM1->CCR2 = motor2;
		TIM1->CCR3 = motor3;
		TIM1->CCR4 = motor4;
		delay_ms(30);
	};
}


//2 PID����
void PID_task(void *pdata)
{
	while(1)
	{
		float Pitch_C,Roll_C,Yaw_C;
		
		Pitch_C = PID_Update(pid_Pitch, pitch, 0);
		Roll_C  = PID_Update(pid_Yaw,   yaw,   0);
		Yaw_C   = PID_Update(pid_Roll,  roll,  0);
	    motor1 = (int16_t)(thr  - Pitch_C - Roll_C - Yaw_C );    
		motor2 = (int16_t)(thr  - Pitch_C + Roll_C + Yaw_C );    
		motor3 = (int16_t)(thr  + Pitch_C + Roll_C - Yaw_C );    
		motor4 = (int16_t)(thr  + Pitch_C - Roll_C + Yaw_C );
		delay_ms(20);
	}		
}


//3 GY86��̬��������
void GY86_task(void *pdata)
{	  
	while(1){
		
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			pitch = FilterGx( pitch );
			roll = FilterGx( roll );
			yaw = FilterGx(yaw);
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,-(int)(roll*100),-(int)(pitch*100),(int)(yaw*10));
		}
		//printf("\r\nTask2---GY86\r\n");
		delay_ms(10);
	}
}

