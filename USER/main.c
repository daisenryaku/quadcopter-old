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
DMP解算
单级 位置型 PID
均值滤波
*/


/* ************************************ 变量声明 ************************************ */
// 保证取值在某个范围内
#define inrange(x) ((x)<=1000 ? 1000 : (x)>1200 ? 1200 : (x))
#define anglerange(x) ((x)<=-90 ? 90 : (x)>90 ? 90 : (x))	

//姿态解算相关变量
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
float pitch, roll, yaw;    //欧拉角

// PID相关变量
int16_t motor1=1150, motor2=1150, motor3=1150, motor4=1150; //电机转速
int thr = 1150; //油门信号，其实应该是从遥控器读的
PID_Typedef* pid_Pitch;
PID_Typedef* pid_Yaw; 
PID_Typedef* pid_Roll; 

/* ************************************ 任务声明 ************************************ */

//0 START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//1 电机转动任务
#define MOTOR_TASK_PRIO       			4  //设置任务优先级
#define MOTOR_STK_SIZE  		    	512 //设置任务堆栈大小	
OS_STK MOTOR_TASK_STK[MOTOR_STK_SIZE];     //任务堆栈
void MOTOR_task(void *pdata);             //任务函数


//2 PID调节任务
#define PID_TASK_PRIO 					6   //设置任务优先级
#define PID_STK_SIZE 					512  //设置任务堆栈大小
OS_STK PID_TASK_STK[PID_STK_SIZE]; 			//任务堆栈
void PID_task(void * pdata);      			//任务函数
 			   

//3 GY86-姿态解算任务
#define GY86_TASK_PRIO       			8  //设置任务优先级
#define GY86_STK_SIZE  					512 //设置任务堆栈大小
OS_STK GY86_TASK_STK[GY86_STK_SIZE];       //任务堆栈
void GY86_task(void *pdata);              //任务函数

/* ************************************  主函数 ************************************ */

int main(void)
{ 
	delay_init(100-1);		//初始化延时函数
    Motor_Init();	        //电机初始化能转
	//LED_Init();		    //初始化LED端口
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
    //uart_init(115200);	//串口初始化波特率为115200  
	uart_init(500000);      //串口初始化
	MPU_Init();             //MPU6050初始化
	while( mpu_dmp_init())  //MPU6050 DMP初始化
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
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
}

/* ************************************ 任务函数 ************************************ */
 //开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
  	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)						   
 	// 创建GY86姿态解算任务
	OSTaskCreate(GY86_task,(void *)0,(OS_STK*)&GY86_TASK_STK[GY86_STK_SIZE-1],GY86_TASK_PRIO);
	// 创建PID调节任务
	 OSTaskCreate(PID_task, (void *)0,(OS_STK*)&PID_TASK_STK[PID_STK_SIZE-1],PID_TASK_PRIO);
    // 创建电机转动任务    
 	OSTaskCreate(MOTOR_task,(void *)0,(OS_STK*)&MOTOR_TASK_STK[MOTOR_STK_SIZE-1],MOTOR_TASK_PRIO);	
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
} 

//1 电机转动任务
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


//2 PID调节
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


//3 GY86姿态解算任务
void GY86_task(void *pdata)
{	  
	while(1){
		
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			pitch = FilterGx( pitch );
			roll = FilterGx( roll );
			yaw = FilterGx(yaw);
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,-(int)(roll*100),-(int)(pitch*100),(int)(yaw*10));
		}
		//printf("\r\nTask2---GY86\r\n");
		delay_ms(10);
	}
}

