/*
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "pwm.h"
#include "MPU6050.h"
#include "niming.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "includes.h"

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO       			7 
//设置任务堆栈大小
#define LED0_STK_SIZE  		    		64
//任务堆栈	
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//任务函数
void led0_task(void *pdata);


//LED1任务
//设置任务优先级
#define LED1_TASK_PRIO       			6 
//设置任务堆栈大小
#define LED1_STK_SIZE  					64
//任务堆栈
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//任务函数
void led1_task(void *pdata);

int main(void)
{ 
	delay_init(100-1);		//初始化延时函数
	LED_Init();		        //初始化LED端口	
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	
}

void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
  	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
 	OSTaskCreate(led0_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);						   
 	OSTaskCreate(led1_task,(void *)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);	 				   
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
} 

//LED0任务
void led0_task(void *pdata)
{	 	
	while(1)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		delay_ms(80);
		GPIO_SetBits(GPIOA,GPIO_Pin_5);	
		delay_ms(920);
	};
}

//LED1任务
void led1_task(void *pdata)
{	  
	while(1)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		delay_ms(300);
		GPIO_SetBits(GPIOA,GPIO_Pin_5);	
		delay_ms(300);
	};
}
*/

/*
int main(void)
{ 
    delay_init(100-1);
	LED_Init();
	while(1){
	  GPIO_ResetBits(GPIOA,GPIO_Pin_5);  
	  delay_ms(1000);  		
	  GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	  delay_ms(1000);  
	}	
	return 0;
}
*/

/*
int main(void)
{ 
	delay_init(99);  //初始化延时函数
	uart_init(500000);
	
	//PWM	
	TIM1_PWM_Init(20000,100-1);
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 2000;
	TIM1->CCR3 = 2000;
	TIM1->CCR4 = 2000;
	delay_ms(4000);
	TIM1->CCR1 = 1000;
	TIM1->CCR2 = 1000;
	TIM1->CCR3 = 1000;
	TIM1->CCR4 = 1000;
	delay_ms(5000);
	TIM1->CCR1 = 1100;
	TIM1->CCR2 = 1100;
	TIM1->CCR3 = 1100;
	TIM1->CCR4 = 1100;	
	
	//MPU6050
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	float pitch, roll, yaw;
	MPU_Init();
	while( mpu_dmp_init())
	{
		delay_ms(100);
	}	

	#define inrange(x) ((x)<=1100 ? 1100 : (x)>2000 ? 2000 : (x))	
	
	while(1){
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			TIM1->CCR1 = inrange(1600 - (int)(roll*100));
			TIM1->CCR2 = inrange(1600 - (int)(pitch*100));
			TIM1->CCR3 = inrange(1600 - (int)(yaw*100));
			delay_ms(100);
		}
	}
}
*/

// 串口发送

/*
int main(){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(99);		//延时初始化 
	//uart_init(115200);
	uart_init(500000);
	
	MPU_Init();
	while( mpu_dmp_init())
	{
		delay_ms(100);
	}	
	
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	float pitch, roll, yaw;

	while(1){
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			//printf("\n陀螺仪传感器原始数据:%d\t %d\t %d\n",(int)gyrox,(int)gyroy,(int)gyroz);
			//printf("\n\n加速度传感器原始数据:%d\t %d\t %d\n",(int)aacx,(int)aacy,(int)aacz);
			//mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			delay_ms(100);
		}
	}
	return 0;
}
*/
