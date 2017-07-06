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

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
 			   
//LED0����
//�����������ȼ�
#define LED0_TASK_PRIO       			7 
//���������ջ��С
#define LED0_STK_SIZE  		    		64
//�����ջ	
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//������
void led0_task(void *pdata);


//LED1����
//�����������ȼ�
#define LED1_TASK_PRIO       			6 
//���������ջ��С
#define LED1_STK_SIZE  					64
//�����ջ
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//������
void led1_task(void *pdata);

int main(void)
{ 
	delay_init(100-1);		//��ʼ����ʱ����
	LED_Init();		        //��ʼ��LED�˿�	
	OSInit();   
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	
}

void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
  	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
 	OSTaskCreate(led0_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);						   
 	OSTaskCreate(led1_task,(void *)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);	 				   
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
} 

//LED0����
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

//LED1����
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
	delay_init(99);  //��ʼ����ʱ����
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
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	float pitch, roll, yaw;
	MPU_Init();
	while( mpu_dmp_init())
	{
		delay_ms(100);
	}	

	#define inrange(x) ((x)<=1100 ? 1100 : (x)>2000 ? 2000 : (x))	
	
	while(1){
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			TIM1->CCR1 = inrange(1600 - (int)(roll*100));
			TIM1->CCR2 = inrange(1600 - (int)(pitch*100));
			TIM1->CCR3 = inrange(1600 - (int)(yaw*100));
			delay_ms(100);
		}
	}
}
*/

// ���ڷ���

/*
int main(){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(99);		//��ʱ��ʼ�� 
	//uart_init(115200);
	uart_init(500000);
	
	MPU_Init();
	while( mpu_dmp_init())
	{
		delay_ms(100);
	}	
	
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	float pitch, roll, yaw;

	while(1){
		if ( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 ){
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			usart2_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			//printf("\n�����Ǵ�����ԭʼ����:%d\t %d\t %d\n",(int)gyrox,(int)gyroy,(int)gyroz);
			//printf("\n\n���ٶȴ�����ԭʼ����:%d\t %d\t %d\n",(int)aacx,(int)aacy,(int)aacz);
			//mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			delay_ms(100);
		}
	}
	return 0;
}
*/
