#include "led.h"
#include "delay.h"
#include "stm32f4xx.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

void LED_Task(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);  //LED2��Ӧ����GPIOA.5���ͣ���  ��ͬLED2=0;
	delay_ms(1000);  		   //��ʱ500ms
	GPIO_SetBits(GPIOA,GPIO_Pin_5);	   //LED2��Ӧ����GPIOA.5���ߣ���  ��ͬLED2=1;
	delay_ms(1000);                     //��ʱ500ms
}	
