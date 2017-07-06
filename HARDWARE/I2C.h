# ifndef _I2CDEV_H
# define _I2CDEV_H

# include "delay.h"
# include "stm32f4xx.h"

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=0<<5*2;}	//PB5����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(5*2));GPIOB->MODER|=1<<5*2;} //PB5���ģʽ
//IO��������	 
#define IIC_SCL    PBout(4) //SCL
#define IIC_SDA    PBout(5) //SDA	 
#define READ_SDA   PBin(5)  //����SDA

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
#endif
