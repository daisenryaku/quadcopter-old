# include "MPU6050.h"
# include "niming.h"
# include "sys.h"
# include "usart.h"
/*
u8 MPU_Init(void)
{
	IIC_Init();
	delay_ms(800);																	
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);					//��λ
	delay_ms(200);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);					//����
	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);					//�������ٶȣ�������
	MPU_Write_Byte(MPU_GYRO_CFG_REG,0X08);  				//���������������̷�Χ 500 deg/s  65.5 LSB/deg/s  
	MPU_Write_Byte(MPU_ACCEL_CFG_REG,0X00);  				//���ٶȴ����������̷�Χ   2g  16384 LSB/g
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,0X07);				//����Ƶ��125Hz
	MPU_Write_Byte(MPU_CFG_REG,0X06); 						//��ͨ�˲�������ֵ 5Hz �����ǣ����ٶȼ����Ƶ�� = 1KHz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);					//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);					//I2C��ģʽ�رգ�HCM588L������������
	MPU_Write_Byte(MPU_FIFO_EN_REG,0x00);					//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X02);					//������������ֱ�ӷ���HCM588L������IIC��
	return 0;
}
*/
u8 MPU_Init(void)
{ 
	u8 res;
	IIC_Init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(1);					//�����Ǵ�����,��500dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(125);						//���ò�����125Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(250);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�¶�
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(((u16)buf[0]<<8)|buf[1]);;  
		*gy=(((u16)buf[2]<<8)|buf[3]);;  
		*gz=(((u16)buf[4]<<8)|buf[5]);;
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//ax,ay,az���ٶ�x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart2_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	
		if(IIC_Wait_Ack())		
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);
    IIC_Wait_Ack();		
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);
		else *buf=IIC_Read_Byte(1);		
		len--;
		buf++; 
	}    
    IIC_Stop();	
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack())	
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);	
	IIC_Wait_Ack();		
    IIC_Send_Byte(reg);	
    IIC_Wait_Ack();		
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);
    IIC_Wait_Ack();		
	res=IIC_Read_Byte(0);
    IIC_Stop();			
	return res;		
}


