#ifndef __NIMING_H
#define __NIMING_H
#include "sys.h"

void usart2_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void usart2_niming_report(u8 fun,u8*data,u8 len);
void usart2_send_char(u8 c);
#endif
