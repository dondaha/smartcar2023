#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4
#define DATA_STK_SIZE2   512 
#define DATA_TASK_PRIO2  4

#define FRAME_HEADER      0XAF //Frame_header //接收帧头
#define FRAME_HEADER2			0XBF //Frame_header //发送帧头
#define FRAME_TAIL        0X7D //Frame_tail   //帧尾
#define SEND_DATA_SIZE    28
#define RECEIVE_DATA_SIZE 90

extern int turn;
extern int diss[360];

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		unsigned char Code;
		short X_speed;	            //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		int32_t Position_X;
		int32_t Position_Y;
		short Position_W;
		short Power_Voltage;        //2 bytes //2个字节
		unsigned char charge_state;
		short Pitch;
		unsigned char Control_mode;
		short Joy_v_speed;
		short Joy_w_speed;
		unsigned char Error_Code;
		unsigned char Frame_Count;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		float start_ang;
		float end_ang;
	}Info;
	
}RECEIVE_DATA;

typedef struct _ELECTRICITY_DATA_ 
{
	unsigned char buffer[17];
}ELECTRICITY_DATA;

void data_task(void *pvParameters);
void data_task2(void *pvParameters);
void data_transition(void);
void data_transition1(void);
//void USART1_SEND(void);
void USART2_SEND(void);
void USART5_STOP_SEND(void);
void USART5_START_SEND(void);

void Relay_Init(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart5_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int USART3_IRQHandler(void);
int UART5_IRQHandler(void);

float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart2_send(u8 data);
void usart3_send(u8 data);
void usart5_send(u8 data);
void turn_right(void);
void go_staight(void);
void go_squire(void);

u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);


#endif

