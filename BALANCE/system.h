#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//引用所有需要用到的头文件
#include "FreeRTOSConfig.h"
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "timer.h"


// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
	double EncoderDelta;
}Motor_parameter;

typedef struct
{
	float VSpeed;
	float WSpeed;
	float DeltaDis;
	float DeltaAngle;
	float XPosition;
	float YPosition;
	float WPosition;
	float Ldis;
	float Rdis;
	float Lv;
	float Rv;
}Odometry;

typedef struct PIDpara
{
    double Kp;
    double Ki;
    double Kd;
    
    double RDeltaEncoder,LDeltaEncoder;
    double RError,LError;
    double RLastError,LLastError;
    double RIntegral,LIntegral;
    double RDiffer,LDiffer;
    double RProperty,LProperty;
    
    float ROutPWM,LOutPWM;
}PID;


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern int controller;
extern u8 temp_rxbuf[8];
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float move_x, move_y, move_z;
extern float Velocity_KP, Velocity_KI,Velocity_KD;
extern float Velocity_KP_s, Velocity_KI_s,Velocity_KD_s;	
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern Odometry ODOMETRY;
extern PID Pid;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;                                            										                 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern int connect_count;
extern int last_connect_count;
extern int Pause_state;
extern int Pause_count;
extern int Encoder_state;
extern double C11,C12,C21,C22;
extern double D11,D12,D21,D22;
extern double cur_V_Speed;
extern double cur_W_Speed;
extern double RDeltaEncoder_Reference;
extern double LDeltaEncoder_Reference;
extern double DeltaW;
extern double DeltaV;
extern int Control_mode;
extern double Joy_v_Speed;
extern double Joy_w_Speed;
extern float Electricity;
extern float Current;
extern int Led_State;
extern int Error_Up;
extern int Error_Down;
extern float Speed_Limit;
extern int t_timer;

void systemInit(void);
void selfCheck(void);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
