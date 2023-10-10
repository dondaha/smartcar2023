#include "system.h"

//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
//0=Mec_Car，1=Omni_Car，2=Akm_Car，3=Diff_Car，4=FourWheel_Car，5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=1000; 

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;
float move_x, move_y, move_z;

int connect_count=0;
int last_connect_count=0;
int Pause_state=0;
int Pause_count=0;
int Encoder_state=1;

u8 temp_rxbuf[8];

//PID parameters of Speed control
//速度控制PID参数
//float Velocity_KP=20,Velocity_KI=15,Velocity_KD=0; 
float Velocity_KP=30,Velocity_KI=0.01,Velocity_KD=0;
float Velocity_KP_s=30,Velocity_KI_s=0.01,Velocity_KD_s=0;
//float Velocity_KP=60,Velocity_KI=50,Velocity_KD=10;

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

//里程计结构体
Odometry ODOMETRY;

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//全向轮转弯半径，单位：m
float Omni_turn_radiaus; 
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2手柄、蓝牙APP、航模手柄、CAN通信、串口1、串口5通信控制标志位。这6个标志位默认都为0，代表串口3控制模式
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag=1; 

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Self-check the relevant flag variables
//自检相关标志变量
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; 

//Check the result code
//自检结果代码
long int ErrorCode=0; 

PID Pid;
double cur_V_Speed;
double cur_W_Speed;
double RDeltaEncoder_Reference;
double LDeltaEncoder_Reference;
double DeltaW;
double DeltaV;
int Control_mode = 0x03;
double Joy_v_Speed;
double Joy_w_Speed;
float Electricity=-1;
float Current;
int Led_State=1;
int Error_Up;
int Error_Down;
float Speed_Limit=1.5;
int t_timer=0;

void systemInit(void)
{       
	
//	//Interrupt priority group setti  ng
//	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	
//	//Delay function initialization
//	//延时函数初始化
	delay_init(168);	
	
	//Serial port 3 is initialized and the baud rate is 115200. 
	//Serial port 3 is the default port used to communicate with ROS terminal
	//串口3初始化，通信波特率115200，串口3为默认用于与ROS端通信的串口
	//uart3_init(115200);
	
	//Serial port 5 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//串口5初始化，通信波特率115200，可用于与ROS端通信
	uart5_init(115200);              
	
	 //初始化底层控制，1ms中断触发一次
	//TIM7_Init(99,84-1);  //高级定时器TIM7的时钟频率为84M    

  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
  //APB1时钟频率为84M，满PWM为5000
	TIM4_PWM_Init(5000,84);
}
