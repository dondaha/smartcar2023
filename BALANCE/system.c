#include "system.h"

//Robot software fails to flag bits
//���������ʧ�ܱ�־λ
u8 Flag_Stop=1;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺ�
int Divisor_Mode;

// Robot type variable
//�������ͺű���
//0=Mec_Car��1=Omni_Car��2=Akm_Car��3=Diff_Car��4=FourWheel_Car��5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//�������PWMֵ��������С��ר��
int Servo;  

//Default speed of remote control car, unit: mm/s
//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=1000; 

//Vehicle three-axis target moving speed, unit: m/s
//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z;
float move_x, move_y, move_z;

int connect_count=0;
int last_connect_count=0;
int Pause_state=0;
int Pause_count=0;
int Encoder_state=1;

u8 temp_rxbuf[8];

//PID parameters of Speed control
//�ٶȿ���PID����
//float Velocity_KP=20,Velocity_KI=15,Velocity_KD=0; 
float Velocity_KP=30,Velocity_KI=0.01,Velocity_KD=0;
float Velocity_KP_s=30,Velocity_KI_s=0.01,Velocity_KD_s=0;
//float Velocity_KP=60,Velocity_KI=50,Velocity_KD=10;

//The parameter structure of the motor
//����Ĳ����ṹ��
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

//��̼ƽṹ��
Odometry ODOMETRY;

/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//����������
float Encoder_precision; 
//Wheel circumference, unit: m
//�����ܳ�����λ��m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//�������־࣬��λ��m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//С��ǰ�������࣬��λ��m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//ȫ����ת��뾶����λ��m
float Omni_turn_radiaus; 
/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1, serial port 5 communication control flag bit.
//These 6 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2�ֱ�������APP����ģ�ֱ���CANͨ�š�����1������5ͨ�ſ��Ʊ�־λ����6����־λĬ�϶�Ϊ0��������3����ģʽ
u8 PS2_ON_Flag=0, APP_ON_Flag=0, Remote_ON_Flag=0, CAN_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag=1; 

//Bluetooth remote control associated flag bits
//����ң����صı�־λ
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//������APP���Ͳ����ı�־λ
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2�ֱ�������ر���
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Self-check the relevant flag variables
//�Լ���ر�־����
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; 

//Check the result code
//�Լ�������
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
//	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//	
//	//Delay function initialization
//	//��ʱ������ʼ��
	delay_init(168);	
	
	//Serial port 3 is initialized and the baud rate is 115200. 
	//Serial port 3 is the default port used to communicate with ROS terminal
	//����3��ʼ����ͨ�Ų�����115200������3ΪĬ��������ROS��ͨ�ŵĴ���
	//uart3_init(115200);
	
	//Serial port 5 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//����5��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart5_init(115200);              
	
	 //��ʼ���ײ���ƣ�1ms�жϴ���һ��
	//TIM7_Init(99,84-1);  //�߼���ʱ��TIM7��ʱ��Ƶ��Ϊ84M    

  //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
  //��ʼ������ٶȿ����Լ������ڿ��Ƶ���ٶȣ�PWMƵ��10KHZ
  //APB1ʱ��Ƶ��Ϊ84M����PWMΪ5000
	TIM4_PWM_Init(5000,84);
}
