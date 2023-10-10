#include "timer.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u8 TIM8CH1_CAPTURE_STA = 0;	
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM8CH2_CAPTURE_STA = 0;		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM8CH3_CAPTURE_STA = 0;		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志
u8 TIM8CH4_CAPTURE_STA = 0;			
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;

u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;

//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;
int encoder_count=0;

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：航模遥控初始化函数，定时器1输入捕获初始化
入口参数：arr：自动重装值，psc：时钟预分频数 
返 回 值：无
**************************************************************************/ 
void TIM7_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr;//1000-1;//10ms 设置自动重载周期值
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;  // 分频后的频率是1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频因子
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 设置计数方式向上计数
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update, ENABLE); //允许TIme6 更新中断使能

   // 中断优先级设置
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel =TIM7_IRQn;//TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1;//占先式优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //副优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断使能
	NVIC_Init(&NVIC_InitStructure);//中断初始化	

	TIM_Cmd(TIM7, ENABLE);  // 使能TIM3
}

/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 8 input capture interrupt
Input   : none
Output  : none
函数功能：航模遥控接收中断，即定时器8输入捕获中断
入口参数：无
返 回 值：无
**************************************************************************/ 
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		
	}
}
