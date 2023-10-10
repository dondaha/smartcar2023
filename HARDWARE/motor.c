#include "motor.h"


/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
函数功能：使能开关引脚初始化
入口参数：无
返回  值：无 
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //KEY对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOB14
} 

void TIM4_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOF时钟
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4时钟

GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);//复用
GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);//复用

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;//PB6,PB7
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化IO

TIM_TimeBaseInitStructure.TIM_Period = arr-1;//自动重装载
TIM_TimeBaseInitStructure.TIM_Prescaler = psc-1;//预分频
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//时钟分频
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM模式
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//比较极性高

TIM_OC1Init(TIM4,&TIM_OCInitStructure);
TIM_OC2Init(TIM4,&TIM_OCInitStructure); 

TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);//输出比较预装载使能
TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);

TIM_CtrlPWMOutputs(TIM4,ENABLE);

TIM_ARRPreloadConfig(TIM4,ENABLE);//自动重载预装载使能

TIM_Cmd(TIM4,ENABLE);//计数使能
}