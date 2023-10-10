#include "motor.h"


/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
�������ܣ�ʹ�ܿ������ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //KEY��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOB14
} 

void TIM4_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��GPIOFʱ��
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4ʱ��

GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);//����
GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);//����

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//����
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;//PB6,PB7
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ��IO

TIM_TimeBaseInitStructure.TIM_Period = arr-1;//�Զ���װ��
TIM_TimeBaseInitStructure.TIM_Prescaler = psc-1;//Ԥ��Ƶ
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ʱ�ӷ�Ƶ
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWMģʽ
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//���
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�Ƚϼ��Ը�

TIM_OC1Init(TIM4,&TIM_OCInitStructure);
TIM_OC2Init(TIM4,&TIM_OCInitStructure); 

TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);//����Ƚ�Ԥװ��ʹ��
TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);

TIM_CtrlPWMOutputs(TIM4,ENABLE);

TIM_ARRPreloadConfig(TIM4,ENABLE);//�Զ�����Ԥװ��ʹ��

TIM_Cmd(TIM4,ENABLE);//����ʹ��
}