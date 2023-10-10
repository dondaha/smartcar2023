#include "timer.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��1���벶���־������λ�������־����6λ�������־		
u8 TIM8CH1_CAPTURE_STA = 0;	
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��2���벶���־������λ�������־����6λ�������־	
u8 TIM8CH2_CAPTURE_STA = 0;		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��3���벶���־������λ�������־����6λ�������־	
u8 TIM8CH3_CAPTURE_STA = 0;		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��4���벶���־������λ�������־����6λ�������־
u8 TIM8CH4_CAPTURE_STA = 0;			
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;

u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;

//Variables related to remote control acquisition of model aircraft
//��ģң�زɼ���ر���
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//��ģң�ؽ��ձ���
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;
int encoder_count=0;

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ���ģң�س�ʼ����������ʱ��1���벶���ʼ��
��ڲ�����arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ�� 
�� �� ֵ����
**************************************************************************/ 
void TIM7_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr;//1000-1;//10ms �����Զ���������ֵ
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;  // ��Ƶ���Ƶ����1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ʱ�ӷ�Ƶ����
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ü�����ʽ���ϼ���
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update, ENABLE); //����TIme6 �����ж�ʹ��

   // �ж����ȼ�����
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel =TIM7_IRQn;//TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1;//ռ��ʽ���ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�ж�ʹ��
	NVIC_Init(&NVIC_InitStructure);//�жϳ�ʼ��	

	TIM_Cmd(TIM7, ENABLE);  // ʹ��TIM3
}

/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 8 input capture interrupt
Input   : none
Output  : none
�������ܣ���ģң�ؽ����жϣ�����ʱ��8���벶���ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/ 
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		
	}
}
