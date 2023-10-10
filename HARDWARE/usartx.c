#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
ELECTRICITY_DATA Electricity_Data;
int turn=0;
int Count=100;
u8 Res_one;
unsigned char header[2];
int diss[360];//����360��ɨ���������
int Res=0;
int dis,ang,diff;

/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
�������ܣ�����5��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART5_STOP_SEND(void)
{
	USART_ClearFlag(UART5,USART_FLAG_TC);	
  if(1)    
	{	USART_SendData(UART5,0xA5);   //�Ӵ���2���Ϳ�ʼָ��  USART_FLAG_TC�� ������λ�Ĵ���������ɱ�־λ��ȫ��������ϻ��� 1
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		USART_SendData(UART5,0x25);		//�Ӵ���2���ͽ���ָ��
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}	
}
void USART5_START_SEND(void)
{
  //unsigned char i = 0;	
	//u8 scan[]={0xA5,0x20};
	//for(i=0; i<2; i++)
	//{
		//usart5_send(buffertest[i]);
		//usart5_send(scan[i]);
	//}
	USART_ClearFlag(UART5,USART_FLAG_TC);	
  if(1)    
	{	USART_SendData(UART5,0xA5);   //�Ӵ���2���Ϳ�ʼָ��  USART_FLAG_TC�� ������λ�Ĵ���������ɱ�־λ��ȫ��������ϻ��� 1
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		USART_SendData(UART5,0x20);		//�Ӵ���2���ͽ���ָ��
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}	
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
		//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;						//Tx�ӿ�ΪC12
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;							//Rx�ӿ�ΪD2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
}
/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Res_one = USART_ReceiveData(UART5);
		if(Res_one==0xAA && Res==0)//֡ͷ�ж��Ƿ�ΪAA55
		{
			header[0]=Res_one;
			Res++;
		}
		else if(Res_one==0x55 && Res==1)
		{
			header[1]=Res_one;
			Res++;
		}
		else if(header[0]==0xaa && header[1]==0x55 && Res>=2)
		{
			Receive_Data.buffer[Res]=Res_one;
			Res++;
		}
		else
			Res=0;

		if(Res==90)//���δ���������ݲ�����90֡
		{
			Res=0;
			Count=Receive_Data.buffer[3];//һ�ν��յĲ�������
			Receive_Data.Info.start_ang=((Receive_Data.buffer[5]<<7) + (Receive_Data.buffer[4]>>1)) / 64;//������ʼ�Ƕ�
			Receive_Data.Info.end_ang=((Receive_Data.buffer[7]<<7) + (Receive_Data.buffer[6]>>1)) / 64;//�����յ�Ƕ�
			if(Receive_Data.Info.end_ang<Receive_Data.Info.start_ang) 
				diff = 360-Receive_Data.Info.start_ang+Receive_Data.Info.end_ang;
			else
				diff = Receive_Data.Info.end_ang-Receive_Data.Info.start_ang;
			
			for(int i=12; i<Count*2+10; i+=2)
			{
				ang=(int)(Receive_Data.Info.start_ang + (diff/(Count-1)) * ((i-10)/2));//��������м��Ƕ�
				dis=(Receive_Data.buffer[i+1]<<6) + (Receive_Data.buffer[i]>>2);//��Ӧ��ľ���
				if(ang<=360)
					diss[ang]=dis;
				else
					continue;
			}

		}
		else if(Res==90 && Receive_Data.buffer[1]!=0x55)
			Res=0;  
	}
	
  return 0;	
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
�������ܣ�����5��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
