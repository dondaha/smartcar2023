#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
ELECTRICITY_DATA Electricity_Data;
int turn=0;
int Count=100;
u8 Res_one;
unsigned char header[2];
int diss[360];//保存360°扫描距离数据
int Res=0;
int dis,ang,diff;

/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
函数功能：串口5发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART5_STOP_SEND(void)
{
	USART_ClearFlag(UART5,USART_FLAG_TC);	
  if(1)    
	{	USART_SendData(UART5,0xA5);   //从串口2发送开始指令  USART_FLAG_TC： 发送移位寄存器发送完成标志位，全部发送完毕会置 1
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
		USART_SendData(UART5,0x25);		//从串口2发送结束指令
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
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
	{	USART_SendData(UART5,0xA5);   //从串口2发送开始指令  USART_FLAG_TC： 发送移位寄存器发送完成标志位，全部发送完毕会置 1
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
		USART_SendData(UART5,0x20);		//从串口2发送结束指令
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	}	
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
函数功能：串口5初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
		//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;						//Tx接口为C12
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;							//Rx接口为D2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //初始化串口5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //使能串口5
}
/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART5_IRQHandler(void)
{	
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{
		Res_one = USART_ReceiveData(UART5);
		if(Res_one==0xAA && Res==0)//帧头判断是否为AA55
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

		if(Res==90)//单次传输采样数据不超过90帧
		{
			Res=0;
			Count=Receive_Data.buffer[3];//一次接收的测距采样量
			Receive_Data.Info.start_ang=((Receive_Data.buffer[5]<<7) + (Receive_Data.buffer[4]>>1)) / 64;//采样起始角度
			Receive_Data.Info.end_ang=((Receive_Data.buffer[7]<<7) + (Receive_Data.buffer[6]>>1)) / 64;//采样终点角度
			if(Receive_Data.Info.end_ang<Receive_Data.Info.start_ang) 
				diff = 360-Receive_Data.Info.start_ang+Receive_Data.Info.end_ang;
			else
				diff = Receive_Data.Info.end_ang-Receive_Data.Info.start_ang;
			
			for(int i=12; i<Count*2+10; i+=2)
			{
				ang=(int)(Receive_Data.Info.start_ang + (diff/(Count-1)) * ((i-10)/2));//计算采样中间点角度
				dis=(Receive_Data.buffer[i+1]<<6) + (Receive_Data.buffer[i]>>2);//对应点的距离
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
函数功能：串口5发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
