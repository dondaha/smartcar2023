#include "led.h"

int Led_Count=0; //LED flicker time control //LED闪烁时间控制
uint8_t buzzer_state=0;
// Some Static Colors
const RGBColor_TypeDef RED      = {255,0,0};
const RGBColor_TypeDef GREEN    = {0,255,0};
const RGBColor_TypeDef BLUE     = {0,0,255};
const RGBColor_TypeDef SKY      = {0,255,255};
const RGBColor_TypeDef MAGENTA  = {255,0,255};
const RGBColor_TypeDef YELLOW   = {255,255,0};
const RGBColor_TypeDef ORANGE   = {127,106,0};
const RGBColor_TypeDef BLACK    = {0,0,0};
const RGBColor_TypeDef WHITE    = {255,255,255};
const RGBColor_TypeDef PURPLE   = {65,105,225};
const RGBColor_TypeDef AMBER   = {255,191,0};

static u8 pixelBuffer[Pixel_S1_NUM][24];                     //灯珠
static u8 pixelBuffer2[Pixel_S1_NUM][24];

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	GPIO_InitTypeDef  GPIO_InitStructure3;
	GPIO_InitTypeDef  GPIO_InitStructure4;
	GPIO_InitTypeDef  GPIO_InitStructure5;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
  GPIO_InitStructure4.GPIO_Pin =  GPIO_Pin_5;//LED对应IO口
  GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure4.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure4.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure4);//初始化GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_3;//LED对应IO口
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure2);//初始化GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟
  GPIO_InitStructure3.GPIO_Pin =  GPIO_Pin_3;//LED对应IO口
  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure3.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure3.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure3);//初始化GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_3);
	
	
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
函数功能：蜂鸣器接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //KEY对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOB14
	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
}


/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无 
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED状态取反，0是点亮，1是熄灭    
      //LED=~LED;              
      //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
      //LED闪烁任务非常简单，对频率精度要求低，使用相对延时函数			
      //vTaskDelay(Led_Count); 
			Led_Flash(100);
    }
}

void led_task2(void *pvParameters)
{
    while(1)
    { 
			Led_Flash2(100);
    }
}

void led_task3(void *pvParameters)
{
    while(1)
    { 
			buzzer_state = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15);
			RunRGB();
			delay_ms(1);
    }
}


/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	//A6：导航灯绿，C5：导航灯红
	//自主导航绿，半自主慢闪，手动绿快闪，急停红,故障红绿就交替
	if(Encoder_state==0)//编码器断连慢闪
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		delay_ms(2000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		delay_ms(2000);
	}
	else if(Error_Down != 0)//速度异常快闪
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		delay_ms(500);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		delay_ms(500);
	}
	else if(Pause_state==0)//急停状态红灯常亮
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	}
	else if(Control_mode==1)//自主导航绿灯常亮
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
	}
	else if(Control_mode==2)//半自主导航绿灯慢闪
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		delay_ms(2000);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		delay_ms(2000);
	}
	else if(Control_mode==3)//手动导航
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		delay_ms(500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		delay_ms(500);
	}
}

void Led_Flash2(u16 time)
{
	//A3：电源灯绿，C3：电源灯红
	//低电灯红，平时绿,充电红绿交替
	if(Current>0)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		delay_ms(1000);
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
		delay_ms(1000);
	}
	else if(Electricity<=0.2 && Electricity >=0)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
	}
	else
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	}

}

/***********************************************************************************************
**     name: WS2812b_Configuration
** function:  WS2812B SPI DMA总线初始化
**parameter: void
************************************************************************************************/
/*****************************************
 说明：
 SPI2：
 引脚：使用的是PB15引脚，在TFTLCD下的LCD BL
 时钟：根据总线图，SPI2由APB1(42MHz)分频而来
 ****************************************/
void WS2812b_Configuration(void){
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  //使能SPI2时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1时钟使能 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                    //PB15复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);                        //初始化
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);        //PB15复用为 SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                      //设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                  //设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                        //串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                        //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                          //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//42M/8=5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                  //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                            //CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);                                   //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI2, ENABLE);                                                //使能SPI外设
  
	DMA_DeInit(DMA1_Stream4);

	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}                       //等待DMA可配置 
		
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;                            //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DR;                //DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)pixelBuffer;                 //DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                   //存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = Pixel_S1_NUM * 24;                     //数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          //外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             // 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                     //中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //外设突发单次传输
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);                               //初始化DMA Stream
  
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);	         // 使能SPI2的DMA发送 	
	DMA_Cmd(DMA1_Stream4, DISABLE);                            //关闭DMA传输 	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	       //确保DMA可以被设置  		
	DMA_SetCurrDataCounter(DMA1_Stream4,Pixel_S1_NUM * 24);    //数据传输量  
	DMA_Cmd(DMA1_Stream4, ENABLE); 
  
  delay_ms(1);
  RGB_BLACK(Pixel_S1_NUM,0);                                   //RGB RESET
  delay_ms(1);
}

void WS2812b_Configuration2(void){
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOB时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //使能SPI2时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   //DMA1时钟使能 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                    //PB15复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);                        //初始化
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);        //PB15复用为 SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                      //设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                  //设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                        //串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                        //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                          //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//42M/8=5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                  //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                            //CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);                                   //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE);                                                //使能SPI外设
  
	DMA_DeInit(DMA2_Stream3);

	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}                       //等待DMA可配置 
		
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;                            //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;                //DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)pixelBuffer2;                 //DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                   //存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = Pixel_S1_NUM2 * 24;                     //数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          //外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             // 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                     //中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //外设突发单次传输
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);                               //初始化DMA Stream
  
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	         // 使能SPI2的DMA发送 	
	DMA_Cmd(DMA2_Stream3, DISABLE);                            //关闭DMA传输 	
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	       //确保DMA可以被设置  		
	DMA_SetCurrDataCounter(DMA2_Stream3,Pixel_S1_NUM2 * 24);    //数据传输量  
	DMA_Cmd(DMA2_Stream3, ENABLE); 
  
  delay_ms(1);
  RGB_BLACK2(Pixel_S1_NUM2,0);                                   //RGB RESET
  delay_ms(1);
}

/***********************************************************************************************
**     name: rgb_SetColor
** function: 设定某个RGB LED的颜色
**parameter: void
**   return: void
************************************************************************************************/
void rgb_SetColor(u16 LedId, RGBColor_TypeDef Color){
	
 	u16 i;
	
  if( LedId > ( Pixel_S1_NUM ) ){
    printf("Error:Out of Range!\r\n");
		return;                               //to avoid overflow
	}
  
  for(i=0;i<=7;i++){
		pixelBuffer[LedId][i]= ( (Color.G & (1 << (7 -i)) )? (CODE1):CODE0 );
	}
  for(i=8;i<=15;i++){
		pixelBuffer[LedId][i]= ( (Color.R & (1 << (15-i)) )? (CODE1):CODE0 );
	}
  for(i=16;i<=23;i++){
		pixelBuffer[LedId][i]= ( (Color.B & (1 << (23-i)) )? (CODE1):CODE0 );
	}
}

void rgb_SetColor2(u16 LedId, RGBColor_TypeDef Color){
	
 	u16 i;
	
  if( LedId > ( Pixel_S1_NUM ) ){
    printf("Error:Out of Range!\r\n");
		return;                               //to avoid overflow
	}
  
  for(i=0;i<=7;i++){
		pixelBuffer2[LedId][i]= ( (Color.G & (1 << (7 -i)) )? (CODE1):CODE0 );
	}
  for(i=8;i<=15;i++){
		pixelBuffer2[LedId][i]= ( (Color.R & (1 << (15-i)) )? (CODE1):CODE0 );
	}
  for(i=16;i<=23;i++){
		pixelBuffer2[LedId][i]= ( (Color.B & (1 << (23-i)) )? (CODE1):CODE0 );
	}
}

/***********************************************************************************************
**     name: rgb_SendArray
** function: Configure colors to RGB pixel series.
             RGBColor_TypeDef: pointer to a RGBColor_TypeDef structure that contains the color configuration information for the RGB pixel.
**parameter: void
**   return: void
************************************************************************************************/
void rgb_SendArray(void){
	
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4) != RESET){ //等待DMA1_Steam5传输完成			
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);                //清除DMA1_Steam5传输完成标志，先预想DMA_FLAG_TCIF0的零，代表的是Channel		
		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);	         // 使能SPI3的DMA发送 	
		DMA_Cmd(DMA1_Stream4, DISABLE);                            //关闭DMA传输 	
		while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	       //确保DMA可以被设置  		
		DMA_SetCurrDataCounter(DMA1_Stream4,Pixel_S1_NUM * 24);    //数据传输量  
		DMA_Cmd(DMA1_Stream4, ENABLE);                             //开启DMA传输 
	}
}

void rgb_SendArray2(void){
	
	if(DMA_GetFlagStatus(DMA2_Stream3,DMA_FLAG_TCIF3) != RESET){ //等待DMA1_Steam5传输完成			
		DMA_ClearFlag(DMA2_Stream3,DMA_FLAG_TCIF3);                //清除DMA1_Steam5传输完成标志，先预想DMA_FLAG_TCIF0的零，代表的是Channel		
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	         // 使能SPI3的DMA发送 	
		DMA_Cmd(DMA2_Stream3, DISABLE);                            //关闭DMA传输 	
		while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	       //确保DMA可以被设置  		
		DMA_SetCurrDataCounter(DMA2_Stream3,Pixel_S1_NUM2 * 24);    //数据传输量  
		DMA_Cmd(DMA2_Stream3, ENABLE);                             //开启DMA传输 
	}
}

/***********************************************************************************************
**     name: RGB_RED
** function: 设定颜色为RED
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_RED(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor(i,RED);
	}
  
  rgb_SendArray();
}

void RGB_RED2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor2(i,RED);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_PURPLE
** function: 设定颜色为PURPLE
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_PURPLE(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor(i,PURPLE);
	}
  
  rgb_SendArray();
}

void RGB_PURPLE2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor2(i,PURPLE);
	}
  
  rgb_SendArray2();
}
/***********************************************************************************************
**     name: RGB_SKY
** function: 设定颜色为SKY
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_SKY(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor(i,SKY);
	}
  
  rgb_SendArray();
}

void RGB_SKY2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor2(i,SKY);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_MAGENTA
** function: 设定颜色为MAGENTA
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_MAGENTA(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor(i,MAGENTA);
	}
  
  rgb_SendArray();
}

void RGB_MAGENTA2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor2(i,MAGENTA);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_ORANGE
** function: 设定颜色为ORANGE
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_ORANGE(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor(i,ORANGE);
	} 
	
  rgb_SendArray();
}

void RGB_ORANGE2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){  
    rgb_SetColor2(i,ORANGE);
	} 
	
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_GREEN
** function: 设定颜色为GREEN
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_GREEN(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor(i,GREEN);
	}
  
  rgb_SendArray();
}

void RGB_GREEN2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor2(i,GREEN);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_BLUE
** function: 设定颜色为BLUE
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_BLUE(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor(i,BLUE);
	}
  
  rgb_SendArray();
}

void RGB_BLUE2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor2(i,BLUE);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_YELLOW
** function: 设定颜色为YELLOW
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_YELLOW(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor(i,YELLOW);
	}
  
  rgb_SendArray();
}

void RGB_YELLOW2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor2(i,YELLOW);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_BLACK
** function: 设定颜色为all-off
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_BLACK(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
  
    rgb_SetColor(i,BLACK);
	}
  
  rgb_SendArray();
}

void RGB_BLACK2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
  
    rgb_SetColor2(i,BLACK);
	}
  
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_WHITE
** function: 设定颜色为WHITE
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_WHITE(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor(i,WHITE);
    
	}
  rgb_SendArray();
}

void RGB_WHITE2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor2(i,WHITE);
    
	}
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: RGB_AMBER
** function: 设定颜色为AMBER
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void RGB_AMBER(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor(i,AMBER);
    
	}
  rgb_SendArray();
}

void RGB_AMBER2(u16 Pixel_LEN, u16 Start){
	
  u16 i;
	
  for(i = Start; i < Pixel_LEN; i++){
    rgb_SetColor2(i,AMBER);
    
	}
  rgb_SendArray2();
}

/***********************************************************************************************
**     name: Colourful_Wheel
** function: 将颜色转换为GRB
**parameter: WheelPos 颜色数值
**   return: RGBColor_TypeDef 颜色GRB
************************************************************************************************/
RGBColor_TypeDef Colourful_Wheel(u8 WheelPos){
	
	RGBColor_TypeDef color;
  WheelPos = 255 - WheelPos;
  
  if(WheelPos < 85){
    color.R = 255 - WheelPos * 3;
    color.G = 0;
    color.B = WheelPos * 3;
		return color;
  }
  if(WheelPos < 170){
    WheelPos -= 85;
    color.R = 0;
    color.G = WheelPos * 3;
    color.B = 255 - WheelPos * 3;
		return color;
  }
  
  WheelPos -= 170;
  color.R = WheelPos * 3; 
  color.G = 255 - WheelPos * 3;
  color.B = 0;
  
  return color;  
}

/***********************************************************************************************
**     name: rainbowCycle
** function: 呼吸灯功能
**parameter: Pixel_LEN 灯珠数
**   return: void
************************************************************************************************/
void rainbowCycle(u16 Pixel_LEN){
	
  u16 i, j = 0;

  for(j = 0; j < 1023; j++){                                                   // 1 cycles of all colors on wheel
    for(i = 10; i < Pixel_LEN; i++){  
      rgb_SetColor(i,Colourful_Wheel(((i * 256 / Pixel_LEN) + j)&255));
		} 
    rgb_SendArray();
    delay_ms(20);
  }
}

void rainbowCycle2(u16 Pixel_LEN){
	
  u16 i, j = 0;

  for(j = 0; j < 1023; j++){                                                   // 1 cycles of all colors on wheel
    for(i = 0; i < Pixel_LEN; i++){  
      rgb_SetColor2(i,Colourful_Wheel(((i * 256 / Pixel_LEN) + j)&255));
		} 
    rgb_SendArray2();
    delay_ms(20);
  }
}

void RunRGB(void){
	if(Led_Count<=1000)
		Led_Count++;
	else
		Led_Count=0;
	
	if(Error_Down!=0)
	{
		if(Led_Count<=500)
		{
			RGB_AMBER(28,0);
		}
		else
		{
			RGB_BLACK(28,0);
		}
	}
	else
	{
		GPIO_ResetBits(GPIOD,GPIO_Pin_15);
		
		if(cur_V_Speed==0 && cur_W_Speed==0 && GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11))
		{
			RGB_RED2(23,6);
		}
		else if(cur_V_Speed>=0)
		{
			RGB_SKY2(23,6);
			if(cur_W_Speed<-0.1)
			{
				//RGB_SKY(18,6);
				if(Led_Count<=500)
				{
					RGB_AMBER(5,0);
				}
				else
				{
					RGB_BLACK(5,0);
				}
			}
			else if(cur_W_Speed>0.1)
			{
				//RGB_SKY(18,6);
				if(Led_Count<=500)
				{
					RGB_AMBER(28,24);
				}
				else
				{
					RGB_BLACK(28,24);
				}
			}
			else
			{
				RGB_BLACK(28,24);
				RGB_SKY(23,6);
				RGB_BLACK(5,0);
			}
		}
		else if(cur_V_Speed<0)
		{
			if(cur_W_Speed<-0.1)
			{
				if(Led_Count<=500)
				{
					RGB_AMBER2(5,0);
				}
				else
				{
					RGB_BLACK2(5,0);
				}
			}
			else if(cur_W_Speed>0.1)
			{
				//RGB_SKY(18,6);
				if(Led_Count<=500)
				{
					RGB_AMBER2(28,24);
				}
				else
				{
					RGB_BLACK2(28,24);
				}
			}
			else
			{
				RGB_BLACK2(28,24);
				RGB_WHITE2(23,6);
				RGB_BLACK2(5,0);
			}
		}
		else
		{
			RGB_SKY(23,6);
			RGB_SKY2(23,6);
		}
	}
}