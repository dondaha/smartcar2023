#include "led.h"

int Led_Count=0; //LED flicker time control //LED��˸ʱ�����
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

static u8 pixelBuffer[Pixel_S1_NUM][24];                     //����
static u8 pixelBuffer2[Pixel_S1_NUM][24];

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
�������ܣ�LED�ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	GPIO_InitTypeDef  GPIO_InitStructure3;
	GPIO_InitTypeDef  GPIO_InitStructure4;
	GPIO_InitTypeDef  GPIO_InitStructure5;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;//LED��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
  GPIO_InitStructure4.GPIO_Pin =  GPIO_Pin_5;//LED��ӦIO��
  GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure4.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure4.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure4);//��ʼ��GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_3;//LED��ӦIO��
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure2);//��ʼ��GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��
  GPIO_InitStructure3.GPIO_Pin =  GPIO_Pin_3;//LED��ӦIO��
  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure3.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure3.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure3);//��ʼ��GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_3);
	
	
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
�������ܣ��������ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //KEY��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOB14
	GPIO_ResetBits(GPIOD,GPIO_Pin_15);
}


/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
�������ܣ�LED����˸����
��ڲ������� 
����  ֵ����
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
			//The status of the LED is reversed. 0 is on and 1 is off
			//LED״̬ȡ����0�ǵ�����1��Ϩ��    
      //LED=~LED;              
      //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
      //LED��˸����ǳ��򵥣���Ƶ�ʾ���Ҫ��ͣ�ʹ�������ʱ����			
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
�������ܣ�LED��˸
��ڲ�������˸ʱ��
�� �� ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
	//A6���������̣�C5�������ƺ�
	//���������̣��������������ֶ��̿�������ͣ��,���Ϻ��̾ͽ���
	if(Encoder_state==0)//��������������
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		delay_ms(2000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		delay_ms(2000);
	}
	else if(Error_Down != 0)//�ٶ��쳣����
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		delay_ms(500);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		delay_ms(500);
	}
	else if(Pause_state==0)//��ͣ״̬��Ƴ���
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_3);
	}
	else if(Control_mode==1)//���������̵Ƴ���
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
	}
	else if(Control_mode==2)//�����������̵�����
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		delay_ms(2000);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		delay_ms(2000);
	}
	else if(Control_mode==3)//�ֶ�����
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
	//A3����Դ���̣�C3����Դ�ƺ�
	//�͵�ƺ죬ƽʱ��,�����̽���
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
** function:  WS2812B SPI DMA���߳�ʼ��
**parameter: void
************************************************************************************************/
/*****************************************
 ˵����
 SPI2��
 ���ţ�ʹ�õ���PB15���ţ���TFTLCD�µ�LCD BL
 ʱ�ӣ���������ͼ��SPI2��APB1(42MHz)��Ƶ����
 ****************************************/
void WS2812b_Configuration(void){
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);  //ʹ��SPI2ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);   //DMA1ʱ��ʹ�� 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;                    //PB15���ù������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //����
  GPIO_Init(GPIOB, &GPIO_InitStructure);                        //��ʼ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);        //PB15����Ϊ SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                      //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                  //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                        //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                        //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                          //NSS�ź���Ӳ����NSS�ܽţ�����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//42M/8=5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                  //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                            //CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);                                   //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI2, ENABLE);                                                //ʹ��SPI����
  
	DMA_DeInit(DMA1_Stream4);

	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}                       //�ȴ�DMA������ 
		
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;                            //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DR;                //DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)pixelBuffer;                 //DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                   //�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = Pixel_S1_NUM * 24;                     //���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          //���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             // ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                     //�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //����ͻ�����δ���
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);                               //��ʼ��DMA Stream
  
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);	         // ʹ��SPI2��DMA���� 	
	DMA_Cmd(DMA1_Stream4, DISABLE);                            //�ر�DMA���� 	
	while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	       //ȷ��DMA���Ա�����  		
	DMA_SetCurrDataCounter(DMA1_Stream4,Pixel_S1_NUM * 24);    //���ݴ�����  
	DMA_Cmd(DMA1_Stream4, ENABLE); 
  
  delay_ms(1);
  RGB_BLACK(Pixel_S1_NUM,0);                                   //RGB RESET
  delay_ms(1);
}

void WS2812b_Configuration2(void){
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOBʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //ʹ��SPI2ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   //DMA1ʱ��ʹ�� 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;                    //PB15���ù������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                  //���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;            //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                  //����
  GPIO_Init(GPIOA, &GPIO_InitStructure);                        //��ʼ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);        //PB15����Ϊ SPI2

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;    //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		                      //����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		                  //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		                        //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	                        //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		                          //NSS�ź���Ӳ����NSS�ܽţ�����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//42M/8=5.25M
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	                  //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	                            //CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);                                   //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE);                                                //ʹ��SPI����
  
	DMA_DeInit(DMA2_Stream3);

	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}                       //�ȴ�DMA������ 
		
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;                            //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;                //DMA�����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)pixelBuffer2;                 //DMA �洢��0��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                   //�洢��������ģʽ
  DMA_InitStructure.DMA_BufferSize = Pixel_S1_NUM2 * 24;                     //���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          //���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                             // ʹ����ͨģʽ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                     //�е����ȼ�
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //����ͻ�����δ���
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);                               //��ʼ��DMA Stream
  
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	         // ʹ��SPI2��DMA���� 	
	DMA_Cmd(DMA2_Stream3, DISABLE);                            //�ر�DMA���� 	
	while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	       //ȷ��DMA���Ա�����  		
	DMA_SetCurrDataCounter(DMA2_Stream3,Pixel_S1_NUM2 * 24);    //���ݴ�����  
	DMA_Cmd(DMA2_Stream3, ENABLE); 
  
  delay_ms(1);
  RGB_BLACK2(Pixel_S1_NUM2,0);                                   //RGB RESET
  delay_ms(1);
}

/***********************************************************************************************
**     name: rgb_SetColor
** function: �趨ĳ��RGB LED����ɫ
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
	
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4) != RESET){ //�ȴ�DMA1_Steam5�������			
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);                //���DMA1_Steam5������ɱ�־����Ԥ��DMA_FLAG_TCIF0���㣬��������Channel		
		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);	         // ʹ��SPI3��DMA���� 	
		DMA_Cmd(DMA1_Stream4, DISABLE);                            //�ر�DMA���� 	
		while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	       //ȷ��DMA���Ա�����  		
		DMA_SetCurrDataCounter(DMA1_Stream4,Pixel_S1_NUM * 24);    //���ݴ�����  
		DMA_Cmd(DMA1_Stream4, ENABLE);                             //����DMA���� 
	}
}

void rgb_SendArray2(void){
	
	if(DMA_GetFlagStatus(DMA2_Stream3,DMA_FLAG_TCIF3) != RESET){ //�ȴ�DMA1_Steam5�������			
		DMA_ClearFlag(DMA2_Stream3,DMA_FLAG_TCIF3);                //���DMA1_Steam5������ɱ�־����Ԥ��DMA_FLAG_TCIF0���㣬��������Channel		
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	         // ʹ��SPI3��DMA���� 	
		DMA_Cmd(DMA2_Stream3, DISABLE);                            //�ر�DMA���� 	
		while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE){}	       //ȷ��DMA���Ա�����  		
		DMA_SetCurrDataCounter(DMA2_Stream3,Pixel_S1_NUM2 * 24);    //���ݴ�����  
		DMA_Cmd(DMA2_Stream3, ENABLE);                             //����DMA���� 
	}
}

/***********************************************************************************************
**     name: RGB_RED
** function: �趨��ɫΪRED
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪPURPLE
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪSKY
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪMAGENTA
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪORANGE
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪGREEN
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪBLUE
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪYELLOW
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪall-off
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪWHITE
**parameter: Pixel_LEN ������
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
** function: �趨��ɫΪAMBER
**parameter: Pixel_LEN ������
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
** function: ����ɫת��ΪGRB
**parameter: WheelPos ��ɫ��ֵ
**   return: RGBColor_TypeDef ��ɫGRB
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
** function: �����ƹ���
**parameter: Pixel_LEN ������
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