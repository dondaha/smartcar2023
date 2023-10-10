#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO			3     //Task priority //任务优先级
#define LED_TASK_PRIO2		3
#define LED_TASK_PRIO3		3
#define LED_STK_SIZE 			128   //Task stack size //任务堆栈大小
#define LED_STK_SIZE2			128
#define LED_STK_SIZE3			128
#define SIG_0  						0XE0
#define SIG_1 						0XF8



/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOD
#define Buzzer_PIN GPIO_Pin_15
#define Buzzer PAout(8)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED_PORT GPIOA
#define LED_PIN1 GPIO_Pin_12
#define LED_PIN2 GPIO_Pin_12
#define LED PAout(12) 
/*----------------------------------*/

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
void Led_Flash2(u16 time);
void led_task2(void *pvParameters);
void led_task3(void *pvParameters);
void Set_pixel_rgb(uint16_t n,u8 color);
extern int Led_Count;
extern int Led_State;
extern uint8_t buzzer_state;

typedef struct
{
  u8 R;
  u8 G;
  u8 B;
} RGBColor_TypeDef;

//extern u8 pixelBuffer[][24];
extern const RGBColor_TypeDef RED ;
extern const RGBColor_TypeDef GREEN;
extern const RGBColor_TypeDef BLUE;
extern const RGBColor_TypeDef SKY;
extern const RGBColor_TypeDef MAGENTA ;
extern const RGBColor_TypeDef YELLOW ;
extern const RGBColor_TypeDef ORANGE;
extern const RGBColor_TypeDef BLACK;
extern const RGBColor_TypeDef WHITE;
extern const RGBColor_TypeDef PURPLE;
/* Exported constants --------------------------------------------------------*/
#define Pixel_S1_NUM 28		//灯珠 RGB数量
#define Pixel_S1_NUM2 28		//灯珠 RGB数量

/**************************************
 说明：

 WS2812B编码协议(单位：ns)：
       min     typ     max
bit 0
 T0H:  220      -      420
 T0L:  750      -      1600
 
bit 1 
 T1H:  750      -      1600
 T1L:  220      -      420
 
 
 RESET: time > 300us


8分频APB1，42MHz/8 = 5.25MHz
时钟周期为：1/5.25/1e6 = 1.90e-7=190ns
**************************************/

#define CODE0 0xC0 // 0码, 发送的时间 1100 0000  根据不同的SCK适当调整
#define CODE1 0xFC // 1码, 发送的时间 1111 1100

void WS2812b_Configuration(void);
void WS2812b_Configuration2(void);

/* Basic Color Effects -------------------------------------------------------*/
void RGB_RED(u16 Pixel_LEN, u16 Start);
void RGB_GREEN(u16 Pixel_LEN, u16 Start);
void RGB_BLUE(u16 Pixel_LEN, u16 Start);
void RGB_YELLOW(u16 Pixel_LEN, u16 Start);
void RGB_MAGENTA(u16 Pixel_LEN, u16 Start);
void RGB_BLACK(u16 Pixel_LEN, u16 Start);
void RGB_WHITE(u16 Pixel_LEN, u16 Start);
void RGB_SKY(u16 Pixel_LEN, u16 Start);
void RGB_ORANGE(u16 Pixel_LEN, u16 Start);
void RGB_PURPLE(u16 Pixel_LEN, u16 Start);
void RGB_AMBER(u16 Pixel_LEN, u16 Start);
/* Complicated Color Effects -------------------------------------------------*/
void rainbowCycle(u16 Pixel_LEN);

/* Basic Color Effects -------------------------------------------------------*/
void RGB_RED2(u16 Pixel_LEN, u16 Start);
void RGB_GREEN2(u16 Pixel_LEN, u16 Start);
void RGB_BLUE2(u16 Pixel_LEN, u16 Start);
void RGB_YELLOW2(u16 Pixel_LEN, u16 Start);
void RGB_MAGENTA2(u16 Pixel_LEN, u16 Start);
void RGB_BLACK2(u16 Pixel_LEN, u16 Start);
void RGB_WHITE2(u16 Pixel_LEN, u16 Start);
void RGB_SKY2(u16 Pixel_LEN, u16 Start);
void RGB_ORANGE2(u16 Pixel_LEN, u16 Start);
void RGB_PURPLE2(u16 Pixel_LEN, u16 Start);
void RGB_AMBER2(u16 Pixel_LEN, u16 Start);
/* Complicated Color Effects -------------------------------------------------*/
void rainbowCycle2(u16 Pixel_LEN);
void RunRGB(void);

#endif
