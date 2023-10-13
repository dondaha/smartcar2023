//#include "system.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "usart.h"
#include "usartx.h"

u8 scan[]={0xA5,0x20}; //开始扫描采样(SCAN)命令请求
u8 stop[]={0xA5,0x25}; //停止扫描命令
u8 distan;
u16 result;
float mindis=0;
float minang=0;
float maxdis=0;
float maxang=0;
float ldis=0;
float rdis=0;
u8 Right_count,Left_count;
float A1,A2;//左右侧赛道边对应的航向角
int servo_PWM0=1500;
float x1,y1,x2,y2,w1,w2;//各采集点xy坐标，及与x轴的夹角
float x3,y3,x4,y4,w3,w4;
int obstacle_ang=0;//障碍物所在角度
int obstacle_dis=9999;//障碍物距离

float avoid_speed = 0.0; // 避障时小车的速度
float navigate_speed = 0.0; // 导航时小车的速度

//控制电机旋转方向的端口初始化
void Motor_Direct_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;//IN1对应IO口，A6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_5;//IN2对应IO口，C5
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure2);//初始化GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
}
//控制电机转速及舵机角度的端口初始化
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
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;//电机转速控制用PB6,舵机角度控制用PB7
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
//速度控制
//输入值：车速speed，-0.8~0.8m/s
//输出值：控制电机的PWM值
void Speed_Control(float speed)
{
	if(speed>0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		TIM4->CCR1 = fabs(speed)/0.8*5000;
	}
	else if(speed<0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_5);
		TIM4->CCR1 = fabs(speed)/0.8*5000;
	}
	else
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);
		TIM4->CCR1 = 0;
	}
}
//转向程序
//输入值：前轮偏向角angle，范围-12°~12°，正值右转
//输出值：控制舵机的PWM值
float angle_0 = 0.0;
void Servo_Control(float angle)
{
	angle_0 = angle;
	if (angle_0>12) angle_0=12;
	if (angle_0<-12) angle_0=-12;
	TIM4->CCR2 = 1430 - angle_0/12*200;
}
//避障程序
//根据障碍物在雷达的左右半侧方位，设置小车的转向方向
//同时小车减速，以便更精确的控制
void Avoiding(void)
{
	Speed_Control(avoid_speed);
	if(obstacle_ang<180)
		Servo_Control(12);
	else if(obstacle_ang>=180)
		Servo_Control(-12);
}
//导航程序
//舵机转向参照，判断延赛道左边走还是右边走
void Nagetive(void)
{
	A1=atan((y2-y1)/(x2-x1));
	A2=atan((y4-y3)/(x4-x3));
	Speed_Control(navigate_speed);
	if(ldis<rdis)
	{
		Servo_Control(A1);
	}
	else if(ldis>rdis)
	{
		Servo_Control(A2);
	}
}

// 计算整数数组的中位数
// Compare function for use with qsort
int compare(const void *a, const void *b) {
    return (*(int*)a - *(int*)b);
}

// Calculate the median of an integer array
int calculateMedian(int arr[], int n) {
    // Sort the array in ascending order using qsort
    qsort(arr, n, sizeof(int), compare);

    // If n is odd, return the middle element
    if (n % 2 == 1) {
        return arr[n / 2];
    }
    // If n is even, return the average of the two middle elements
    else {
        int mid1 = arr[(n - 1) / 2];
        int mid2 = arr[n / 2];
        return (mid1 + mid2) / 2;
    }
}

//对雷达扫描数据进行分析，小车正前方对应雷达的180°
void Lidar(void)
{
	obstacle_ang=0;
	for(int i = 0; i<360; i++)
	{
		int dis = diss[i];
		if(i>90 && i<135 && dis!=0 && dis <1000)//检测左侧赛道边情况
		{
			if(minang==0)
			{
				mindis=dis;
				ldis=dis;//取得雷达左侧与赛道边间的最近距离
				minang=i;
				w1=180-minang;
				x1=mindis*cos(w1*PI/180);
				y1=mindis*sin(w1*PI/180);//取得左侧赛道边近点坐标
			}
			if(i>maxang)
			{
				maxdis=dis;
				maxang=i;
				w2=180-maxang;
				x2=maxdis*cos(w2*PI/180);
				y2=maxdis*sin(w2*PI/180);//取得左侧赛道边远点坐标
			}
		}
		else if (i>225 && i<270 && dis!=0 && dis <1000)//检测右侧赛道边情况
		{
			if(minang<225)
			{
				mindis=dis;
				minang=i;
				w4=180-minang;
				x4=mindis*cos(w4*PI/180);
				y4=mindis*sin(w4*PI/180);//取得右侧赛道边近点坐标
			}
			if(i>maxang)
			{
				maxdis=dis;
				rdis=dis;//取得雷达右侧与赛道边间的最近距离
				maxang=i;
				w3=180-maxang;
				x3=maxdis*cos(w3*PI/180);
				y3=maxdis*sin(w3*PI/180);//取得右侧赛道边远点坐标
			}
		}
		else if(i>135 && i<225 && dis!=0 && dis<500)//检测前方是否有障碍物
		{
			if(obstacle_dis>dis)
			{
				obstacle_dis=dis;//找到障碍物离雷达最近的点的距离和角度
				obstacle_ang=i;
			}
		}
		else if(i==359)
		{
			mindis=0;
			minang=0;
			maxdis=0;
			maxang=0;
		}
	}
	//当发现障碍物时，优先躲避障碍物，否则执行导航
	if(obstacle_ang!=0)
	{
		Avoiding();
	}
	else
	{
		Nagetive();
	}
}

void lidar_new(){
	// 计算左前方和右前方最近的障碍物距离
	int left_front_dis = 9999;
	int right_front_dis = 9999;
	for(int i = 0; i<360; i++)
	{
		int dis = diss[i];
		if(i>90 && i<135 && dis!=0 && dis <1000)//检测左侧赛道边情况
		{
			if(dis < left_front_dis){
				left_front_dis = dis;
			}
		}
		else if (i>225 && i<270 && dis!=0 && dis <1000)//检测右侧赛道边情况
		{
			if(dis < right_front_dis){
				right_front_dis = dis;
			}
		}
	}
	if (left_front_dis < 300 || right_front_dis < 300){
		Speed_Control(0.4);
		// 如果左侧障碍物距离小于右侧障碍物距离，那么小车向右转
		if(left_front_dis < right_front_dis){
			Servo_Control(12);
		}
		// 如果右侧障碍物距离小于左侧障碍物距离，那么小车向左转
		else if(left_front_dis > right_front_dis){
			Servo_Control(-12);
		}
		return;
	}else{
		Speed_Control(0.5);
		Servo_Control(0);
	}	
}


void lidar_new1(){
	// 判断前方左右30度内是否有障碍物
	// 计算diss[150]到diss[210]内过近的点的数量
	int sum_l, sum_r = 0;
	int front_count_l, front_count_r = 0; // 合法点的数量
	int front_dis_l, front_dis_r = 0; // 近距离点的数量
	for(int i = 0; i<=30; i++)
	{
		int dis_l = diss[180-i];
		int dis_r = diss[180+i];
		if (dis_l!=0&&dis_l<600) {++front_dis_l;++front_count_l;sum_l+=dis_l;}else if (dis_l>=600) {++front_count_l;sum_l+=dis_l;}
		if (dis_r!=0&&dis_r<600) {++front_dis_r;++front_count_r;sum_r+=dis_r;}else if (dis_r>=600) {++front_count_r;sum_r+=dis_r;}
	}
	if (front_count_l == 0||front_count_r == 0) {Servo_Control(0);return;} // 数据不对就退出
	float front_avg_dis_l = sum_l / front_count_l; // 前方左侧障碍物的平均距离
	float front_avg_dis_r = sum_r / front_count_r; // 前方右侧障碍物的平均距离
	// 前方有障碍物就开始避障
	if (front_dis_l > 6 || front_dis_r > 6){
		// 如果左侧障碍物距离小于右侧障碍物距离，那么小车向右转
		float target = (front_avg_dis_l - front_avg_dis_r) / (front_avg_dis_l + front_avg_dis_r);
		float kp = 12.0;
		float output = kp * target;
		if (output > 12) output = 12;
		if (output < -12) output = -12;
		Servo_Control(output);
	}else{
		Servo_Control(0);
	}
}

void lidar_new2(){
	// 计算左前方和右前方最近的点的距离
	int left_front_dis = 9999;
	int right_front_dis = 9999;
	int front_dis = 9999;
	for(int i=0; i<90; ++i){
		int dis_l = diss[180-i];
		int dis_r = diss[180+i];
		if (dis_l!=0 && dis_l <2000 && dis_l<left_front_dis)//检测前方赛道边情况
		{
			left_front_dis = dis_l;
		}
		if (dis_r!=0 && dis_r <2000 && dis_r<right_front_dis)//检测前方赛道边情况
		{
			right_front_dis = dis_r;
		}
	}
	// 计算赛道前方最近的点
	for(int i=150; i<=210; ++i){
		int dis = diss[i];
		if (dis!=0 && dis <2000 && dis<front_dis)//检测前方赛道边情况
		{
			front_dis = dis;
		}
	}
	if (front_dis < 600){
		Speed_Control(0.6);
		// // 如果左侧障碍物距离小于右侧障碍物距离，那么小车向右转
		// if(left_front_dis < right_front_dis){
		// 	Servo_Control(12);
		// }
		// // 如果右侧障碍物距离小于左侧障碍物距离，那么小车向左转
		// else if(left_front_dis > right_front_dis){
		// 	Servo_Control(-12);
		// }
		// else{
		// 	Servo_Control(0);
		// }
		if (left_front_dis > 1000) left_front_dis = 1000;
		if (right_front_dis > 1000) right_front_dis = 1000;
		float output = 100.0 * (right_front_dis - left_front_dis) / (left_front_dis + right_front_dis);
		Servo_Control(output);
	}else{
		Speed_Control(0.8);
		Servo_Control(0);
	}
}

void lidar_new3(){
	int dis_min[12] = {9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};
	// 180~210 对应dis_min[6]
	// 150~180 对应dis_min[5]
	int dis;
	for (int i=0; i<360; ++i){
		dis = diss[i];
		if (dis!=0 && dis<dis_min[i/30]){
			dis_min[i/30] = dis;
		}
	}
	// 前方有障碍物就开始减速
	if (dis_min[5] < 800 || dis_min[6] < 800) Speed_Control(0.6);
	else Speed_Control(0.8);
	if (dis_min[5] < 600 && dis_min[5] > 400 || dis_min[6] < 600 && dis_min[6] > 400){
		Speed_Control(0.6);
		// if (dis_min[4]<400 && dis_min[7]<400){
		// 	Servo_Control(0);
		// }
		// else if (dis_min[4]<400 && dis_min[7]>=400){
		// 	Servo_Control(12);
		// }
		// else if (dis_min[4]>=400 && dis_min[7]<400){
		// 	Servo_Control(-12);
		// }
		// else if (dis_min[4]>dis_min[7]) Servo_Control(12);
		// else if (dis_min[4]<dis_min[7]) Servo_Control(-12);
		// else Servo_Control(0);
		if (dis_min[4]>dis_min[7]) Servo_Control(10);
		else if (dis_min[4]<dis_min[7]) Servo_Control(-10);
		else Servo_Control(0);
	}
	else if (dis_min[5] <= 400 || dis_min[6] <=400){
		Speed_Control(0.5);
		if (dis_min[3]<dis_min[8]) Servo_Control(12);
		else if (dis_min[3]>dis_min[8]) Servo_Control(-12);
		else Servo_Control(0);
	}
	else{
		Speed_Control(0.8);
		Servo_Control(0);
	}
}

void lidar_new4(){
	// 前往最远的一个方向
	int dis_max[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	int dis_min[12] = {9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};
	for (int i=0; i<360; ++i){
		int dis = diss[i];
		if (dis<2000 && dis>dis_max[i/30]){
			dis_max[i/30] = dis;
		}
		if (dis!=0 && dis<dis_min[i/30]){
			dis_min[i/30] = dis;
		}
	}
	// 如果前方有障碍物，那么小车减速，并前往最远的方向
	if (dis_min[5] < 800 || dis_min[6] < 800) Speed_Control(0.6);
	else Speed_Control(0.8);

	if (dis_min[5]<600 || dis_min[6]<600){
		Speed_Control(0.6);
		// 求出左前方最远的距离和右前方最远的距离
		int dis_max_l, dis_max_r = 0;
		for (int i=3; i<=4; ++i){
			if (dis_max[i]>dis_max_l) dis_max_l = dis_max[i];
		}
		for (int i=7; i<=8; ++i){
			if (dis_max[i]>dis_max_r) dis_max_r = dis_max[i];
		}
		// 如果左侧障碍物距离小于右侧障碍物距离，那么小车向右转
		if (dis_max_l < dis_max_r) Servo_Control(12);
		// 如果右侧障碍物距离小于左侧障碍物距离，那么小车向左转
		else if (dis_max_l > dis_max_r) Servo_Control(-12);
		else Servo_Control(0);
	}
	else{
		Speed_Control(0.8);
		Servo_Control(0);
	}
}

int avg_dis(int dis[], int n){
	int sum = 0;
	int count = 0;
	for (int i=0; i<n; ++i){
		if (dis[i]!=0){
			if (dis[i]<2000) sum += dis[i];
			else sum += 2000;
			++count;
		}
	}
	return sum / count;
}

int get_max_index(int dis[], int n){
	int max_index = 0;
	for (int i=0; i<n; ++i){
		if (dis[i]>dis[max_index]) max_index = i;
	}
	return max_index;
}

#define COUNT 20

void lidar_new5(){
	// new4改进版，细化分区，计算平均距离
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算每个分区的平均距离
	for (int i=COUNT/4; i<=COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// 计算前方张角30度内的最短距离
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// 如果前方有障碍物，那么小车减速，并前往最远的方向
	if (dis_min_front < 800) Speed_Control(0.6);
	else Speed_Control(0.8);
	
	if (dis_min_front<700){
		int max_dis_index = get_max_index(dis, COUNT);
		// 如果最远的分区在左侧，那么小车向左转，否则右转
		if (max_dis_index < COUNT/2-1) Servo_Control(-12);
		else if (max_dis_index > COUNT/2) Servo_Control(12);
		else Servo_Control(0);
	}else{
		Servo_Control(0);
	}
}

void lidar_new6(){
	// new5的改进版本 Create at 2023.10.8
	// 目前效果不好,不如new5
	// 调整方向使得前方总是最远平均距离区块，同时增加条件，在左右有障碍物的时候不转向
	// 如果最远区块在前方的角度范围内，不转向或者小角度转向，防止频繁转向降速
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算每个分区的平均距离
	for (int i=COUNT/4; i<COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// 计算前方张角30度内的最短距离
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// 如果前方有障碍物，那么小车减速
	if (dis_min_front < 800) Speed_Control(0.6);
	else Speed_Control(0.8);
	// 前往最远的区块
	int max_dis_index = get_max_index(dis, COUNT);
	if (max_dis_index < COUNT/2-1) Servo_Control(-12);
	else if (max_dis_index > COUNT/2) Servo_Control(12);
	else Servo_Control(0);
}

void lidar_new7(int direction){
	// pid调节版本 // direction为0代表逆时针，1代表顺时针
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算前方张角30度内的最短距离
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// 去除了速度调整，任务交给了lidar_new8
	// 计算小车左侧30度范围内的平均距离
	int dis_left_front;
	int dis_left_back;
	int dis_left;
	if (direction==0){
		dis_left_front = avg_dis(diss+100, 20);
		dis_left_back = avg_dis(diss+60, 20);
		dis_left = avg_dis(diss+80, 20);
	}else{
		dis_left_front = avg_dis(diss+240, 20);
		dis_left_back = avg_dis(diss+280, 20);
		dis_left = avg_dis(diss+260, 20);
	}
	// int target = 0;
	float kp_1 = 0.1; // 调成0试试
	float kp_2 = 0.24;
	float error_1 = dis_left_back - dis_left_front;
	// error_1绝对值限定范围
	if (error_1 > 120) error_1 = 120;
	if (error_1 < -120) error_1 = -120;
	float error_2 = 260-dis_left; // 【超级重要】的参数，250会经常撞向左侧障碍物，300明显好很多
	// error_2绝对值限定范围
	if (error_2 > 120) error_2 = 120;
	if (error_2 < -120) error_2 = -120;
	float error =  kp_1 * error_1 + kp_2 * error_2;
	float output = error*(direction==0?1:-1); // direction为0代表逆时针，1代表顺时针，增加了一个对称的负号
	Servo_Control(output);
}

void avoid_obstacle(){
	// 避免左右有障碍物的时候还往障碍物上撞
	int dis_min = 250; //临界距离，可以调整
	int dis_l = avg_dis(diss+100, 20);
	int dis_r = avg_dis(diss+240, 20);
	if (dis_l < dis_min && angle_0<0) Servo_Control(0);
	if (dis_r < dis_min && angle_0>0) Servo_Control(0);
}

void lidar_new8(){
	// new5 new7结合版
	// 效果还可以，目前的最优解 Update at 2023.10.9 
	int direction = 0; //【0：逆时针；1：顺时针】
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算每个分区的平均距离
	for (int i=COUNT/4; i<COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// 计算前方张角30度内的最短距离
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// 如果前方有障碍物，那么小车减速，并前往最远的方向
	if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(0.6);
	else if (dis_min_front < 500) Speed_Control(0.5);
	else Speed_Control(0.8);
	
	if (dis_min_front<=900){ // 这个阈值可以用来调整小车何时调整大方向而不是调整与障碍物的距离
		int max_dis_index = get_max_index(dis, COUNT);
		// // 如果最远的分区在左侧，那么小车向左转，否则右转
		if (max_dis_index < COUNT/2-1) Servo_Control(-12);
		else if (max_dis_index > COUNT/2) Servo_Control(12);
		else lidar_new7(direction); // 这句还挺重要，大大提升稳定性，在方向无误的时候贴近墙壁
	}else{
		lidar_new7(direction);
	} // TO DO 增加贴着右边赛道的版本
	avoid_obstacle();
}



double error_last = 0;

void run(double kp, double kd, int max_dis)
{
	// lyh的idea 目前的最优算法 update at 2023.10.12
	int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	// 计算前方张角30度内的最短距离
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// 如果前方有障碍物，那么小车减速
	if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(0.8);
	else if (dis_min_front < 500) Speed_Control(0.8);
	else Speed_Control(0.8);

	// 计算前方赛道中心点的方位，使用pid逼近那个点
    int count = 0;
    int index1 = 0, dis1 = 0; // 临时变量
    int index2 = 0, dis2 = 0;
	double idx_1 = 0, idx_2 = 0; // 记录的最远的两个点的索引
	double dis_1 = 0, dis_2 = 0;

    // int idx_max = 0;
    double d_max = 0;

    for (int i = 90; i < 270; i++)
    {
        int dis = diss[i];
        if (dis > 0 && dis < max_dis)
        {
            count++;
            index1 = index2;
            dis1 = dis2;

            index2 = i;
			dis2 = dis;

            if (count > 1)
            {
				// 此处的距离计算函数错误，但效果很好（？）
                double d = (double)(index2-index1) * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                // 纠正回来了，但是效果一坨，不如上面的
				// double d = (double)(index2-index1)* PI / 180 * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                if (d > d_max)
                {
					idx_1 = index1;
					idx_2 = index2;
					dis_1 = dis1;
					dis_2 = dis2;
                    d_max = d;
                }
            }
        }
    }
	double oppsite_side = sqrt(pow(dis_1, 2) + pow(dis_2, 2) - 2 * dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180));
	double mid_line = sqrt((pow(dis_1, 2)+pow(dis_2, 2))/2-pow(oppsite_side, 2)/4);
	double offset_angle = acos((pow(dis_1, 2)+pow(mid_line, 2)-pow(oppsite_side/2, 2))/(2*dis_1*mid_line));
	double target_angle = idx_1 + offset_angle * 180 / PI;

    double error = target_angle - 180;  // 逆时针转的时候error一般小于0，顺时针大于0
    Servo_Control(kp * error + kd * (error - error_last));
	avoid_obstacle();
    error_last = error;
}

double error_last_reverse = 0;

void run_reverse(double kp, double kd, int max_dis)
{
	// lyh的idea 目前的最优算法 update at 2023.10.12
	// int dis_min_front = 9999; // 记录前方张角30度内的最短距离
	Speed_Control(-0.8);

	// 计算前方赛道中心点的方位，使用pid逼近那个点
    int count = 0;
    int index1 = 0, dis1 = 0; // 临时变量
    int index2 = 0, dis2 = 0;
	double idx_1 = 0, idx_2 = 0; // 记录的最远的两个点的索引
	double dis_1 = 0, dis_2 = 0;

    // int idx_max = 0;
    double d_max = 0;

    for (int i = 270; i < 270+180; i++)
    {
        int dis = diss[i%360];
        if (dis > 0 && dis < max_dis)
        {
            count++;
            index1 = index2;
            dis1 = dis2;

            index2 = i;
			dis2 = dis;

            if (count > 1)
            {
				// 此处的距离计算函数错误，但效果很好（？）
                double d = (double)(index2-index1) * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                // 纠正回来了，但是效果一坨，不如上面的
				// double d = (double)(index2-index1)* PI / 180 * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                if (d > d_max)
                {
					idx_1 = index1;
					idx_2 = index2;
					dis_1 = dis1;
					dis_2 = dis2;
                    d_max = d;
                }
            }
        }
    }
	double oppsite_side = sqrt(pow(dis_1, 2) + pow(dis_2, 2) - 2 * dis_1 * dis_2 * cos((idx_2 - idx_1) * PI / 180));
	double mid_line = sqrt((pow(dis_1, 2)+pow(dis_2, 2))/2-pow(oppsite_side, 2)/4);
	double offset_angle = acos((pow(dis_1, 2)+pow(mid_line, 2)-pow(oppsite_side/2, 2))/(2*dis_1*mid_line));
	double target_angle = idx_1 + offset_angle * 180 / PI;

    double error = 360-target_angle;  // 逆时针转的时候error一般小于0，顺时针大于0
    Servo_Control(kp * error + kd * (error - error_last));
	avoid_obstacle();
    error_last_reverse = error;
}

//Main function 
int main(void)
{	
	//初始化PWM控制I/O口，用于控制电机转速及舵机方向。APB1时钟频率为84M，满PWM为5000
	TIM4_PWM_Init(5000,84);
	//初始化串口5，用于接收雷达数据。波特率为115200
	uart5_init(115200);
	//初始化电机方向控制I/O口，用于控制电机方向
	Motor_Direct_Init();
	//设置电机旋转方向，A6设高电平，C5设低电平，若方向错误，两I/O高低反转即可
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	//设置电机转速
	Speed_Control(0);
	//设置舵机角度
	Servo_Control(0);
	//请求雷达测距数据，发送一次即可
	USART5_START_SEND();
	//开始循环分析雷达数据
	while(1){
		run(0.5, 0.0, 900);
		// Servo_Control(0);
		// run_reverse(0.5, 0.0, 900);
	}
}

