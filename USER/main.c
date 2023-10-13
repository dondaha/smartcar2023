//#include "system.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "usart.h"
#include "usartx.h"

u8 scan[]={0xA5,0x20}; //��ʼɨ�����(SCAN)��������
u8 stop[]={0xA5,0x25}; //ֹͣɨ������
u8 distan;
u16 result;
float mindis=0;
float minang=0;
float maxdis=0;
float maxang=0;
float ldis=0;
float rdis=0;
u8 Right_count,Left_count;
float A1,A2;//���Ҳ������߶�Ӧ�ĺ����
int servo_PWM0=1500;
float x1,y1,x2,y2,w1,w2;//���ɼ���xy���꣬����x��ļн�
float x3,y3,x4,y4,w3,w4;
int obstacle_ang=0;//�ϰ������ڽǶ�
int obstacle_dis=9999;//�ϰ������

float avoid_speed = 0.0; // ����ʱС�����ٶ�
float navigate_speed = 0.0; // ����ʱС�����ٶ�

//���Ƶ����ת����Ķ˿ڳ�ʼ��
void Motor_Direct_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;//IN1��ӦIO�ڣ�A6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
  GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_5;//IN2��ӦIO�ڣ�C5
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure2);//��ʼ��GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
}
//���Ƶ��ת�ټ�����ǶȵĶ˿ڳ�ʼ��
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
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;//���ת�ٿ�����PB6,����Ƕȿ�����PB7
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
//�ٶȿ���
//����ֵ������speed��-0.8~0.8m/s
//���ֵ�����Ƶ����PWMֵ
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
//ת�����
//����ֵ��ǰ��ƫ���angle����Χ-12��~12�㣬��ֵ��ת
//���ֵ�����ƶ����PWMֵ
float angle_0 = 0.0;
void Servo_Control(float angle)
{
	angle_0 = angle;
	if (angle_0>12) angle_0=12;
	if (angle_0<-12) angle_0=-12;
	TIM4->CCR2 = 1430 - angle_0/12*200;
}
//���ϳ���
//�����ϰ������״�����Ұ�෽λ������С����ת����
//ͬʱС�����٣��Ա����ȷ�Ŀ���
void Avoiding(void)
{
	Speed_Control(avoid_speed);
	if(obstacle_ang<180)
		Servo_Control(12);
	else if(obstacle_ang>=180)
		Servo_Control(-12);
}
//��������
//���ת����գ��ж�����������߻����ұ���
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

// ���������������λ��
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

//���״�ɨ�����ݽ��з�����С����ǰ����Ӧ�״��180��
void Lidar(void)
{
	obstacle_ang=0;
	for(int i = 0; i<360; i++)
	{
		int dis = diss[i];
		if(i>90 && i<135 && dis!=0 && dis <1000)//���������������
		{
			if(minang==0)
			{
				mindis=dis;
				ldis=dis;//ȡ���״�����������߼���������
				minang=i;
				w1=180-minang;
				x1=mindis*cos(w1*PI/180);
				y1=mindis*sin(w1*PI/180);//ȡ����������߽�������
			}
			if(i>maxang)
			{
				maxdis=dis;
				maxang=i;
				w2=180-maxang;
				x2=maxdis*cos(w2*PI/180);
				y2=maxdis*sin(w2*PI/180);//ȡ�����������Զ������
			}
		}
		else if (i>225 && i<270 && dis!=0 && dis <1000)//����Ҳ����������
		{
			if(minang<225)
			{
				mindis=dis;
				minang=i;
				w4=180-minang;
				x4=mindis*cos(w4*PI/180);
				y4=mindis*sin(w4*PI/180);//ȡ���Ҳ������߽�������
			}
			if(i>maxang)
			{
				maxdis=dis;
				rdis=dis;//ȡ���״��Ҳ��������߼���������
				maxang=i;
				w3=180-maxang;
				x3=maxdis*cos(w3*PI/180);
				y3=maxdis*sin(w3*PI/180);//ȡ���Ҳ�������Զ������
			}
		}
		else if(i>135 && i<225 && dis!=0 && dis<500)//���ǰ���Ƿ����ϰ���
		{
			if(obstacle_dis>dis)
			{
				obstacle_dis=dis;//�ҵ��ϰ������״�����ĵ�ľ���ͽǶ�
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
	//�������ϰ���ʱ�����ȶ���ϰ������ִ�е���
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
	// ������ǰ������ǰ��������ϰ������
	int left_front_dis = 9999;
	int right_front_dis = 9999;
	for(int i = 0; i<360; i++)
	{
		int dis = diss[i];
		if(i>90 && i<135 && dis!=0 && dis <1000)//���������������
		{
			if(dis < left_front_dis){
				left_front_dis = dis;
			}
		}
		else if (i>225 && i<270 && dis!=0 && dis <1000)//����Ҳ����������
		{
			if(dis < right_front_dis){
				right_front_dis = dis;
			}
		}
	}
	if (left_front_dis < 300 || right_front_dis < 300){
		Speed_Control(0.4);
		// �������ϰ������С���Ҳ��ϰ�����룬��ôС������ת
		if(left_front_dis < right_front_dis){
			Servo_Control(12);
		}
		// ����Ҳ��ϰ������С������ϰ�����룬��ôС������ת
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
	// �ж�ǰ������30�����Ƿ����ϰ���
	// ����diss[150]��diss[210]�ڹ����ĵ������
	int sum_l, sum_r = 0;
	int front_count_l, front_count_r = 0; // �Ϸ��������
	int front_dis_l, front_dis_r = 0; // ������������
	for(int i = 0; i<=30; i++)
	{
		int dis_l = diss[180-i];
		int dis_r = diss[180+i];
		if (dis_l!=0&&dis_l<600) {++front_dis_l;++front_count_l;sum_l+=dis_l;}else if (dis_l>=600) {++front_count_l;sum_l+=dis_l;}
		if (dis_r!=0&&dis_r<600) {++front_dis_r;++front_count_r;sum_r+=dis_r;}else if (dis_r>=600) {++front_count_r;sum_r+=dis_r;}
	}
	if (front_count_l == 0||front_count_r == 0) {Servo_Control(0);return;} // ���ݲ��Ծ��˳�
	float front_avg_dis_l = sum_l / front_count_l; // ǰ������ϰ����ƽ������
	float front_avg_dis_r = sum_r / front_count_r; // ǰ���Ҳ��ϰ����ƽ������
	// ǰ�����ϰ���Ϳ�ʼ����
	if (front_dis_l > 6 || front_dis_r > 6){
		// �������ϰ������С���Ҳ��ϰ�����룬��ôС������ת
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
	// ������ǰ������ǰ������ĵ�ľ���
	int left_front_dis = 9999;
	int right_front_dis = 9999;
	int front_dis = 9999;
	for(int i=0; i<90; ++i){
		int dis_l = diss[180-i];
		int dis_r = diss[180+i];
		if (dis_l!=0 && dis_l <2000 && dis_l<left_front_dis)//���ǰ�����������
		{
			left_front_dis = dis_l;
		}
		if (dis_r!=0 && dis_r <2000 && dis_r<right_front_dis)//���ǰ�����������
		{
			right_front_dis = dis_r;
		}
	}
	// ��������ǰ������ĵ�
	for(int i=150; i<=210; ++i){
		int dis = diss[i];
		if (dis!=0 && dis <2000 && dis<front_dis)//���ǰ�����������
		{
			front_dis = dis;
		}
	}
	if (front_dis < 600){
		Speed_Control(0.6);
		// // �������ϰ������С���Ҳ��ϰ�����룬��ôС������ת
		// if(left_front_dis < right_front_dis){
		// 	Servo_Control(12);
		// }
		// // ����Ҳ��ϰ������С������ϰ�����룬��ôС������ת
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
	// 180~210 ��Ӧdis_min[6]
	// 150~180 ��Ӧdis_min[5]
	int dis;
	for (int i=0; i<360; ++i){
		dis = diss[i];
		if (dis!=0 && dis<dis_min[i/30]){
			dis_min[i/30] = dis;
		}
	}
	// ǰ�����ϰ���Ϳ�ʼ����
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
	// ǰ����Զ��һ������
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
	// ���ǰ�����ϰ����ôС�����٣���ǰ����Զ�ķ���
	if (dis_min[5] < 800 || dis_min[6] < 800) Speed_Control(0.6);
	else Speed_Control(0.8);

	if (dis_min[5]<600 || dis_min[6]<600){
		Speed_Control(0.6);
		// �����ǰ����Զ�ľ������ǰ����Զ�ľ���
		int dis_max_l, dis_max_r = 0;
		for (int i=3; i<=4; ++i){
			if (dis_max[i]>dis_max_l) dis_max_l = dis_max[i];
		}
		for (int i=7; i<=8; ++i){
			if (dis_max[i]>dis_max_r) dis_max_r = dis_max[i];
		}
		// �������ϰ������С���Ҳ��ϰ�����룬��ôС������ת
		if (dis_max_l < dis_max_r) Servo_Control(12);
		// ����Ҳ��ϰ������С������ϰ�����룬��ôС������ת
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
	// new4�Ľ��棬ϸ������������ƽ������
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	// ����ÿ��������ƽ������
	for (int i=COUNT/4; i<=COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// ����ǰ���Ž�30���ڵ���̾���
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// ���ǰ�����ϰ����ôС�����٣���ǰ����Զ�ķ���
	if (dis_min_front < 800) Speed_Control(0.6);
	else Speed_Control(0.8);
	
	if (dis_min_front<700){
		int max_dis_index = get_max_index(dis, COUNT);
		// �����Զ�ķ�������࣬��ôС������ת��������ת
		if (max_dis_index < COUNT/2-1) Servo_Control(-12);
		else if (max_dis_index > COUNT/2) Servo_Control(12);
		else Servo_Control(0);
	}else{
		Servo_Control(0);
	}
}

void lidar_new6(){
	// new5�ĸĽ��汾 Create at 2023.10.8
	// ĿǰЧ������,����new5
	// ��������ʹ��ǰ��������Զƽ���������飬ͬʱ�������������������ϰ����ʱ��ת��
	// �����Զ������ǰ���ĽǶȷ�Χ�ڣ���ת�����С�Ƕ�ת�򣬷�ֹƵ��ת����
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	// ����ÿ��������ƽ������
	for (int i=COUNT/4; i<COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// ����ǰ���Ž�30���ڵ���̾���
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// ���ǰ�����ϰ����ôС������
	if (dis_min_front < 800) Speed_Control(0.6);
	else Speed_Control(0.8);
	// ǰ����Զ������
	int max_dis_index = get_max_index(dis, COUNT);
	if (max_dis_index < COUNT/2-1) Servo_Control(-12);
	else if (max_dis_index > COUNT/2) Servo_Control(12);
	else Servo_Control(0);
}

void lidar_new7(int direction){
	// pid���ڰ汾 // directionΪ0������ʱ�룬1����˳ʱ��
	int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	// ����ǰ���Ž�30���ڵ���̾���
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// ȥ�����ٶȵ��������񽻸���lidar_new8
	// ����С�����30�ȷ�Χ�ڵ�ƽ������
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
	float kp_1 = 0.1; // ����0����
	float kp_2 = 0.24;
	float error_1 = dis_left_back - dis_left_front;
	// error_1����ֵ�޶���Χ
	if (error_1 > 120) error_1 = 120;
	if (error_1 < -120) error_1 = -120;
	float error_2 = 260-dis_left; // ��������Ҫ���Ĳ�����250�ᾭ��ײ������ϰ��300���Ժúܶ�
	// error_2����ֵ�޶���Χ
	if (error_2 > 120) error_2 = 120;
	if (error_2 < -120) error_2 = -120;
	float error =  kp_1 * error_1 + kp_2 * error_2;
	float output = error*(direction==0?1:-1); // directionΪ0������ʱ�룬1����˳ʱ�룬������һ���ԳƵĸ���
	Servo_Control(output);
}

void avoid_obstacle(){
	// �����������ϰ����ʱ�����ϰ�����ײ
	int dis_min = 250; //�ٽ���룬���Ե���
	int dis_l = avg_dis(diss+100, 20);
	int dis_r = avg_dis(diss+240, 20);
	if (dis_l < dis_min && angle_0<0) Servo_Control(0);
	if (dis_r < dis_min && angle_0>0) Servo_Control(0);
}

void lidar_new8(){
	// new5 new7��ϰ�
	// Ч�������ԣ�Ŀǰ�����Ž� Update at 2023.10.9 
	int direction = 0; //��0����ʱ�룻1��˳ʱ�롿
	int dis[COUNT] = {0};
	int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	// ����ÿ��������ƽ������
	for (int i=COUNT/4; i<COUNT*3/4; ++i){
		dis[i] = avg_dis(diss+i*360/COUNT, 360/COUNT);
	}
	// ����ǰ���Ž�30���ڵ���̾���
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// ���ǰ�����ϰ����ôС�����٣���ǰ����Զ�ķ���
	if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(0.6);
	else if (dis_min_front < 500) Speed_Control(0.5);
	else Speed_Control(0.8);
	
	if (dis_min_front<=900){ // �����ֵ������������С����ʱ������������ǵ������ϰ���ľ���
		int max_dis_index = get_max_index(dis, COUNT);
		// // �����Զ�ķ�������࣬��ôС������ת��������ת
		if (max_dis_index < COUNT/2-1) Servo_Control(-12);
		else if (max_dis_index > COUNT/2) Servo_Control(12);
		else lidar_new7(direction); // ��仹ͦ��Ҫ����������ȶ��ԣ��ڷ��������ʱ������ǽ��
	}else{
		lidar_new7(direction);
	} // TO DO ���������ұ������İ汾
	avoid_obstacle();
}



double error_last = 0;

void run(double kp, double kd, int max_dis)
{
	// lyh��idea Ŀǰ�������㷨 update at 2023.10.12
	int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	// ����ǰ���Ž�30���ڵ���̾���
	for (int i=150; i<=210; ++i){
		if (diss[i]!=0 && diss[i]<dis_min_front) dis_min_front = diss[i];
	}
	// ���ǰ�����ϰ����ôС������
	if (dis_min_front < 800 && dis_min_front >= 500) Speed_Control(0.8);
	else if (dis_min_front < 500) Speed_Control(0.8);
	else Speed_Control(0.8);

	// ����ǰ���������ĵ�ķ�λ��ʹ��pid�ƽ��Ǹ���
    int count = 0;
    int index1 = 0, dis1 = 0; // ��ʱ����
    int index2 = 0, dis2 = 0;
	double idx_1 = 0, idx_2 = 0; // ��¼����Զ�������������
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
				// �˴��ľ�����㺯�����󣬵�Ч���ܺã�����
                double d = (double)(index2-index1) * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                // ���������ˣ�����Ч��һ�磬���������
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

    double error = target_angle - 180;  // ��ʱ��ת��ʱ��errorһ��С��0��˳ʱ�����0
    Servo_Control(kp * error + kd * (error - error_last));
	avoid_obstacle();
    error_last = error;
}

double error_last_reverse = 0;

void run_reverse(double kp, double kd, int max_dis)
{
	// lyh��idea Ŀǰ�������㷨 update at 2023.10.12
	// int dis_min_front = 9999; // ��¼ǰ���Ž�30���ڵ���̾���
	Speed_Control(-0.8);

	// ����ǰ���������ĵ�ķ�λ��ʹ��pid�ƽ��Ǹ���
    int count = 0;
    int index1 = 0, dis1 = 0; // ��ʱ����
    int index2 = 0, dis2 = 0;
	double idx_1 = 0, idx_2 = 0; // ��¼����Զ�������������
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
				// �˴��ľ�����㺯�����󣬵�Ч���ܺã�����
                double d = (double)(index2-index1) * (double)(dis1+dis2) / 2.0 + (double)((dis2-dis1)>0?(dis2-dis1):(dis1-dis2));
                // ���������ˣ�����Ч��һ�磬���������
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

    double error = 360-target_angle;  // ��ʱ��ת��ʱ��errorһ��С��0��˳ʱ�����0
    Servo_Control(kp * error + kd * (error - error_last));
	avoid_obstacle();
    error_last_reverse = error;
}

//Main function 
int main(void)
{	
	//��ʼ��PWM����I/O�ڣ����ڿ��Ƶ��ת�ټ��������APB1ʱ��Ƶ��Ϊ84M����PWMΪ5000
	TIM4_PWM_Init(5000,84);
	//��ʼ������5�����ڽ����״����ݡ�������Ϊ115200
	uart5_init(115200);
	//��ʼ������������I/O�ڣ����ڿ��Ƶ������
	Motor_Direct_Init();
	//���õ����ת����A6��ߵ�ƽ��C5��͵�ƽ�������������I/O�ߵͷ�ת����
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	//���õ��ת��
	Speed_Control(0);
	//���ö���Ƕ�
	Servo_Control(0);
	//�����״������ݣ�����һ�μ���
	USART5_START_SEND();
	//��ʼѭ�������״�����
	while(1){
		run(0.5, 0.0, 900);
		// Servo_Control(0);
		// run_reverse(0.5, 0.0, 900);
	}
}

