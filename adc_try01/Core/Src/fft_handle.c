#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"
#include "stdio.h"
#include "lcd.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "LCD_Show.h"
#include "math.h"

#define FFT_LENGTH 4096										//采样点个数
#define LOW_F_INDEX 50
#define JUDGE_AVE 250
#define BIZHI_SIZE 5
#define FOCUS_NUM 10                                           //能量集中聚焦数量
#define FOCUS_JUDGE  20                                      //20个索引内（20*f0频率内），能量直接集中到先位
#define SIN_JUDGE  890
#define FOCUS_JUDGE_3 40                                    //40个索引内（40*f0频率内），能量直接集中到三倍基波分量
#define TYPE_JUDGE_NUM 6                                   //波形类型的削平参数

extern float adcVolt[FFT_LENGTH];
extern float PWM_frequency;
extern uint32_t f_sa;										//基础采样频率为400K=84M/(21*10)	
float fft_inputbuf[FFT_LENGTH * 2];  			//fft计算前数组(含实部虚部)
float fft_outputbuf[FFT_LENGTH];  				//fft计算后数组
float bizhi1[BIZHI_SIZE]; //最大基波分量与其三倍分量的比值
int p1=0; //bizhi1索引
float buff3[400];													//存储数值缓存
float buff2[FFT_LENGTH*2]={0};						//数值缓存
char temp_str;
uint16_t fft_index_max = 1;
uint16_t fft_index_max2 = 1;
uint16_t fft_index_min = 1;
uint16_t waveform_mode = 1;							//1输出方波，2输出三角波，3输出正弦波
extern float fft_frequency;
extern float fft_frequency2;
float fft_direct = 0;
char f_buff[30];
char V_buff[20];
char K_buff[20];
int cnt_calculate = 0;									//用于降低刷新率
int flag_judge;											//是否开始判波的标志
char SPR_STR[100];
int TYPEsumA=0; 
int TYPEsumB=0; 
int CLKsum=0;  //记录判断的次数

struct FRE
{
	float FREC;
	int FRECint;
	int TYPE;
	int TYPEpro;
}FREa,FREb;


void quickSort(float values[], int indices[], int low, int high) {
    if (low < high) {
        float pivot = values[high];
        int i = low - 1;

        for (int j = low; j < high; j++) {
            if (values[j] > pivot) { // 从大到小排序
                i++;
                // 交换值
                float tempVal = values[i];
                values[i] = values[j];
                values[j] = tempVal;

                // 交换索引
                int tempIndex = indices[i];
                indices[i] = indices[j];
                indices[j] = tempIndex;
            }
        }
        // 最后交换
        float tempVal = values[i + 1];
        values[i + 1] = values[high];
        values[high] = tempVal;

        int tempIndex = indices[i + 1];
        indices[i + 1] = indices[high];
        indices[high] = tempIndex;

        quickSort(values, indices, low, i);
        quickSort(values, indices, i + 2, high);
    }
}


void get_vpp(void)	   //获取峰峰值
{
	float max_data=adcVolt[0];
	float min_data=adcVolt[0];
	u32 n=0;
	float Vpp=0;

	for(n = 1;n<FFT_LENGTH;n++)
	{
			if(adcVolt[n] > max_data)
			{
				max_data = adcVolt[n];
			}
			if(adcVolt[n] < min_data)
			{
				min_data = adcVolt[n];
			}	
	} 
		Vpp = (float)(max_data - min_data);

}

float fft_frequency_Vpp_check(uint16_t fft_index_max)				//fft测频率及峰峰值优化函数，用于补偿栅栏效应 
																	//此函数就是将下标值转化为频率值
{
	float sum_A = 0;
//	float sum_Vpp = 0;
//	float Vpp = 0;
	float frequency_average = 0;
	
	//这里只取了5个点，如果不同主要频率点下标相差比较大，可以取更多；反之更少
	int left_dot  = fft_index_max-2;				//设置栅栏效应的左邻谐波边界
	int right_dot = fft_index_max+3;				//设置栅栏效应的右邻谐波边界

	//边界处理
	if(left_dot <= 0)
	{
		left_dot = 1;
	}
	if(right_dot >= FFT_LENGTH/2)
	{
		right_dot = FFT_LENGTH/2-1;
	}
	
	//求Vpp,这里对fft_outputbuf[fft_index_max]附近的点,我们用先求平方和后开方的方法来聚集能量，减小误差		@光 24.7.12
	for (int i = left_dot;i <= right_dot;i++)
	{
		sum_A += fft_outputbuf[i];																					//为下面计算frequency_average做准备，所以不能删
//		sum_Vpp += fft_outputbuf[i]*fft_outputbuf[i];
	}
//	Vpp = sqrt(sum_Vpp)*2;
//	sprintf(V_buff,"Vpp:%0.3f",Vpp);
//	lcd_show_string(10,420,80,16,16,V_buff,WHITE);
	
	//求frequency_average,这里对fft_outputbuf[fft_index_max]附近的点进行加权处理		@光 24.7.12
	for (int i = left_dot;i <= right_dot;i++)
	{
		frequency_average += (i*f_sa/FFT_LENGTH) * (fft_outputbuf[i]/sum_A);			//F测 =（F下标*f_sa采样频率/采样点）*（权重占比）
	}
//	lcd_show_string(405,40,80,16,16,"fft_F(Hz):",YELLOW);
//	sprintf((char*)f_buff,"%0.3fHz",fft_frequency);
//	lcd_show_string(405,60,130,16,16,f_buff,WHITE);
//	sprintf((char*)f_buff,"%0.3fHz",fft_frequency2);
//	lcd_show_string(405,120,80,16,16,"fft_F2(Hz):",YELLOW);
//	lcd_show_string(405,140,80,16,16,f_buff,WHITE);
	return frequency_average;
}

 
void fft_waveform_check_H(void)	//为信号分离H题服务的波形判断及频率测量函数   @Chris 24.10.2
{
	
	//对fft_outputbuff 进行快排
	// 复制原数组从第二个元素开始的部分
	float sortedfft_v[FFT_LENGTH-1];
	int sortedfft_i[FFT_LENGTH-1]; 
	float MID=0;
	

	
	int flag_focus[10]={0,0,0,0,0,0,0,0,0,0};
	int focus_index=0; 
	float FA3=0;          //三倍基波分量 幅值
	float F1=0;
	float F2=0;
	

	
	for (int i = 1; i < FFT_LENGTH/2-1; i++)
	{
		sortedfft_v[i-1]=fft_outputbuf[i]; //记录值
		
		sortedfft_i[i-1]=i;                //记录索引
		
		
	}

	quickSort(sortedfft_v,sortedfft_i,0,FFT_LENGTH/2-2);
	MID=sortedfft_v[0]/(fft_outputbuf[sortedfft_i[0]*3]);
	//能量集中算法，聚焦前 FOCUS_NUM 个
	for (int i=0;i<FOCUS_NUM;i++)
	{
		if (flag_focus[i]!=-1  )
		{
			for (int j=i+1;j<FOCUS_NUM;j++)
			{
				if (abs(sortedfft_i[i]-sortedfft_i[j])<FOCUS_JUDGE)
				{
					sortedfft_v[i]=sqrt(sortedfft_v[j]*sortedfft_v[j]+sortedfft_v[i]*sortedfft_v[i]);  //能量直接相加集中
					flag_focus[j]=-1;
					sortedfft_v[j]=-1;
				}
			}
		}
	}
	//更新集中后的能量 
	
	for (int i=0;i<FOCUS_NUM;i++)
	{
		if (fft_frequency_Vpp_check(sortedfft_i[i])<10000)
		{
			sortedfft_v[i]=-1; //低频能量失效
			flag_focus[focus_index]=-1;
			continue;
		}
		if (flag_focus[i]!=-1)
		{
			sortedfft_i[focus_index]=sortedfft_i[i];
			sortedfft_v[focus_index]=sortedfft_v[i];
			focus_index+=1;
		}

	}
	//再次排序
	quickSort(sortedfft_v,sortedfft_i,0,FOCUS_NUM);
	F1=fft_frequency_Vpp_check(sortedfft_i[0]);
	F2=fft_frequency_Vpp_check(sortedfft_i[1]);
	//判定波形
	if (fabs(sortedfft_v[0]-sortedfft_v[1])>50) //两种波
	{


		if (F1<F2)
		{

			if (sortedfft_v[0]>860)
			{
				//lcd_show_string(100,10,80,16,16,"SIN    ",YELLOW);
				FREa.TYPE=0; //0为SIN
				FREb.TYPE=1;
			}
			else
			{
				//lcd_show_string(100,10,80,16,16,"SANJIAO",YELLOW);
				FREb.TYPE=0; //1为SANJIAO
				FREa.TYPE=1;
			}
		}
		else
		{

			if (sortedfft_v[0]>860)
			{
				//lcd_show_string(100,10,80,16,16,"SIN    ",YELLOW);
				FREb.TYPE=0; //0为SIN
				FREa.TYPE=1;
			}
			else
			{
				//lcd_show_string(100,10,80,16,16,"SANJIAO",YELLOW);
				FREa.TYPE=1; //1为SANJIAO
				FREb.TYPE=0;
			}
		}

	}
	else //一种波
	{
		if (sortedfft_v[0]+sortedfft_v[1]>SIN_JUDGE*2)
		{

			FREb.TYPE=0;
			FREa.TYPE=0;

		}
		else
		{

			FREb.TYPE=1;
			FREa.TYPE=1;

		}

	}


//	//三倍基波分量 能量集中
//	for (int i=sortedfft_i[0]*3-FOCUS_JUDGE_3;i<sortedfft_i[0]*3+FOCUS_JUDGE_3;i++) //(sortedfft_i[0]*3+10<FFT_LENGTH)?sortedfft_i[0]*3+10:FFT_LENGTH
//	{
//		fft_outputbuf[sortedfft_i[0]*3]=sqrt(fft_outputbuf[sortedfft_i[0]*3]*fft_outputbuf[sortedfft_i[0]*3]+fft_outputbuf[i]*fft_outputbuf[i]);  //能量直接相加集中
//	}
	//printf("A1*3:%f,F1*3:%f\n",fft_outputbuf[sortedfft_i[0]*3] ,fft_frequency_Vpp_check(sortedfft_i[0])*3); //验证三倍基波分量
	printf("A1:%f,F1:%f\n",fft_outputbuf[sortedfft_i[0]] ,F1); 
	printf("A2:%f,F2:%f\n",fft_outputbuf[sortedfft_i[1]] ,F2); 
	if (F1<F2)
	{
		FREa.FREC=F1;
		FREb.FREC=F2;
	}
	else
	{
		FREb.FREC=F1;
		FREa.FREC=F2;
	}
	printf("aF:%fHZ\n",FREa.FREC);
	printf("bF:%fHZ\n",FREb.FREC);
	if (fabs(sortedfft_v[0]-sortedfft_v[1])>450) //此时AB波频率相同
	{
		FA3=fft_outputbuf[sortedfft_i[0]*3];
		if (FA3>125)
		{

			FREa.TYPE=1;
			FREb.TYPE=1;
		}
		else if (FA3>50)
		{

			FREa.TYPE=1;
			FREb.TYPE=0;
		}
		else
		{

			FREa.TYPE=0;
			FREb.TYPE=0;
		}
		FREa.FREC=F1;
		FREb.FREC=FREa.FREC;
	}
	
	//对ferc做整数化归类
	FREa.FRECint=FREa.FREC;
	FREb.FRECint=FREb.FREC;
	for (int i=20000;i<=150000;i+=5000)
	{
		if (fabs(FREa.FREC-i)<=2500)
		{
			FREa.FRECint=i;
			break;
		}

	}
	for (int i=20000;i<=150000;i+=5000)
	{
		if (fabs(FREb.FREC-i)<=2500)
		{
			FREb.FRECint=i;
			break;
		}

	}

	//削减TYPE的跳变，保持稳定
	TYPEsumA+=FREa.TYPE;
	TYPEsumB+=FREb.TYPE;
	CLKsum+=1;
	if (CLKsum==TYPE_JUDGE_NUM)
	{
		CLKsum=0;
		
		if (TYPEsumA>TYPE_JUDGE_NUM/2)
		{
			FREa.TYPEpro=1;
			printf("FAT:%d\n",FREa.TYPEpro);
		}
		else
		{
			FREa.TYPEpro=0;
			printf("FBT:%d\n",FREa.TYPEpro);
		}
		TYPEsumA=0;
		if (TYPEsumB>TYPE_JUDGE_NUM/2)
		{
			FREb.TYPEpro=1;
			printf("FBT:%d\n",FREb.TYPEpro);
		}
		else
		{
			FREb.TYPEpro=0;
			printf("FAT:%d\n",FREb.TYPEpro);
		}
		TYPEsumB=0;
		sprintf(SPR_STR,"aT:%1d",FREa.TYPEpro);
		lcd_show_string(10,10,80,16,16,SPR_STR,WHITE);
		sprintf(SPR_STR,"bT:%1d",FREb.TYPEpro);
        lcd_show_string(10,35,80,16,16,SPR_STR,WHITE);
	}
	
	printf("CLK:%d\n",CLKsum);
	//UART打印及屏幕显示
	printf("aT:%d,aF:%fHZ\n",FREa.TYPEpro,FREa.FREC);
	printf("bT:%d,bF:%fHZ\n",FREb.TYPEpro,FREb.FREC);
	

	sprintf(SPR_STR,"aF:%3dkHZ",FREa.FRECint/1000);
	lcd_show_string(100,10,80,16,16,SPR_STR,YELLOW);
	

	sprintf(SPR_STR,"bF:%3dkHZ",FREb.FRECint/1000);
	lcd_show_string(100,35,80,16,16,SPR_STR,YELLOW);
	
	lcd_show_string(10,70,80,16,16,"Tips:",YELLOW);
	lcd_show_string(10,90,80,16,16,"SIN:0",YELLOW);
	lcd_show_string(10,110,80,16,16,"SANJIAO:1",YELLOW);
	
	
	
	printf("-------------------------------------------------\n-----------------------------------------\n----------------------------------------\n");
	

}

void fftCalculate(void)// FFT 计算函数
{
    
		if (cnt_calculate == 0)
		{
			for (int i = 0; i < FFT_LENGTH; i++)								//均值滤波来提高采样精度,每两个点取平均，这样就可以达到2次均值滤波
		{
				fft_inputbuf[i * 2] = adcVolt[i];//实部赋值
				fft_inputbuf[i * 2 + 1] = 0;//虚部赋值，固定为0.
		}

		
		arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);			//这边len后参数要联动FFT_LENGTH修改
		arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);
		
		fft_index_max = 1;
		fft_index_min = 1;
		
		fft_direct = fft_outputbuf[0]/FFT_LENGTH; 						//计算各次谐波幅值并找出最大频率分量
		//处理直流分量（低频）
		for (int i =0;i<LOW_F_INDEX;i++)
		{
			fft_outputbuf[i] = 0;  //存入fft_direct直流分量并让fft_outputbuf[0]置0，保证不影响波形判断
		}
													
		
		fft_waveform_check_H();
		

			cnt_calculate = 25;
		}
		else
		{
			cnt_calculate -= 1;
		}
		
//		printf("FFT Result:\r\n");
//		for (int i = 0; i < FFT_LENGTH/2; i++)							//输出各次谐波幅值
//		{
//			printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);		//FFT输出数组的下标表示的频率=数组下标
//		}
}


