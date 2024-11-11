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

#define FFT_LENGTH 4096										//���������
#define LOW_F_INDEX 50
#define JUDGE_AVE 250
#define BIZHI_SIZE 5
#define FOCUS_NUM 10                                           //�������о۽�����
#define FOCUS_JUDGE  20                                      //20�������ڣ�20*f0Ƶ���ڣ�������ֱ�Ӽ��е���λ
#define SIN_JUDGE  890
#define FOCUS_JUDGE_3 40                                    //40�������ڣ�40*f0Ƶ���ڣ�������ֱ�Ӽ��е�������������
#define TYPE_JUDGE_NUM 6                                   //�������͵���ƽ����

extern float adcVolt[FFT_LENGTH];
extern float PWM_frequency;
extern uint32_t f_sa;										//��������Ƶ��Ϊ400K=84M/(21*10)	
float fft_inputbuf[FFT_LENGTH * 2];  			//fft����ǰ����(��ʵ���鲿)
float fft_outputbuf[FFT_LENGTH];  				//fft���������
float bizhi1[BIZHI_SIZE]; //�����������������������ı�ֵ
int p1=0; //bizhi1����
float buff3[400];													//�洢��ֵ����
float buff2[FFT_LENGTH*2]={0};						//��ֵ����
char temp_str;
uint16_t fft_index_max = 1;
uint16_t fft_index_max2 = 1;
uint16_t fft_index_min = 1;
uint16_t waveform_mode = 1;							//1���������2������ǲ���3������Ҳ�
extern float fft_frequency;
extern float fft_frequency2;
float fft_direct = 0;
char f_buff[30];
char V_buff[20];
char K_buff[20];
int cnt_calculate = 0;									//���ڽ���ˢ����
int flag_judge;											//�Ƿ�ʼ�в��ı�־
char SPR_STR[100];
int TYPEsumA=0; 
int TYPEsumB=0; 
int CLKsum=0;  //��¼�жϵĴ���

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
            if (values[j] > pivot) { // �Ӵ�С����
                i++;
                // ����ֵ
                float tempVal = values[i];
                values[i] = values[j];
                values[j] = tempVal;

                // ��������
                int tempIndex = indices[i];
                indices[i] = indices[j];
                indices[j] = tempIndex;
            }
        }
        // ��󽻻�
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


void get_vpp(void)	   //��ȡ���ֵ
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

float fft_frequency_Vpp_check(uint16_t fft_index_max)				//fft��Ƶ�ʼ����ֵ�Ż����������ڲ���դ��ЧӦ 
																	//�˺������ǽ��±�ֵת��ΪƵ��ֵ
{
	float sum_A = 0;
//	float sum_Vpp = 0;
//	float Vpp = 0;
	float frequency_average = 0;
	
	//����ֻȡ��5���㣬�����ͬ��ҪƵ�ʵ��±����Ƚϴ󣬿���ȡ���ࣻ��֮����
	int left_dot  = fft_index_max-2;				//����դ��ЧӦ������г���߽�
	int right_dot = fft_index_max+3;				//����դ��ЧӦ������г���߽�

	//�߽紦��
	if(left_dot <= 0)
	{
		left_dot = 1;
	}
	if(right_dot >= FFT_LENGTH/2)
	{
		right_dot = FFT_LENGTH/2-1;
	}
	
	//��Vpp,�����fft_outputbuf[fft_index_max]�����ĵ�,����������ƽ���ͺ󿪷��ķ������ۼ���������С���		@�� 24.7.12
	for (int i = left_dot;i <= right_dot;i++)
	{
		sum_A += fft_outputbuf[i];																					//Ϊ�������frequency_average��׼�������Բ���ɾ
//		sum_Vpp += fft_outputbuf[i]*fft_outputbuf[i];
	}
//	Vpp = sqrt(sum_Vpp)*2;
//	sprintf(V_buff,"Vpp:%0.3f",Vpp);
//	lcd_show_string(10,420,80,16,16,V_buff,WHITE);
	
	//��frequency_average,�����fft_outputbuf[fft_index_max]�����ĵ���м�Ȩ����		@�� 24.7.12
	for (int i = left_dot;i <= right_dot;i++)
	{
		frequency_average += (i*f_sa/FFT_LENGTH) * (fft_outputbuf[i]/sum_A);			//F�� =��F�±�*f_sa����Ƶ��/�����㣩*��Ȩ��ռ�ȣ�
	}
//	lcd_show_string(405,40,80,16,16,"fft_F(Hz):",YELLOW);
//	sprintf((char*)f_buff,"%0.3fHz",fft_frequency);
//	lcd_show_string(405,60,130,16,16,f_buff,WHITE);
//	sprintf((char*)f_buff,"%0.3fHz",fft_frequency2);
//	lcd_show_string(405,120,80,16,16,"fft_F2(Hz):",YELLOW);
//	lcd_show_string(405,140,80,16,16,f_buff,WHITE);
	return frequency_average;
}

 
void fft_waveform_check_H(void)	//Ϊ�źŷ���H�����Ĳ����жϼ�Ƶ�ʲ�������   @Chris 24.10.2
{
	
	//��fft_outputbuff ���п���
	// ����ԭ����ӵڶ���Ԫ�ؿ�ʼ�Ĳ���
	float sortedfft_v[FFT_LENGTH-1];
	int sortedfft_i[FFT_LENGTH-1]; 
	float MID=0;
	

	
	int flag_focus[10]={0,0,0,0,0,0,0,0,0,0};
	int focus_index=0; 
	float FA3=0;          //������������ ��ֵ
	float F1=0;
	float F2=0;
	

	
	for (int i = 1; i < FFT_LENGTH/2-1; i++)
	{
		sortedfft_v[i-1]=fft_outputbuf[i]; //��¼ֵ
		
		sortedfft_i[i-1]=i;                //��¼����
		
		
	}

	quickSort(sortedfft_v,sortedfft_i,0,FFT_LENGTH/2-2);
	MID=sortedfft_v[0]/(fft_outputbuf[sortedfft_i[0]*3]);
	//���������㷨���۽�ǰ FOCUS_NUM ��
	for (int i=0;i<FOCUS_NUM;i++)
	{
		if (flag_focus[i]!=-1  )
		{
			for (int j=i+1;j<FOCUS_NUM;j++)
			{
				if (abs(sortedfft_i[i]-sortedfft_i[j])<FOCUS_JUDGE)
				{
					sortedfft_v[i]=sqrt(sortedfft_v[j]*sortedfft_v[j]+sortedfft_v[i]*sortedfft_v[i]);  //����ֱ����Ӽ���
					flag_focus[j]=-1;
					sortedfft_v[j]=-1;
				}
			}
		}
	}
	//���¼��к������ 
	
	for (int i=0;i<FOCUS_NUM;i++)
	{
		if (fft_frequency_Vpp_check(sortedfft_i[i])<10000)
		{
			sortedfft_v[i]=-1; //��Ƶ����ʧЧ
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
	//�ٴ�����
	quickSort(sortedfft_v,sortedfft_i,0,FOCUS_NUM);
	F1=fft_frequency_Vpp_check(sortedfft_i[0]);
	F2=fft_frequency_Vpp_check(sortedfft_i[1]);
	//�ж�����
	if (fabs(sortedfft_v[0]-sortedfft_v[1])>50) //���ֲ�
	{


		if (F1<F2)
		{

			if (sortedfft_v[0]>860)
			{
				//lcd_show_string(100,10,80,16,16,"SIN    ",YELLOW);
				FREa.TYPE=0; //0ΪSIN
				FREb.TYPE=1;
			}
			else
			{
				//lcd_show_string(100,10,80,16,16,"SANJIAO",YELLOW);
				FREb.TYPE=0; //1ΪSANJIAO
				FREa.TYPE=1;
			}
		}
		else
		{

			if (sortedfft_v[0]>860)
			{
				//lcd_show_string(100,10,80,16,16,"SIN    ",YELLOW);
				FREb.TYPE=0; //0ΪSIN
				FREa.TYPE=1;
			}
			else
			{
				//lcd_show_string(100,10,80,16,16,"SANJIAO",YELLOW);
				FREa.TYPE=1; //1ΪSANJIAO
				FREb.TYPE=0;
			}
		}

	}
	else //һ�ֲ�
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


//	//������������ ��������
//	for (int i=sortedfft_i[0]*3-FOCUS_JUDGE_3;i<sortedfft_i[0]*3+FOCUS_JUDGE_3;i++) //(sortedfft_i[0]*3+10<FFT_LENGTH)?sortedfft_i[0]*3+10:FFT_LENGTH
//	{
//		fft_outputbuf[sortedfft_i[0]*3]=sqrt(fft_outputbuf[sortedfft_i[0]*3]*fft_outputbuf[sortedfft_i[0]*3]+fft_outputbuf[i]*fft_outputbuf[i]);  //����ֱ����Ӽ���
//	}
	//printf("A1*3:%f,F1*3:%f\n",fft_outputbuf[sortedfft_i[0]*3] ,fft_frequency_Vpp_check(sortedfft_i[0])*3); //��֤������������
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
	if (fabs(sortedfft_v[0]-sortedfft_v[1])>450) //��ʱAB��Ƶ����ͬ
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
	
	//��ferc������������
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

	//����TYPE�����䣬�����ȶ�
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
	//UART��ӡ����Ļ��ʾ
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

void fftCalculate(void)// FFT ���㺯��
{
    
		if (cnt_calculate == 0)
		{
			for (int i = 0; i < FFT_LENGTH; i++)								//��ֵ�˲�����߲�������,ÿ������ȡƽ���������Ϳ��Դﵽ2�ξ�ֵ�˲�
		{
				fft_inputbuf[i * 2] = adcVolt[i];//ʵ����ֵ
				fft_inputbuf[i * 2 + 1] = 0;//�鲿��ֵ���̶�Ϊ0.
		}

		
		arm_cfft_f32(&arm_cfft_sR_f32_len4096, fft_inputbuf, 0, 1);			//���len�����Ҫ����FFT_LENGTH�޸�
		arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);
		
		fft_index_max = 1;
		fft_index_min = 1;
		
		fft_direct = fft_outputbuf[0]/FFT_LENGTH; 						//�������г����ֵ���ҳ����Ƶ�ʷ���
		//����ֱ����������Ƶ��
		for (int i =0;i<LOW_F_INDEX;i++)
		{
			fft_outputbuf[i] = 0;  //����fft_directֱ����������fft_outputbuf[0]��0����֤��Ӱ�첨���ж�
		}
													
		
		fft_waveform_check_H();
		

			cnt_calculate = 25;
		}
		else
		{
			cnt_calculate -= 1;
		}
		
//		printf("FFT Result:\r\n");
//		for (int i = 0; i < FFT_LENGTH/2; i++)							//�������г����ֵ
//		{
//			printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);		//FFT���������±��ʾ��Ƶ��=�����±�
//		}
}


