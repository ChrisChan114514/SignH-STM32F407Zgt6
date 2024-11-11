
#include "stdio.h"
#include "lcd.h"

#define FFT_LENGTH 4096
extern uint16_t adcBuff[FFT_LENGTH];
extern float buff3[400];
extern uint32_t f_sa ;										//��������Ƶ��Ϊ400K=84M/(21*10)	
extern float fft_frequency;
extern float adcVolt[FFT_LENGTH];								//adcת�������ʵ��ѹ����
uint16_t c = 0;
float Vmin=0.0;
float Vmax=0.0;
float Vpp=0.0;
char spr_str[100];  //Ϊsprintf����
float wave[4096]={0}; //�洢�������ڵĲ�
int lenwave=1;
#define Tnumber 10; //��ȡ10�����ڽ���ʾ����FFT


void clear_point(u16 num)//������ʾ����ǰ��
{
	u16 index_clear_lie = 0; 
	g_point_color = DARKBLUE ;
	for(index_clear_lie = 1;index_clear_lie < 320;index_clear_lie++)
	{		
		lcd_draw_point(num,index_clear_lie,g_point_color);
	}
	if(!(num%40))//�ж�hang�Ƿ�Ϊ40�ı��� ���е�
	{
		for(index_clear_lie = 10;index_clear_lie < 320;index_clear_lie += 10)
		{		
			lcd_draw_point(num ,index_clear_lie,WHITE );
		}
	}
	if(!(num%10))//�ж�hang�Ƿ�Ϊ10�ı��� ���е�
	{
		for(index_clear_lie = 40;index_clear_lie <320;index_clear_lie += 40)
		{		
			lcd_draw_point(num ,index_clear_lie,WHITE );
		}
	}	
	g_point_color = YELLOW;	
}

//void DrawOscillogram(float *buff)//������ͼ
//{
//	static u16 Ypos1 = 0,Ypos2 = 0;
////	u16 Yinit=100;
//	u16 i = 0;
//	g_point_color = YELLOW;
//	if(c==0)    //0��ʵʱ����  1����ʾ�洢�Ĳ���
//	{
//		for(i = 1;i < 400;i++)
//		{
//			clear_point(i );	
//			Ypos2 = 320-(buff[i] * 320/ 4096);//ת������
//			//Ypos2 = Ypos2 * bei;
//			if(Ypos2 >400)
//				Ypos2 =400; //������Χ����ʾ
//			lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,g_point_color);
//			Ypos1 = Ypos2 ;
//		}
//	}
//	else
//	{
//		for(i = 1;i < 400;i++)
//		{
//			clear_point(i );	
//			Ypos2 = 320-(buff3[i] * 320/ 4096);//ת������
//			//Ypos2 = Ypos2 * bei;
//			if(Ypos2 >400)
//				Ypos2 =400; //������Χ����ʾ
//			lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,g_point_color);
//			Ypos1 = Ypos2 ;
//		}	
//	}
//    Ypos1 = 0;	
//}

void ADCvolt_ANA(void) //����adcvolt����
{
//	//����ȡVpp Vmin Vmax ������
//	int i;
//	Vmin=adcVolt[0];
//	Vmax=adcVolt[0];
//	for (i=0;i<FFT_LENGTH;i++)
//	{
//		if (Vmin>adcVolt[i]) 
//		{
//			Vmin=adcVolt[i];
//		}
//		if (Vmax<adcVolt[i])
//		{
//			Vmax=adcVolt[i];
//		}
//	}
//	Vpp=Vmax-Vmin;
	//��ʾ����ֵ
//	sprintf(spr_str,"%.2f",Vpp);
//	lcd_show_string(410,20,80,16,16,"              ",YELLOW);
//	lcd_show_string(410,20,80,16,16,spr_str,YELLOW);
//	sprintf(spr_str,"%.2f",Vmin);
//	lcd_show_string(405,190,80,16,16,"             ",YELLOW);
//	lcd_show_string(410,190,80,16,16,spr_str,YELLOW);
//	sprintf(spr_str,"%.2f",Vmax);
//	lcd_show_string(405,230,80,16,16,"             ",YELLOW);
//	lcd_show_string(410,230,80,16,16,spr_str,YELLOW);
	//��ȡ����������
	lenwave=f_sa/fft_frequency;																//f_sa�ǲ���Ƶ��,fft_frequency���źŵ�Ƶ�ʡ���������õ�һ�������ڵĲ�������
	lenwave=lenwave*Tnumber; 																	//Tnumber�����ڲ�������
	if (lenwave>4095)
	{
		lenwave=4095;
	}
	lcd_show_string(405,200,80,16,16,"Sa_num:",YELLOW);
	sprintf(spr_str,"%4d",lenwave);
	lcd_show_string(405,220,80,16,16,spr_str,WHITE);
	for (int i=0;i<lenwave;i++)
	{
		wave[i]=adcVolt[i];
		//printf("wave:%.1f\n\r",wave[i]);
	}
	//��Чֵ�ж���
}

void DrawOscillogram(void)//������ͼ ����������
{
	static u16 Ypos1 = 0,Ypos2 = 0;
	int chazhi=1;
	int i;
	int j=1;
	float wave2[4096];
	float pos0=0;
	if (lenwave==1)
	{
		lenwave=1;
	}
	if (400/lenwave<=1)
	{
		lenwave=400;
	}
	//���в�ֵ ��Tnumber��չ��400
	for (i=0;i<lenwave;i++)
	{
		chazhi=400/lenwave;
		if (i==0) //��0����������
		{
			for (j=0;j<chazhi+(400-chazhi*lenwave);j++)
			{
				wave2[j]=wave[0]+(wave[1]-wave[0])/(chazhi+(400-chazhi*lenwave))*j; //��ֵ��ʽ
			}
			pos0=chazhi+(400-chazhi*lenwave);
		}
		else
		{
			for (j=pos0+(i-1)*chazhi;j<chazhi*i+pos0;j++)
			{
				wave2[j]=wave[i]+(wave[(i+1)%lenwave]-wave[i])/(chazhi)*(j-pos0-(i-1)*chazhi);
			}
		}
	}
	
	//�����ϲ�ֵ���(�Ѽ��飬��������Npoint+1Ϊ����)
	for (i = 1;i < 400;i++)
	{
		clear_point(i );
		Ypos2 = 320-((wave2[i]*4096/3.3)* 320/ 4096); 
	
		if(Ypos2 >400)
		Ypos2 =400; //������Χ����ʾ
		lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,WHITE);
		Ypos1 = Ypos2 ;
		
	}
}


void Set_BackGround(void)
{
	g_point_color = YELLOW;
  lcd_clear(DARKBLUE);
	lcd_draw_rectangle(0,0,400,320,g_point_color);//����
	//LCD_DrawLine(0,220,700,220);//����
	//LCD_DrawLine(350,20,350,420);//����
	//POINT_COLOR = WHITE;
	//BACK_COLOR = DARKBLUE;
	//LCD_ShowString(330,425,210,24,24,(u8*)"vpp=");	
}

void Lcd_DrawNetwork(void)
{
	u16 index_y = 0;
	u16 index_x = 0;	
	
    //���е�	
	for(index_x = 40;index_x < 400;index_x += 40)
	{
		for(index_y = 10;index_y < 320;index_y += 10)
		{
			lcd_draw_point(index_x,index_y,WHITE);	
		}
	}
	//���е�
	for(index_y = 40;index_y < 320;index_y += 40)
	{
		for(index_x = 10;index_x < 400;index_x += 10)
		{
			lcd_draw_point(index_x,index_y,WHITE);	
		}
	}
}
void Draw_Prompt(void)
{
	g_back_color = DARKBLUE;
	g_point_color=WHITE; 
	
	
	

//	lcd_show_string(405,170,80,16,16,"Memory:",WHITE);

	
}
