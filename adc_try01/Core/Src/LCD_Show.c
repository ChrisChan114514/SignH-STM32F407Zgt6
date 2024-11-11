
#include "stdio.h"
#include "lcd.h"

#define FFT_LENGTH 4096
extern uint16_t adcBuff[FFT_LENGTH];
extern float buff3[400];
extern uint32_t f_sa ;										//基础采样频率为400K=84M/(21*10)	
extern float fft_frequency;
extern float adcVolt[FFT_LENGTH];								//adc转换后的真实电压数组
uint16_t c = 0;
float Vmin=0.0;
float Vmax=0.0;
float Vpp=0.0;
char spr_str[100];  //为sprintf服务
float wave[4096]={0}; //存储整数周期的波
int lenwave=1;
#define Tnumber 10; //提取10个周期进行示波与FFT


void clear_point(u16 num)//更新显示屏当前列
{
	u16 index_clear_lie = 0; 
	g_point_color = DARKBLUE ;
	for(index_clear_lie = 1;index_clear_lie < 320;index_clear_lie++)
	{		
		lcd_draw_point(num,index_clear_lie,g_point_color);
	}
	if(!(num%40))//判断hang是否为40的倍数 画列点
	{
		for(index_clear_lie = 10;index_clear_lie < 320;index_clear_lie += 10)
		{		
			lcd_draw_point(num ,index_clear_lie,WHITE );
		}
	}
	if(!(num%10))//判断hang是否为10的倍数 画行点
	{
		for(index_clear_lie = 40;index_clear_lie <320;index_clear_lie += 40)
		{		
			lcd_draw_point(num ,index_clear_lie,WHITE );
		}
	}	
	g_point_color = YELLOW;	
}

//void DrawOscillogram(float *buff)//画波形图
//{
//	static u16 Ypos1 = 0,Ypos2 = 0;
////	u16 Yinit=100;
//	u16 i = 0;
//	g_point_color = YELLOW;
//	if(c==0)    //0：实时更新  1：显示存储的波形
//	{
//		for(i = 1;i < 400;i++)
//		{
//			clear_point(i );	
//			Ypos2 = 320-(buff[i] * 320/ 4096);//转换坐标
//			//Ypos2 = Ypos2 * bei;
//			if(Ypos2 >400)
//				Ypos2 =400; //超出范围不显示
//			lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,g_point_color);
//			Ypos1 = Ypos2 ;
//		}
//	}
//	else
//	{
//		for(i = 1;i < 400;i++)
//		{
//			clear_point(i );	
//			Ypos2 = 320-(buff3[i] * 320/ 4096);//转换坐标
//			//Ypos2 = Ypos2 * bei;
//			if(Ypos2 >400)
//				Ypos2 =400; //超出范围不显示
//			lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,g_point_color);
//			Ypos1 = Ypos2 ;
//		}	
//	}
//    Ypos1 = 0;	
//}

void ADCvolt_ANA(void) //分析adcvolt数组
{
//	//先提取Vpp Vmin Vmax 基本量
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
	//显示基本值
//	sprintf(spr_str,"%.2f",Vpp);
//	lcd_show_string(410,20,80,16,16,"              ",YELLOW);
//	lcd_show_string(410,20,80,16,16,spr_str,YELLOW);
//	sprintf(spr_str,"%.2f",Vmin);
//	lcd_show_string(405,190,80,16,16,"             ",YELLOW);
//	lcd_show_string(410,190,80,16,16,spr_str,YELLOW);
//	sprintf(spr_str,"%.2f",Vmax);
//	lcd_show_string(405,230,80,16,16,"             ",YELLOW);
//	lcd_show_string(410,230,80,16,16,spr_str,YELLOW);
	//提取整数个周期
	lenwave=f_sa/fft_frequency;																//f_sa是采样频率,fft_frequency是信号的频率。两者相除得到一个周期内的采样点数
	lenwave=lenwave*Tnumber; 																	//Tnumber个周期采样点数
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
	//有效值判断略
}

void DrawOscillogram(void)//画波形图 整数个周期
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
	//进行插值 将Tnumber拓展至400
	for (i=0;i<lenwave;i++)
	{
		chazhi=400/lenwave;
		if (i==0) //在0处加入余项
		{
			for (j=0;j<chazhi+(400-chazhi*lenwave);j++)
			{
				wave2[j]=wave[0]+(wave[1]-wave[0])/(chazhi+(400-chazhi*lenwave))*j; //插值公式
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
	
	//理论上插值完成(已检验，但是是已Npoint+1为周期)
	for (i = 1;i < 400;i++)
	{
		clear_point(i );
		Ypos2 = 320-((wave2[i]*4096/3.3)* 320/ 4096); 
	
		if(Ypos2 >400)
		Ypos2 =400; //超出范围不显示
		lcd_draw_line(i ,Ypos1 , i+1 ,Ypos2,WHITE);
		Ypos1 = Ypos2 ;
		
	}
}


void Set_BackGround(void)
{
	g_point_color = YELLOW;
  lcd_clear(DARKBLUE);
	lcd_draw_rectangle(0,0,400,320,g_point_color);//矩形
	//LCD_DrawLine(0,220,700,220);//横线
	//LCD_DrawLine(350,20,350,420);//竖线
	//POINT_COLOR = WHITE;
	//BACK_COLOR = DARKBLUE;
	//LCD_ShowString(330,425,210,24,24,(u8*)"vpp=");	
}

void Lcd_DrawNetwork(void)
{
	u16 index_y = 0;
	u16 index_x = 0;	
	
    //画列点	
	for(index_x = 40;index_x < 400;index_x += 40)
	{
		for(index_y = 10;index_y < 320;index_y += 10)
		{
			lcd_draw_point(index_x,index_y,WHITE);	
		}
	}
	//画行点
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
