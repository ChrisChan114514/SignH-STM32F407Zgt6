//-----------------------------------------------------------------
// ³ÌÐòÃèÊö: AD9910²âÊÔ³ÌÐò
// ×÷¡¡¡¡Õß: ÁèÖÇµç×Ó
// ¿ªÊ¼ÈÕÆÚ: 2020-05-01
// Íê³ÉÈÕÆÚ: 2020-05-05
// µ±Ç°°æ±¾: V1.2£¬Ôö¼Ó·½²¨ºÍÈý½Ç²¨Êä³ö¹¦ÄÜ
// ÀúÊ·°æ±¾:
//		v1.0:³õÊ¼°æ±¾
//		v1.1:Ôö¼Ó²¢ÐÐµ÷ÖÆÄ£Ê½
// µ÷ÊÔ¹¤¾ß: ÁèÖÇSTM32ºËÐÄ¿ª·¢°å¡¢2.8´çÒº¾§¡¢LZE_ST_LINK2
// Ëµ¡¡¡¡Ã÷: 
//				
//-----------------------------------------------------------------
//												AD9910ÓëSTM32F1Á¬½Ó
//						  AD9910                    		STM32F1
//								MRT														PB15
//								CSN														PB14
//								SCK														PB13
//								SDI														PB12
//								IUP														PB11
//								DRH														PB10
//								DRC														PB9
//								PF0														PB8
//								PF1														PA7
//								PF2														PA6
//								OSK														PC7
//
//								GND														JGND
//
//			ÆäÖÐ 	DRG¹¦ÄÜÒ²¿ÉÒÔÍâ½Ó¿ØÖÆÐÅºÅÀ´¿ØÖÆ    ¿ØÖÆÒý½ÅÎª DRC	   DRH
//			 			OSK¹¦ÄÜÒ²¿ÉÒÔÍâ½ç¿ØÖÆÐÅºÅÀ´¿ØÖÆ    ¿ØÖÆÒý½ÅÎª OSK
// Èç¹ûÐèÒª½øÐÐ²¢ÐÐµ÷ÖÆÄ£Ê½, Ôò»¹ÐèÒª¶ÔÄ£¿éµÄP1½Ó¿Ú½øÐÐÁ¬½Ó
// ²»ÐèÒª²¢ÐÐµ÷ÖÆÄ£Ê½, ²»ÐèÒªÁ¬½ÓÒÔÏÂP1½Ó¿Ú£¨Ä£Ê½µÄ£©

//								F0														PA5
//								F1														PA4
//								D0														PA3
//								D1														PA2
//								D2														PA1
//								D3														PA0
//								D4														PC5
//								D5														PC4
//								D6														PC3
//								D7														PC2
//								D8														PC1
//								D9														PC0
//								D10														PC8
//								D11														PC9
//								D12														PC10
//								D13														PC11
//								D14														PC12
//								D15														PC13
//
//
//-----------------------------------------------------------------
// Í·ÎÄ¼þ°üº¬
//-----------------------------------------------------------------
#include <stm32f10x.h>
#include "Delay.h"
#include "string.h"
#include "PeripheralInit.h"
#include "AD9910.h"
#include "key.h" 
#include "lze_lcd.h"
#include "stdio.h"
#include "fonts.h"

unsigned char static DataStr[]=__DATE__;
unsigned char static TimeStr[]=__TIME__;

u8 key_numb;											// °´¼ü±êÖ¾Î»
u8 numb=0;												// ±êÖ¾Î»
u8 NB=0;													// AD9910¹¦ÄÜ±êÖ¾Î»
u8 buf[10];												// ÏÔÊ¾Êý×é
int i;														// Ñ­»·±êÖ¾Î»£¨ÑéÖ¤ÓÃµÄ£©

// ³õÊ¼»¯µÄµ¥ÆµÆµÂÊ
uint32_t fre_data[] = {0, 1, 10, 100, 500, 1000, 5000, 10000, 50000, 100000, 500000, 1000000, 5000000, 10000000, 50000000, 100000000, 200000000, 300000000, 380000000, 400000000, 420000000};
// ³õÊ¼»¯µÄµ¥Æµ·ù¶È
int amp_data[] = {0,1023,2047,4095,8191,9215,10239,11263,12287,13311,14335,15359,16383};
// ³õÊ¼»¯µÄµ¥ÆµÏàÎ»
int pha_data[] = {0,90,180,270,360};

// ²¢¿ÚµÄÆ«ÒÆÁ¿
int offset_fre[64] = {0, 1000, 2000, 4000, 8000, 10000, 20000, 40000, 60000};
// ²¢¿ÚµÄÔöÒæ
int gain[64] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	
//-----------------------------------------------------------------
// int get_data(int pos_fre, int num[])
//-----------------------------------------------------------------
//
//  º¯Êý¹¦ÄÜ: ½«numÖÐµÄÊýÖµºÏ³ÉÒ»¸öÊý
//  Èë¿Ú²ÎÊý: pos_freÊý×éÓÐÐ§Î»Êý£¬numÊý¾Ý´æ·ÅµÄÊý×é
//  ·µ»Ø²ÎÊý: ·µ»ØÊý¾ÝÖµ
//  ×¢ÒâÊÂÏî: ÎÞ
//
//-----------------------------------------------------------------
int get_data(int pos_fre, int num[])
{
	int i = 0, data;
	
	data = num[i];
	i++;
	while(i <= pos_fre)
	{
		data = data*10 + num[i];	
		i++;
	}
	
	return data;	
}
//-----------------------------------------------------------------
// void  clr_data(int pos_fre, int num[])
//-----------------------------------------------------------------
//
//  º¯Êý¹¦ÄÜ: Çå³ýÊý×éÀïÃæµÄÊý
//  Èë¿Ú²ÎÊý: pos_freÊý×éÓÐÐ§Î»Êý£¬numÊý¾Ý´æ·ÅµÄÊý×é
//  ·µ»Ø²ÎÊý: ÎÞ
//  ×¢ÒâÊÂÏî: ÎÞ
//
//-----------------------------------------------------------------
void  clr_data(int pos_fre, int num[])
{
	int i = 0;
	
	for(; i <= pos_fre; i++)
	{
		num[i] = 0;
	}
}
//9910É¨Æµ²ÎÊý
typedef struct dds_ad9910
{
	int start_parm;
	int stop_parm;
	int rising_step;
	int falling_step;
	int rising_time;
	int falling_time;
}DDS;
//-----------------------------------------------------------------
// Ö÷³ÌÐò
//-----------------------------------------------------------------
int main(void)
{
	int flag = 0;
	int num[10];
	DDS ad9910;
	uint8_t  tx_buf[64];
	int pos_fre = 0;
	int pos_f = 0,pos_a = 0, pos_p = 0;
	int pos_1 = 0,pos_2 = 0, pos_3 = 0, pos_4 = 0, pos_5 = 0, pos_6 = 0, pos_7 = 0;
	int posx = 0, posy = 40, posx1 = 100, posy1 = 40;
	int fre = 0;
	int amp = 0, pha = 0;
	PeripheralInit();																		// ÍâÉè³õÊ¼»¯
	AD9910_Init();																			// AD9910Ê±ÖÓµÄÉèÖÃºÍ¸¨ÖúDACÉèÖÃ ÒÔ¼°AD9910¸´Î»
	LCD_Clear(LCD_COLOR_WHITE); // ÉèÖÃ±³¾°É«

	LCD_SetFont(&Font8, &LCD_DEFAULT_FONT_CN); 
	LCD_WriteString (   230,    150,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"HARDWARE_VERSION" );
	LCD_WriteString (   230,    170,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"FIRMWARE_VERSION" );
	sprintf((char *)tx_buf, "BUILD TIME: %s  %s", DataStr, TimeStr);
	LCD_WriteString(    150,    190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  tx_buf);
	LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN); 
	
	LCD_SetFont(&Font16, &Font16_CN);
	LCD_WriteString (     40,   220,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"k1ÏòÏÂ  k2ÏòÉÏ  k3È·¶¨  k4·µ»Ø" );
	LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN);
	
	while(1)
	{
		LCD_SetFont(&Font8, &LCD_DEFAULT_FONT_CN); 
		LCD_WriteString (   230,    150,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"HARDWARE_VERSION" );
		LCD_WriteString (   230,    170,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"FIRMWARE_VERSION" );
		sprintf((char *)tx_buf, "BUILD TIME: %s  %s", DataStr, TimeStr);
		LCD_WriteString(    150,    190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  tx_buf);
		LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN); 
		
		LCD_WriteString (    60,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"AD9910Ä£Ê½Ñ¡Ôñ" );
		LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
		LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.µ¥ÆµÄ£Ê½" );
		LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.RAMÄ£Ê½" );
		LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.Êý×ÖÐ±ÆÂµ÷ÖÆÄ£Ê½" );
		LCD_WriteString (    30,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"4.OSKÄ£Ê½" );
		LCD_WriteString (    30,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"5.²¢¿Úµ÷ÖÆ" );
		
		 
		
		key_numb = Key_Scan();															// °´¼üÉ¨Ãè
		switch(key_numb)
		{
			case 1: 
				LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"   " );
				if(posy == 160)
				{
					posy = 10;
				}
				posy += 30;
				break;
			case 2:
				LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"   " );
				if(posy == 40)
				{
					posy = 190;
				}
				posy -= 30;
				break;
			case 3: 
				LCD_WriteString (    60,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                            " );
				LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                            " );
				LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                            " );
				LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                            " );
				LCD_WriteString (    30,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                           " );
				LCD_WriteString (    30,    130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                           " );
				LCD_WriteString (    30,    160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                           " );
				while(key_numb != 4)
				{
					LCD_WriteString (   230,    150,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                " );
					LCD_WriteString (   230,    170,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                " );
					LCD_WriteString (   150,    190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
					switch(posy/10)
					{
						case 4:  //µ¥ÆµÄ£Ê½
							posx = 0;
							posy = 40;
							posx1 = 100; posy1 = 40; 
							LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"µ¥ÆµÄ£Ê½" );
					  	LCD_WriteString (    40,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆµÂÊ:" );
						  LCD_WriteString (    40,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"·ù¶È:" );
						  LCD_WriteString (    40,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÏàÎ»:" );
							LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );

						
							LCD_WriteString (    100,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
							LCD_WriteNumLong(    100,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_f]);
							LCD_WriteString (    100,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
							LCD_WriteNumLong(    100,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_a]);
							LCD_WriteString (    100,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
							LCD_WriteNumLong(    100,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_p]);
							while(key_numb != 4)
							{
								key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
								switch(key_numb)
								{
									case 1:	
										if(posy1 == 40)
										{
											if(pos_f == (sizeof(fre_data)/sizeof(uint32_t)-1))
											{
												pos_f = 0;
											}
											pos_f++;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_f]);
										}else if(posy1 == 70)
										{
											if(pos_a == (sizeof(amp_data)/sizeof(int)-1))
											{
												pos_a = 0;
											}
											pos_a++;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_a]);
										}else if(posy1 == 100)
										{
											if(pos_p == (sizeof(pha_data)/sizeof(int)-1))
											{
												pos_p = 0;
											}
											pos_p++;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_p]);
										}		
										break;
									case 2:	
									  if(posy1 == 40)
										{		
											if(pos_f == 0)
											{
												pos_f = sizeof(fre_data)/sizeof(uint32_t);
											}
											pos_f--;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_f]);
									  }else if(posy1 == 70)
										{
											if(pos_a == 0)
											{
												pos_a = sizeof(amp_data)/sizeof(int);
											}
											pos_a--;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_a]);
										}else if(posy1 == 100)
										{
											if(pos_p == 0)
											{
												pos_p = sizeof(pha_data)/sizeof(int)-1;
											}		
											pos_p--;
											LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
											LCD_WriteNumLong(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_p]);								
										}
										break;
									case 3:		
										if(posy1 == 40)
										{
											fre = fre_data[pos_f];
										}else if(posy1 == 70)
										{
											amp = amp_data[pos_a];
										}else if(posy1 == 100)
										{
											pha = pha_data[pos_p];	
											AD9910_Singal_Profile_Init();																	// µ¥ÆµPROFILE³õÊ¼»¯
											AD9910_Singal_Profile_Set(0, fre, amp, pha);									// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©
										}	
										LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
										posy += 30;
										LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
										if(posy >= 130)
										{
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
										}	
										posy1 += 30;
										break;
									case 4:
										LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                                            " );
										LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                                            " );
										LCD_WriteString (    30,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                                           " );
										LCD_WriteString (    30,    130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                                           " );
										LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
										
									  posx = 0; posy = 40; posx1 = 100; posy1 = 40;	
										break;
									default:
										break;
								}
							}
							break;
						case 7:    //RAMÄ£Ê½
							posx = 0;
  						posy = 40;		
							while(key_numb != 4)
							{	
								flag = 0;
								LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"RAMÄ£Ê½" );
								LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
								LCD_WriteString (    35,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.Ö±½Ó×ª»»Ä£Ê½" );
								LCD_WriteString (    35,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.ÉÏÐ±ÆÂÄ£Ê½" );
								LCD_WriteString (    35,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.Ë«ÏòÐ±ÆÂÄ£Ê½" );
								LCD_WriteString (    35,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"4.Á¬ÐøË«ÏòÐ±ÆÂÄ£Ê½" );
								LCD_WriteString (    35,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"5.Á¬ÐøÑ­»·Ä£Ê½" );
								LCD_WriteString (    35,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"6.Èý½Ç²¨ºÍ·½²¨" );
								while(flag == 0 && key_numb != 4)
								{
									key_numb = Key_Scan();															// °´¼üÉ¨Ãè
									switch(key_numb)
									{
										case 1:		
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											if(posy == 190)
											{
												posy = 10;
											}
											posy += 30;
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
											break;
										case 2:
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											if(posy == 40)
											{
												posy = 220;
											}
											posy -= 30;
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
											break;
										case 3:
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"               " );					
											LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"               " );
											LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"               " );
											LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"              " );
											LCD_WriteString (    30,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"              " );
											LCD_WriteString (    30,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"               " );
											LCD_WriteString (    30,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"           " );
											if(posy == 40)
											{														
												while(key_numb != 5)
												{		
													LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Ö±½Ó×ª»»Ä£Ê½" );
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 130)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 100;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_Fre_W();																							// Ð´ÆµÂÊµÄRAM Ð´Èëprofie0-4·Ö±ð2M 4M 6M 8M 10M 			
																	AD9910_RAM_DIR_Fre_R();																					// »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÖ±½Ó×ª»»Ä£Ê½-----ÆµÂÊ
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();																				// ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);																	// ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_AMP_W();																							// Ð´·ù¶ÈµÄRAM Ð´Èëprofile0 740mv×óÓÒ  profile1 90mv×óÓÒ profile2 180mv×óÓÒ  profile3 370mv×óÓÒ profile4 740mv×óÓÒ
																	AD9910_RAM_DIR_AMP_R();																					// »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÖ±½Ó×ª»»Ä£Ê½-----·ù¶È
																	break;
																case 10:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_Pha_W();                                             // Ð´ÏàÎ»µÄRAM
																	AD9910_RAM_DIR_PHA_R();                                         // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÖ±½Ó×ª»»Ä£Ê½-----ÏàÎ»				
																	break;
																default:
																	break;
															}
															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
															Delay_50ms(10);
															Set_Profile(1);			//¶ÁÈ¡profile1µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(2);			//¶ÁÈ¡profile2µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(3);			//¶ÁÈ¡profile3µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(4);			//¶ÁÈ¡profile4µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"              " );
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"        " );						
															LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"        " );
															LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"        " );
															LCD_WriteString (    30,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"        " );
															flag = 1;
															key_numb = 5;
															posy = 40;
															break;
														default:
															break;
													}
												}
											}else if(posy == 70)
											{
												posy = 40;	
												while(key_numb != 5)
												{		
													LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÉÏÐ±ÆÂÄ£Ê½" );
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 130)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 100;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_Fre_W();																							// Ð´ÆµÂÊµÄRAM Ð´Èëprofie0-4·Ö±ð2M 4M 6M 8M 10M
																	AD9910_RAM_RAMP_UP_TWO_Fre_R();																	// »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÉÏÐ±ÆÂÄ£Ê½-----ÆµÂÊ       £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿ØÖÆ£©
															
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_AMP_W();                                             // Ð´·ù¶ÈµÄRAM  Ð´Èëprofile0 740mv×óÓÒ  profile1 90mv×óÓÒ profile2 180mv×óÓÒ  profile3 370mv×óÓÒ profile4 740mv×óÓÒ
																	AD9910_RAM_RAMP_UP_TWO_AMP_R();                                 // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÉÏÐ±ÆÂÄ£Ê½-----·ù¶È       £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿ØÖÆ£©
																	break;
																case 10:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_Pha_W();                                             // Ð´ÏàÎ»µÄRAM
																	AD9910_RAM_RAMP_UP_TWO_PHA_R();                                 // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÉÏÐ±ÆÂÄ£Ê½-----ÏàÎ»       £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿ØÖÆ£©
																	break;
																default:
																	break;
															}
															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
															Delay_50ms(10);
															Set_Profile(1);			//¶ÁÈ¡profile1µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(2);			//¶ÁÈ¡profile2µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(3);			//¶ÁÈ¡profile3µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(4);			//¶ÁÈ¡profile4µÄÊý¾Ý
															Delay_50ms(10);
															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
													  	flag = 1;
															key_numb = 5;
															posy = 40;
															break;
														default:
															break;
													}
												}
											}else if(posy == 100)
											{
												posy = 40;	
												while(key_numb != 5)
												{		
													LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Ë«ÏòÐ±ÆÂÄ£Ê½" );
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 130)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 100;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_Fre_W();                                             // Ð´ÆµÂÊµÄRAM Ð´Èëprofie0-4·Ö±ð2M 4M 6M 8M 10M
																	AD9910_RAM_BID_RAMP_Fre_R();                                    // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªË«ÏòÐ±ÆÂÄ£Ê½-----ÆµÂÊ   
																
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_AMP_W();                                             // Ð´·ù¶ÈµÄRAM  Ð´Èëprofile0 740mv×óÓÒ  profile1 90mv×óÓÒ profile2 180mv×óÓÒ  profile3 370mv×óÓÒ profile4 740mv×óÓÒ
																	AD9910_RAM_BID_RAMP_AMP_R();                                    // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªË«ÏòÐ±ÆÂÄ£Ê½-----·ù¶È
																	break;
																case 10:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_Pha_W();                                             // Ð´ÏàÎ»µÄRAM
																	AD9910_RAM_BID_RAMP_PHA_R();                                    // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªË«ÏòÐ±ÆÂÄ£Ê½-----ÏàÎ»
																	break;
																default:
																	break;
															}
//															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
//															Delay_50ms(10);
//															Set_Profile(1);			//¶ÁÈ¡profile1µÄÊý¾Ý
//															Delay_50ms(10);
//															Set_Profile(2);			//¶ÁÈ¡profile2µÄÊý¾Ý
//															Delay_50ms(10);
//															Set_Profile(3);			//¶ÁÈ¡profile3µÄÊý¾Ý
//															Delay_50ms(10);
//															Set_Profile(4);			//¶ÁÈ¡profile4µÄÊý¾Ý
//															Delay_50ms(10);
//															Set_Profile(0);     //¶ÁÈ¡profile0µÄÊý¾Ý  profileµÄÊý¾ÝÔÚAD9910.cÎÄ¼þÖÐ  
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															flag = 1;
															key_numb = 5;
															posy = 40;
															break;
														default:
															break;
													}
												}
											}else if(posy == 130)
											{
												posy = 40;	
												while(key_numb != 5)
												{		
													LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Á¬ÐøË«ÏòÐ±ÆÂÄ£Ê½" );
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 130)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 100;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_Fre_W();                                             // Ð´ÆµÂÊµÄRAM  Ð´Èëprofie0 2M 4M 6M 8M 10M
																	AD9910_RAM_CON_BID_RAMP_Fre_R();                                // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøË«ÏòÐ±ÆÂÄ£Ê½-----ÆµÂÊ
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_AMP_W();                                             // Ð´·ù¶ÈµÄRAM  Ð´Èëprofie0 90mv 180mv 370mv 740mv×óÓÒ
																	AD9910_RAM_CON_BID_RAMP_AMP_R();                                // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøË«ÏòÐ±ÆÂÄ£Ê½-----·ù¶È
																	break;
																case 10:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_Pha_W();                                             // Ð´ÏàÎ»µÄRAM
																	AD9910_RAM_CON_BID_RAMP_PHA_R();                                // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøË«ÏòÐ±ÆÂÄ£Ê½-----ÏàÎ»
																	break;
																default:
																	break;
															}
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
															flag = 1;		
															key_numb = 5;														
															posy = 40;
															break;
														default:
															break;
													}
												}
											}else if(posy == 160)
											{
												posy = 40;	
												while(key_numb != 5)
												{		
													LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Á¬ÐøÑ­»·Ä£Ê½"  );
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè	
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 130)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 100;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_Fre_W();                                             // Ð´ÆµÂÊµÄRAM Ð´Èëprofie0 2M 4M 6M 8M 10M
																	AD9910_RAM_CON_RECIR_Fre_R();                                   // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøÑ­»·Ä£Ê½-----ÆµÂÊ
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_AMP_W();                                             // Ð´·ù¶ÈµÄRAM Ð´Èëprofie0 90mv 180mv 370mv 740mv×óÓÒ
																	AD9910_RAM_CON_RECIR_AMP_R();                                   // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøÑ­»·Ä£Ê½-----·ù¶È     £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿ØÖÆ£©
																	break;
																case 10:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(1000000);                                 // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_RAM_Pha_W();                                             // Ð´ÏàÎ»µÄRAM
																	AD9910_RAM_CON_RECIR_PHA_R();                                   // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøÑ­»·Ä£Ê½-----ÏàÎ»
																	break;
																default:
																	break;
															}
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
														  flag = 1;
															key_numb = 5;
															posy = 40;
															break;
														default:
															break;
													}
												}
											}
											else if(posy == 190)
											{
												posy = 40;	
												while(key_numb != 5)
												{		
													LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.Èý½Ç²¨" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·½²¨" );
													key_numb = Key_Scan();															// °´¼üÉ¨Ãè
													switch(key_numb)
													{
														case 1:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy += 30;
															if(posy == 100)
															{
																posy = 40;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 2:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
															posy -= 30;
															if(posy == 10)
															{
																posy = 70;
															}
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
															break;
														case 3:
															switch(posy/10)
															{
																case 4:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(0);                                       // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_WAVE_RAM_AMP_W(2);                                       // Ð´·ù¶ÈµÄRAM
																	AD9910_RAM_CON_RECIR_AMP_R();                                   // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøÑ­»·Ä£Ê½-----·ù¶È     £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿
																	break;
																case 7:
																	AD9910_RAM_Init();																							// RAM¹¦ÄÜ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Init();                                       // ÔØ²¨ÆµÂÊ³õÊ¼»¯
																	AD9910_RAM_ZB_Fre_Set(0);                                       // ÉèÖÃÔØ²¨ÆµÂÊ
																	AD9910_WAVE_RAM_AMP_W(3);                                       // Ð´·ù¶ÈµÄRAM
																	AD9910_RAM_CON_RECIR_AMP_R();                                   // »Ø·ÅRAM¡ª¡ª¡ª¡ª¡ª¡ªÁ¬ÐøÑ­»·Ä£Ê½-----·ù¶È     £¨CFR1¼Ä´æÆ÷ÖÐCFR1[20:17]ÄÚ²¿¿
																	break;
																default:
																	break;
															}
															break;
														case 4:
															LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
														  flag = 1;
															key_numb = 5;
															posy = 40;
															break;
														default:
															break;
														}
													}
											}
											break;
									case 4:
										LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
										LCD_WriteString (   100,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"           " );
										LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										LCD_WriteString (    30,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										LCD_WriteString (    30,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										LCD_WriteString (    30,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
										posy = 40;
										break;
									default:
										break;
									}
								}
								if(key_numb != 4)
								{
//									Set_Profile(0);    //¶ÁÈ¡RAM profile0µÄÊý¾Ý
//									Delay_50ms(2);
//									Set_Profile(1);    //¶ÁÈ¡RAM profile1µÄÊý¾Ý
//									Delay_50ms(2);
//									Set_Profile(2);    //¶ÁÈ¡RAM profile2µÄÊý¾Ý
//									Delay_50ms(2);
//									Set_Profile(3);    //¶ÁÈ¡RAM profile3µÄÊý¾Ý
//									Delay_50ms(2);
//									Set_Profile(4);    //¶ÁÈ¡RAM profile4µÄÊý¾Ý
//									Delay_50ms(2);
//									Set_Profile(0);    //¶ÁÈ¡RAM profile0µÄÊý¾Ý
								}
							}
							break;
						case 10:   //Êý×ÖÐ±ÆÂµ÷ÖÆÄ£Ê½
							posy = 40;
							while(key_numb != 4)
							{			
									LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
									LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
									LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
									LCD_WriteString (    0,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
									LCD_WriteString (    0,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
									LCD_WriteString (    0,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                 " );																					
									while(key_numb != 4)
									{	
										LCD_WriteString (   60,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Êý×ÖÐ±ÆÂµ÷ÖÆÄ£Ê½" );
										LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
										LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
										LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
										LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
										key_numb = Key_Scan();		// °´¼üÉ¨Ãè
										switch(key_numb)
										{
											case 1:
												LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );		
												posy += 30;
												if(posy == 130)
												{
													posy = 40;
												}
												LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );		
												break;
											case 2:
												LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );		
												posy -= 30;
												if(posy == 10)
												{
													posy = 100;
												}
												LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );		
												break;
											case 3:
												LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
												LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
												LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
												LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
												posx1 = 100;
												posy1 = 40;
												switch(posy/10)
												{
													case 4:			
														LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼ÆµÂÊ:" );
														LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÖÕÖ¹ÆµÂÊ:" );	
														break;
													case 7:
														LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼·ù¶È:" );
														LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÖÕÖ¹·ù¶È:" );
														break;
													case 10:
														LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼ÏàÎ»:" );
														LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÖÕÖ¹ÏàÎ»:" );
														break;
													default:
														break;
												}
												LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÉÏÉý²½½ø´óÐ¡:" );
												LCD_WriteString (    0,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÏÂ½µ²½½ø´óÐ¡:" );
												LCD_WriteString (    0,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÉÏÉý²½½øÊ±¼ä:" );
												LCD_WriteString (    0,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÏÂ½µ²½½øÊ±¼ä:" );
												LCD_SetFont(&Font16, &Font16_CN);
												LCD_WriteString (     40,   220,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"k1ÏòÏÂ  k2ÏòÉÏ  k3Î»È¨  k4È·¶¨" );
												LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN);
												posx1 += 10; 
												LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"_" );
												Delay_50ms(10);
												LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
												Delay_50ms(10);
												while(key_numb != 5)
												{
													key_numb = Key_Scan();		// °´¼üÉ¨Ãè
													switch(key_numb)
													{
														case 1:
															if(num[pos_fre] < 9)
															{
																num[pos_fre]++;
															}else
															{
																num[pos_fre] = 0;
															}
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)" " );
															LCD_WriteNumChar(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t )num[pos_fre]);
															break;
														case 2:
															if(num[pos_fre] > 0)
															{
																num[pos_fre]--;
															}else
															{
																num[pos_fre] = 9;
															}
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)" " );
															LCD_WriteNumChar(    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t )num[pos_fre]);
															break;
														case 3:
															pos_fre++;
															posx1 += 15;
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"_" );
															Delay_50ms(10);
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
															Delay_50ms(10);
															break;
														case 4:
															if(posy1 == 40)
															{
																ad9910.start_parm = get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 110;
															}else if(posy1 == 70)
															{
																ad9910.stop_parm	= get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 154;
															}else if(posy1 == 100){
																ad9910.rising_step = get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 154;
															}else if(posy1 == 130)
															{
																ad9910.falling_step = get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 154;
															}else if(posy1 == 160)
															{
																ad9910.rising_time = get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 154;
															}else if(posy1 == 190)
															{	
																ad9910.falling_time = get_data(pos_fre, num);
																clr_data(pos_fre, num);
																posx1 = 110;	
																LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
																LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
																LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
																LCD_WriteString (    0,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
																LCD_WriteString (    0,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
																LCD_WriteString (    0,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );	
																LCD_SetFont(&Font16, &Font16_CN);
																LCD_WriteString (     40,   220,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"k1ÏòÏÂ  k2ÏòÉÏ  k3È·¶¨  k4·µ»Ø" );
																LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN);
																key_numb = 5;
															}
															pos_fre = 0;	
															if(posy1 == 190)
															{
															
																if(posy == 40)
																{
																	AD9910_DRG_Fre_Init();																															// Êý×ÖÐ±ÆÂ³õÊ¼»¯------------ÆµÂÊ
																	// ÉèÖÃ£¨ÉÏÏÞÆµÂÊ£¬ÏÂÏÞÆµÂÊ£¬ÆµÂÊ¼Ó²½½ø£¬ÆµÂÊ¼õ²½½ø£¬ÆµÂÊ¼Ó²½½øÊ±¼ä£¬ÆµÂÊ¼õ²½½øÊ±¼ä£©
																	AD9910_DRG_Freq_set(ad9910.stop_parm, ad9910.start_parm,   ad9910.rising_step,  ad9910.falling_step, ad9910.rising_time, ad9910.falling_time);		
																	AD9910_DRC_Set;
																}
																else if(posy == 70)
																{
																	AD9910_Singal_Profile_Init();																	// PROFILE³õÊ¼»¯
																	AD9910_Singal_Profile_Set(0, 1000000, 0, 0);									// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©

																	AD9910_DRG_AMP_Init();
																	// ÉèÖÃ£¨ÉÏÏÞ·ù¶È£¬ÏÂÏÞ·ù¶È£¬·ù¶È¼Ó²½½ø£¬·ù¶È¼õ²½½ø£¬·ù¶È¼Ó²½½øÊ±¼ä£¬·ù¶È¼õ²½½øÊ±¼ä£©
																	AD9910_DRG_Amp_Set(ad9910.stop_parm, ad9910.start_parm,   ad9910.rising_step,  ad9910.falling_step, ad9910.rising_time, ad9910.falling_time);																	
																}
																else if(posy == 100)
																{			
																	AD9910_Singal_Profile_Init();																    	// PROFILE³õÊ¼»¯
																	AD9910_Singal_Profile_Set(0, 1000000, 16383, 0);									// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©
																	
																	AD9910_DRG_Pha_Init();
																	// ÉèÖÃ£¨ÉÏÏÞÏàÎ»£¬ÏÂÏÞÏàÎ»£¬ÏàÎ»¼Ó²½½ø£¬ÏàÎ»¼õ²½½ø£¬ÏàÎ»¼Ó²½½øÊ±¼ä£¬ÏàÎ»¼õ²½½øÊ±¼ä£©
																	AD9910_DRG_Pha_Set(ad9910.stop_parm, ad9910.start_parm,   ad9910.rising_step,  ad9910.falling_step, ad9910.rising_time, ad9910.falling_time);  																	
																}
															}
															
															posy1 += 30;
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"_" );
															Delay_50ms(10);
															LCD_WriteString (    posx1,    posy1,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
															Delay_50ms(10);
															break;
														default:		
																					
															break;
													}
												}
											
												break;
											case 4:
												break;
											default:
												break;
										}
										if(posy == 40)
										{
										
										}else if(posy == 70)
										{
											AD9910_DRC_Set;	
											Delay_250us(50);
											AD9910_DRC_Clr;
											Delay_250us(50);	
										}else if(posy == 100)
										{
										}										
									}
								}
								AD9910_DRC_Set;	
								break;
						case 13:    //oskÄ£Ê½
							LCD_WriteString (   30,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"OSKÄ£Ê½" );
							AD9910_Osk_Init();																							// OSK¹¦ÄÜ³õÊ¼»¯
							AD9910_Singal_Profile_Set(0,1000000,0 ,0);											// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©-----Ö»ÒªÉèÖÃÍ¨µÀºÍÆµÂÊºÍÏàÎ»¾ÍºÃ£¬·ù¶È²»ÒªÉèÖÃ
							AD9910_Osk_Set();
							while(key_numb != 4)
							{
								key_numb = Key_Scan();
								if(key_numb == 1)						// K1°´¼ü°´ÏÂ, ´ò¿ªOSKÊä³ö
								{
									AD9910_OSK_Set;	
								}else if(key_numb == 2)			// K2°´¼ü°´ÏÂ, ¹Ø±ÕOSKÊä³ö
								{
									AD9910_OSK_Clr;	
								}else if(key_numb == 4)			// K4°´¼ü°´ÏÂ, ÍË³ö
								{
									LCD_WriteString (   30,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                         " );
								}  
							}		

							break;
						case 16:     //²¢ÐÐÊý¾Ý¶Ë¿Úµ÷ÖÆÄ£Ê½
								LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
								LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
								LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
								LCD_WriteString (    0,   130,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
								LCD_WriteString (    0,   160,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                     " );
								LCD_WriteString (    0,   190,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                 " );		
								LCD_WriteString (   60,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"²¢ÐÐÊý¾Ý¶Ë¿Úµ÷ÖÆÄ£Ê½" );
								posy = 40;
							
								while(key_numb != 4)
								{		
									LCD_WriteString (   60,    10,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"²¢ÐÐÊý¾Ý¶Ë¿Úµ÷ÖÆÄ£Ê½" );
									LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );						
									LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"1.ÆµÂÊ" );
									LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"2.·ù¶È" );
									LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"3.ÏàÎ»" );
									key_numb = Key_Scan();		// °´¼üÉ¨Ãè
									switch(key_numb)
									{
										case 1:
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											posy += 30;					
											if(posy == 130)
											{
												posy = 40;
											}	
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
											break;
										case 2:
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											posy -= 30;					
											if(posy == 10)
											{
												posy = 100;
											}	
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
											break;
										case 3:
											LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
											LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
											LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"         " );
											LCD_WriteString (  posx,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
											posx1 = 100;
											posy1 = 40;
											switch(posy/10)
											{
												case 4:
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼ÆµÂÊ:" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Æ«ÒÆÆµÂÊ:" );
													LCD_WriteString (    30,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÔöÒæ:" );
													LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"           " );
													LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_1]);
													LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"           " );
													LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  offset_fre[pos_2]);
													LCD_WriteString (    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
													LCD_WriteNumLong(    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  gain[pos_3]);
													LCD_WriteString (     0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
													while(key_numb != 5)
													{			
														key_numb = Key_Scan();		// °´¼üÉ¨Ãè
														switch(key_numb)
														{
															case 1:
																if(posy == 40)
																{			
																	pos_1 ++;
																	if(pos_1 == (sizeof(fre_data)/sizeof(uint32_t)-1))
																	{
																		pos_1 = 0;
																	}	
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_1]);
																}else if(posy == 70)
																{
																	pos_2 ++;
																	if(pos_2 == 9)
																	{
																		pos_2 = 0;
																	}	
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  offset_fre[pos_2]);
																}else if(posy == 100)
																{
																	pos_3 ++;
																	if(pos_3 == 16)
																	{
																		pos_3 = 0;
																	}	
																	LCD_WriteString (    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  gain[pos_3]);
																}
																break;
															case 2:
																if(posy == 40)
																{
																	if(pos_1 == 0)
																	{
																		pos_1 = (sizeof(fre_data)/sizeof(uint32_t)-1);
																	}
																	pos_1 --;
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  fre_data[pos_1]);
																}else if(posy == 70)
																{
																	if(pos_2 == 0)
																	{
																		pos_2 = 9;
																	}
																	pos_2 --;
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  offset_fre[pos_2]);				
																}else if(posy == 100)
																{
																	if(pos_3 == 0)
																	{
																		pos_3 = 16;
																	}
																	pos_3 --;
																	LCD_WriteString (    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    90,    100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  gain[pos_3]);				
																}
																break;
															case 3:
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
																posy += 30;
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
																if(posy >= 130)
																{
																	LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
																	Freq_convert(fre_data[pos_1]);				// »ù´¡ÆµÂÊ   
																	AD9910_Init_Sin(gain[pos_3]);
																	Par_mod(2, offset_fre[pos_2]);				// µ÷ÖÆÆµÂÊ
																	
																}
																break;
															case 4:
																LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"                  " );
																LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																posy = 40;
																key_numb = 5;
																break;
															default:
																break;
														}
													}
													break;
												case 7:
													posy = 40;
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼·ù¶È:" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Æ«ÒÆ·ù¶È:" );
													LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
													LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_4]);
													LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
													LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_5]);
													LCD_WriteString (     0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
													while(key_numb != 5)
													{			
														key_numb = Key_Scan();		// °´¼üÉ¨Ãè
														switch(key_numb)
														{
															case 1:
																if(posy == 40)
																{
																	pos_4 ++;
																	if(pos_4 == 13)
																	{
																		pos_4 = 0;
																	}	
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_4]);
																}else if(posy == 70)
																{
																	pos_5 ++;
																	if(pos_5 == 13)
																	{
																		pos_5 = 0;
																	}	
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_5]);
																}
																break;
															case 2:
																if(posy == 40)
																{
																	if(pos_4 == 0)
																	{
																		pos_4 = 18;
																	}
																	pos_4 --;
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_4]);
																}else if(posy == 70)
																{
																	if(pos_5 == 0)
																	{
																		pos_5 = 18;
																	}
																	pos_5 --;
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  amp_data[pos_5]);
																}
																break;
															case 3:
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
																posy += 30;
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
																if(posy >= 100)
																{
																	LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
																	AD9910_Singal_Profile_Init();																		// µ¥ÒôPROFILE³õÊ¼»¯
																	AD9910_Singal_Profile_Set(0, 1000000, amp_data[pos_4], 0);									// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©
																	Par_mod(0, amp_data[pos_5]);				// µ÷ÖÆÆµÂÊ	
																}
																break;
															case 4:
																LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );					
																key_numb = 5;
																posy = 40;
																break;
															default:
																break;
														}
													}
													break;
												case 10:
													posy = 40;	
													LCD_WriteString (    30,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"ÆðÊ¼ÏàÎ»:" );
													LCD_WriteString (    30,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"Æ«ÒÆÏàÎ»:" );
													LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
													LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_6]);
													LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
													LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_7]);												
													LCD_WriteString (     0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );	
													while(key_numb != 5)
													{			
														key_numb = Key_Scan();		// °´¼üÉ¨Ãè
														switch(key_numb)
														{
															case 1:
																if(posy == 40)
																{
																	pos_6 ++;
																	if(pos_6 == 6)
																	{
																		pos_6 = 0;
																	}	
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_6]);
																}else if(posy == 70)
																{
																	pos_7 ++;
																	if(pos_7 == 6)
																	{
																		pos_7 = 0;
																	}	
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_7]);
																}
																break;
															case 2:
																if(posy == 40)
																{
																	if(pos_6 == 0)
																	{
																		pos_6 = 6;
																	}
																	pos_6 --;
																	LCD_WriteString (    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_6]);
																}else if(posy == 70)
																{
																	if(pos_7 == 0)
																	{
																		pos_7 = 6;
																	}
																	pos_7 --;
																	LCD_WriteString (    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"          " );
																	LCD_WriteNumLong(    140,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  pha_data[pos_7]);
																}
																break;
															case 3:
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );	
																posy += 30;
																LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"->" );
																if(posy >= 100)
																{
																	LCD_WriteString (  0,  posy,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"  " );
																	AD9910_Singal_Profile_Init();																		// µ¥ÒôPROFILE³õÊ¼»¯
																	AD9910_Singal_Profile_Set(0, 1000000, 10230, pha_data[pos_6]);									// ÉèÖÃÕýÏÒ²¨ÐÅÏ¢ £¨ Í¨µÀ£¨0~7£© £¬ ÆµÂÊ £¬·ù¶È £¬ ÏàÎ» £©
																	Par_mod(1, amp_data[pos_7]);				// µ÷ÖÆÆµÂÊ	
																}
																break;
															case 4:
																LCD_WriteString (    0,    40,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																LCD_WriteString (    0,    70,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
																LCD_WriteString (    0,   100,  LCD_COLOR_RED,  LCD_COLOR_WHITE,  (uint8_t *)"             " );
															  key_numb = 5;
																posy = 40;
																break;
															default:
																break;
														}
													}
													break;
												default:
													break;
											}
	
											break;
										case 4:
											break;
										default:
											break;
									}
								}
							break;
						default:
							break;
						}
					}
				break;
			case 4:
		
				break;
			default:
				
				break;
			}
		
	}
}
	

//-----------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------
