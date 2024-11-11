/**
  ******************************************************************************
  * @file    stm324xg_eval_lcd.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    11-January-2013
  * @brief   This file contains all the functions prototypes for the stm324xg_eval_lcd.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LZ_LCD_H
#define __LZ_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include <stm32f10x.h>
#include <stdio.h>
#include <stm32f10x_fsmc.h>
#include "fonts.h"


/**
  * @brief  LCD Registers
  */
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2B
#define LCD_REG_45            0x2D
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3A
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6A
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8B
#define LCD_REG_140           0x8C
#define LCD_REG_141           0x8D
#define LCD_REG_143           0x8F
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9A
#define LCD_REG_157           0x9D
#define LCD_REG_192           0xC0
#define LCD_REG_193           0xC1
#define LCD_REG_229           0xE5

/**
  * @brief  LCD color
  */
#define LCD_COLOR_WHITE          0xFFFF
#define LCD_COLOR_BLACK          0x0000
#define LCD_COLOR_GREY           0xF7DE
#define LCD_COLOR_BLUE           0x001F
#define LCD_COLOR_BLUE2          0x051F
#define LCD_COLOR_RED            0xF800
#define LCD_COLOR_MAGENTA        0xF81F
#define LCD_COLOR_GREEN          0x07E0
#define LCD_COLOR_CYAN           0x7FFF
#define LCD_COLOR_YELLOW         0xFFE0


// LCD 端口分配   


//-----------------------------------------------------------------
// 位操作
//-----------------------------------------------------------------
#define LCD_CS_Set 	( GPIO_SetBits   ( GPIOD, GPIO_Pin_7  )) 
#define LCD_CS_Clr 	( GPIO_ResetBits ( GPIOD, GPIO_Pin_7  ))

#define LCD_RST_Set	( GPIO_SetBits   ( GPIOD, GPIO_Pin_12 )) 
#define LCD_RST_Clr	( GPIO_ResetBits ( GPIOD, GPIO_Pin_12 ))

#define LCD_RS_Set 	( GPIO_SetBits   ( GPIOD, GPIO_Pin_11 )) 
#define LCD_RS_Clr 	( GPIO_ResetBits ( GPIOD, GPIO_Pin_11 ))

#define LCD_WR_Set 	( GPIO_SetBits   ( GPIOD, GPIO_Pin_5  )) 
#define LCD_WR_Clr 	( GPIO_ResetBits ( GPIOD, GPIO_Pin_5  ))

#define LCD_RD_Set 	( GPIO_SetBits   ( GPIOD, GPIO_Pin_4  ))
#define LCD_RD_Clr 	( GPIO_ResetBits ( GPIOD, GPIO_Pin_4  ))

#define LCD_LED_OFF 	( GPIO_SetBits   ( GPIOD, GPIO_Pin_13 ))
#define LCD_LED_ON	  ( GPIO_ResetBits ( GPIOD, GPIO_Pin_13 ))


   
// 显示方向 
#define DISPLAY_DIR_D          2                                       // 下
#define DISPLAY_DIR_U          1                                       // 上
#define DISPLAY_DIR_L          3                                       // 左
#define DISPLAY_DIR_R          4                                       // 右

#define LCD_DIR                DISPLAY_DIR_L                          // 默认显示方向

// 画虚线时间隔点个数
#define DOT		                 3

// ST7789V ID
#define ST7789V_ID              0x8552
// SPFD5408 ID
#define SPFD5408_ID             0x5408

/* Note: LCD /CS is NE1 - Bank 1 of NOR/PSRAM Bank 1 A16<->LCD_RS*/
#define LCD_BASE               ((uint32_t)(0x60000000 | 0x0001FFFE ))
#define LCD                    ((LCD_TypeDef *) LCD_BASE)
#define MAX_POLY_CORNERS       200



// @defgroup LCD_Exported_Types
typedef struct
{
  int16_t X;
  int16_t Y;
} Point, * pPoint;

// @defgroup LCD_Private_TypesDefinitions
typedef struct
{
  __IO uint16_t LCD_REG;
  __IO uint16_t LCD_RAM;
} LCD_TypeDef;


//LCD重要参数集
typedef struct
{
  uint16_t WIDTH;                                                      // LCD 显示宽度
  uint16_t HEIGHT;                                                     // LCD 显示高度
  uint8_t  DISPLAY_DIR;                                                // 显示方向
  uint16_t SCAN_DIR;                                                   // 扫描方向
  uint16_t REG_ID;                                                     // LCD ID
  uint16_t REG_GRAM;                                                   // 读写显存寄存器
  uint16_t REG_XPOSITION;                                              // 设置x坐标寄存器
  uint16_t REG_YPOSITION;                                              // 设置y坐标寄存器
} LCD__DevTypeDef;


// 全局变量
extern __IO LCD__DevTypeDef LCD_Dev;                                   //管理LCD重要参数
extern sFONT                   *LCD_Currentfonts;                      // 显示字符字体信息
extern sFONT_CN                *LCD_Currentfonts_CN;                   // 显示中文字体信息

// @brief LCD default font
#define LCD_DEFAULT_FONT         Font24                             // 指定显示字符字体
#define LCD_DEFAULT_FONT_CN      Font24_CN                          // 指定显示中文字体

// @brief  LCD Direction
// @brief  LCD Size (Width and Height)
#define LCD_PIXEL_WIDTH          0x0140
#define LCD_PIXEL_HEIGHT         0x00F0

//#define ASSEMBLE_RGB(R ,G, B)    ((((R)& 0xF8) << 8) | (((G) & 0xFC) << 3) | (((B) & 0xF8) >> 3))


void LCD_Init(void);

void LCD_SetColors(
  __IO uint16_t _TextColor, 
  __IO uint16_t _BackColor
);

void LCD_GetColors(
  __IO uint16_t *_TextColor, 
  __IO uint16_t *_BackColor
);

void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_Clear(uint16_t Color);

void LCD_SetCursor(
  uint16_t Xpos, 
  uint16_t Ypos
);

void LCD_SetFont(sFONT *fonts, sFONT_CN *fonts_cn);

void LCD_SetDisplayWindow(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height, 
  uint16_t Width
);

void LCD_WindowModeDisable(void);

void LCD_DrawHLine(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Length
);

void LCD_DrawDotHLine(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Length
);

void LCD_DrawVLine(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height
);

void LCD_DrawDotVLine(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height
);

void LCD_DrawRect(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height, 
  uint16_t Width
);

void LCD_DrawMonoPict(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height, 
  uint16_t Width, 
  const uint8_t *Pict
);
  
void LCD_DrawRgbPict(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Height, 
  uint16_t Width, 
  const uint8_t *Pict
);

void LCD_DrawFullRect(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint16_t Width, 
  uint16_t Height
);

void LCD_DrawChar(
  uint16_t Xpos, 
  uint16_t Ypos, 
  const uint8_t *c
);
  
void LCD_DisplayChar(
  uint16_t Line, 
  uint16_t Column, 
  uint8_t Ascii
);

void LCD_DrawCN(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint8_t *c
);

void LCD_DisplayChineseChar(
  uint16_t Xpos, 
  uint16_t Ypos, 
  uint8_t ChineseCode[2]
);

void LCD_DisplayString(
  uint16_t Xpos,
  uint16_t Ypos,
  uint8_t *ptr
);

void PutPixel (
  int16_t x, 
  int16_t y 
);

void LCD_SetPoint(
  uint16_t Xpos,                                                    // X轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t PointColor                                               // 像素点颜色
);

void LCD_WriteString(
  uint16_t Xpos,                                                    // X轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  *sp                                                      // 指向字符串的字符指针
);

void LCD_WriteChinese24x24string(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  *sp                                                      // 中文字符指针
);

void LCD_WriteNumChar (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  udata                                                    // 显示的数值
);

void LCD_WriteNumInt (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint16_t udata                                                    // 显示的数值
);

void LCD_WriteNumLong (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint32_t udata                                                    // 显示的数值
);

void LCD_Line_H(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t Length,                                                  // 长度
  uint16_t LineColor                                                // 线条颜色
);

void LCD_DotLine_H(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t Length,                                                  // 长度
  uint16_t LineColor,                                               // 线条颜色
  uint16_t BackColor_                                               // 背景颜色
);

void LCD_Line_V(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 长度
  uint16_t LineColor                                                // 线条颜色
);

void LCD_DotLine_V(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 长度
  uint16_t LineColor,                                               // 线条颜色
  uint16_t BackColor_                                               // 背景颜色
);

void LCD_LineBox(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t Width,                                                   // 宽度
  uint16_t High,                                                    // 高度
  uint16_t LineColor                                                // 线条颜色
);

void LCD_FillBox(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 高度
  uint16_t Width,                                                   // 宽度
  uint16_t FillColor                                                // 线条颜色
);


/**
  * @}
  */

/** @defgroup
  * @{
  */
void LCD_WR_REG(uint8_t RegIndex);
void LCD_WR_DATA(uint16_t RegValue);

void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);

void LCD_WriteRAM_Prepare(void);

void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadRAM(void);

void LCD_Display_Dir ( uint8_t Dir );

void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);

/**
  * @}
  */

/** @defgroup
  * @{
  */
void LCD_CtrlLinesConfig(void);
void LCD_FSMCConfig(void);
/**
  * @}
  */
/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* __LZ_LCD_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
