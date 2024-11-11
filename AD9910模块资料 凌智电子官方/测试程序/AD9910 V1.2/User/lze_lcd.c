//-----------------------------------------------------------------
// 程序描述:
//      基于FSMC驱动SPFD5408/ST7789V内核的2.8寸液晶屏驱动子程序
// 作    者: 凌智电子
// 开始日期: 2020-05-01
// 完成日期: 2020-05-05
// 修改日期:
// 当前版本: V2.0
// 历史版本:
//  - V1.0: 2.8寸TFT液晶显示 (SPFD5408)
//  - V2.0: 2.8寸TFT液晶显示 (SPFD5408 / ST7789V)
// 调试工具: 凌智STM32+FPGA电子系统设计开发板、2.8寸液晶屏、LZE_ST_LINK2
// 说    明:
//
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// 头文件包含
//-----------------------------------------------------------------
#include "lze_lcd.h"
#include "fonts.h"
#include "Delay.h"

// 字符字模
#include "./Fonts/font8.c"
#include "./Fonts/font12.c"
#include "./Fonts/font16.c"
#include "./Fonts/font20.c"
#include "./Fonts/font24.c"
// 中文字模
#include "./Fonts/GB1616.c"
#include "./Fonts/GB2424.c"
#include "./Fonts/GB3232.c"

// 宏定义
#define POLY_Y(Z)               ((int32_t)((Points + Z)->X))
#define POLY_X(Z)               ((int32_t)((Points + Z)->Y))
#define ABS(X)                  ((X) > 0 ? (X) : -(X))                      // 取绝对值
#define LCD_Delay(t_ms)         Delay_1ms(t_ms)                             // LCD 初始化延时函数 重定向
//#define LCD_Delay(t_ms)         HAL_Delay(t_ms)                             // LCD 初始化延时函数 重定向


//// 全局变量
sFONT                         *LCD_Currentfonts;                          // 显示字符字体信息
sFONT_CN                      *LCD_Currentfonts_CN;                       // 显示中文字体信息

__IO LCD__DevTypeDef           LCD_Dev;                                    //管理 LCD 重要参数
__IO uint16_t                  TextColor = 0x0000,
                               BackColor = 0xFFFF;                         // 字体颜色 和 背景颜色




//-----------------------------------------------------------------
// 初始化程序区
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// void LCD_CtrlLinesConfig(void)
//-----------------------------------------------------------------
//
// 函数功能: LCD MCU接口引脚配置
// 入口参数: 无
// 返回参数: 无
// 全局变量: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void LCD_CtrlLinesConfig(void)
{
  /** FSMC GPIO Configuration
  PD14   -----> FSMC_D0
  PD15   -----> FSMC_D1
  PD0   ------> FSMC_D2
  PD1   ------> FSMC_D3
  PE7   ------> FSMC_D4
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE10   -----> FSMC_D7
  PE11   -----> FSMC_D8
  PE12   -----> FSMC_D9
  PE13   -----> FSMC_D10
  PE14   -----> FSMC_D11
  PE15   -----> FSMC_D12
  PD8   ------> FSMC_D13
  PD9   ------> FSMC_D14
  PD10   -----> FSMC_D15
  PD11   -----> FSMC_A16
  PD4   ------> FSMC_NOE
  PD5   ------> FSMC_NWE
  PD7   ------> FSMC_NE1
  */
  GPIO_InitTypeDef GPIO_InitStructure;

  // 使能IO口时钟
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | 
                          RCC_APB2Periph_GPIOE, ENABLE);  
  // 部分数据线、部分控制线配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0   | GPIO_Pin_1  |  // D2,D3 
                                GPIO_Pin_4   | GPIO_Pin_5  | // RD,WR
                                GPIO_Pin_7   | GPIO_Pin_8  | // CS,D13
                                GPIO_Pin_9   | GPIO_Pin_10 | // D14,D15
                                GPIO_Pin_11  | GPIO_Pin_14 | // RS,D0
                                GPIO_Pin_15;                // D1
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  // 推挽复用输出
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;             
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  // 部分数据线配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7  | GPIO_Pin_8  |  // D4,D5
                                GPIO_Pin_9  | GPIO_Pin_10 | // D6,D7
                                GPIO_Pin_11 | GPIO_Pin_12 | // D8,D9
                                GPIO_Pin_13 | GPIO_Pin_14 | // D10,D11
                                GPIO_Pin_15;                // D12
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // 推挽复用输出
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  // 液晶复位，背光灯引脚配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;  // RST,LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          // 推挽输出
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  LCD_LED_OFF;                                              // LED关
}

//-----------------------------------------------------------------
// void LCD_FSMCConfig(void)
//-----------------------------------------------------------------
//
// 函数功能: LCD FMC控制器配置 LCD(Parallel mode)
// 入口参数: 无
// 返回参数: 无
// 全局变量: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void LCD_FSMCConfig(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
  // 打开FSMC时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);  
  // 打开复用功能时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  

  // FSMC_Bank1_NORSRAM1 时间配置
  // 地址建立时间期限   
  p.FSMC_AddressSetupTime = 0x02;                                                              
  // 地址的持续时间 
  p.FSMC_AddressHoldTime = 0x00;                      
  // 设定数据时间期限
  p.FSMC_DataSetupTime = 0x05;                        
   // 总线转向时间
  p.FSMC_BusTurnAroundDuration = 0x00;              
  // CLK时钟输出信号的HCLK周期数表示时间
  p.FSMC_CLKDivision = 0x00;      
  // 指定在获得第一个数据前的时钟周期
  p.FSMC_DataLatency = 0x00;      
  p.FSMC_AccessMode = FSMC_AccessMode_B;
  // 指定的FSMC块
  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  // 地址和数据值不复用的数据总线 
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = 
                  FSMC_DataAddressMux_Disable; 
  // 外部存储器的类型
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = 
                            FSMC_MemoryType_NOR;
  // 数据宽度
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = 
                            FSMC_MemoryDataWidth_16b;
  // 禁用突发访问模式
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = 
                            FSMC_BurstAccessMode_Disable;
  // 指定等待信号的极性
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = 
                            FSMC_WaitSignalPolarity_Low;
  // disables the Wrapped burst access mode for Flash
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = 
                            FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = 
                            FSMC_WaitSignalActive_BeforeWaitState;
  // 启用指定的FSMC块的写操作
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation =
                            FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = 
                            FSMC_WaitSignal_Disable;
  // 扩展模式
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = 
                            FSMC_ExtendedMode_Disable;
  // 禁用写突发操作
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = 
                            FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;      
  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
  // 使能 FSMC Bank1_SRAM 
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}


//-----------------------------------------------------------------
// void LCD_WR_REG ( uint8_t RegIndex )
//-----------------------------------------------------------------
//
// 函数功能: LCD 写寄存器
// 入口参数: RegIndex  寄存器索引
// 返回参数: 无
// 全局变量: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WR_REG ( uint8_t RegIndex )
{
  LCD->LCD_REG = RegIndex;
}

//-----------------------------------------------------------------
//void LCD_WR_DATA(uint16_t RegValue)
//-----------------------------------------------------------------
//
// 函数功能: LCD 写数据值
// 入口参数: RegValue  数据寄存器值
// 返回参数: 无
// 全局变量: 无
// 注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WR_DATA(uint16_t RegValue)
{
  LCD->LCD_RAM = RegValue;
}


//-----------------------------------------------------------------
// void LCD_WriteRAM(uint16_t RGB_Code)
//-----------------------------------------------------------------
//
//  函数功能: LCD 写入现存数据
//  入口参数: 待输入的数据
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  LCD->LCD_RAM = RGB_Code;
}

//-----------------------------------------------------------------
// uint16_t LCD_ReadRAM(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 读取显存数据
//  入口参数: 无
//  返回参数: 显存数据
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
uint16_t LCD_ReadRAM(void)
{
  /* Read 16-bit Reg */
  return (LCD->LCD_RAM);
}

//-----------------------------------------------------------------
// void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
//-----------------------------------------------------------------
//
//  函数功能: LCD 对应寄存器写人寄存器值
//  入口参数: LCD_Reg 寄存器索引; LCD_RegValue 写入参数
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WriteReg(
  uint8_t   LCD_Reg,
  uint16_t  LCD_RegValue
)
{
  /* Write 16-bit Index, then Write Reg */
  LCD->LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
  LCD->LCD_RAM = LCD_RegValue;
}

//-----------------------------------------------------------------
// uint16_t LCD_ReadReg(uint8_t LCD_Reg)
//-----------------------------------------------------------------
//
//  函数功能: LCD 对应寄存器读取寄存器值
//  入口参数: LCD_Reg          寄存器索引
//  返回参数: 对应寄存器值
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD->LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (LCD->LCD_RAM);
}

//-----------------------------------------------------------------
// void LCD_WriteRAM_Prepare(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 准备写入数据  (即指定LCD RAM 寄存器)
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WriteRAM_Prepare(void)
{
  LCD->LCD_REG = LCD_Dev.REG_GRAM;
}

//-----------------------------------------------------------------
// void LCD_Display_Dir ( uint8_t Dir )
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置显示方向
//  入口参数: 显示方向
//            DISPLAY_DIR_L         // 左
//            DISPLAY_DIR_R         // 右
//            DISPLAY_DIR_U         // 上
//            DISPLAY_DIR_D         // 下
//  返回参数: 无
//  全局变量: LCD_Dev 显示信息结构体
//              包括:显示窗口大小
//                   显示寄存器索引
//                   设置坐标值索引
//                   屏幕扫描方向
//  注意事项: 不同驱动芯片，寄存器索引不一样
//
//-----------------------------------------------------------------
void LCD_Display_Dir ( uint8_t Dir )
{
  LCD_Dev.DISPLAY_DIR = Dir;
  if ( Dir == DISPLAY_DIR_L )
  {
    LCD_Dev.WIDTH   = 320;                                        // 显示窗口宽度
    LCD_Dev.HEIGHT  = 240;                                        // 显示窗口高度
    if ( LCD_Dev.REG_ID == ST7789V_ID )
    {
      LCD_Dev.REG_GRAM        = 0X2C;                             // 显示寄存器索引
      LCD_Dev.REG_XPOSITION   = 0x2A;                             // X轴坐标寄存器索引
      LCD_Dev.REG_YPOSITION   = 0x2B;                             // Y轴坐标寄存器索引
      LCD_Dev.SCAN_DIR        = ((0<<7)|(1<<6)|(1<<5));           // MY, MX, MV
    }
    else if ( LCD_Dev.REG_ID == SPFD5408_ID )
    {
      LCD_Dev.REG_GRAM        = 0X22;
      LCD_Dev.REG_XPOSITION   = 0x20;
      LCD_Dev.REG_YPOSITION   = 0x21;
      LCD_Dev.SCAN_DIR        = 0x1000 | ((0<<5)|(0<<4)|(1<<3));   // MY, MX, MV
    }
  }
  else if ( Dir == DISPLAY_DIR_R )
  {
    LCD_Dev.WIDTH             = 320;
    LCD_Dev.HEIGHT            = 240;
    if ( LCD_Dev.REG_ID == ST7789V_ID )
    {
      LCD_Dev.REG_GRAM        = 0X2C;
      LCD_Dev.REG_XPOSITION   = 0x2A;
      LCD_Dev.REG_YPOSITION   = 0x2B;
      LCD_Dev.SCAN_DIR        = ((1<<7)|(0<<6)|(1<<5));             // MY, MX, MV
    }
    else if ( LCD_Dev.REG_ID == SPFD5408_ID )
    {
      LCD_Dev.REG_GRAM        = 0X22;
      LCD_Dev.REG_XPOSITION   = 0x20;
      LCD_Dev.REG_YPOSITION   = 0x21;
      LCD_Dev.SCAN_DIR        = 0x1000 | ((1<<5)|(1<<4)|(1<<3));    // MY, MX, MV
    }
  }
  else if ( Dir == DISPLAY_DIR_U )
  {
    LCD_Dev.WIDTH             = 240;
    LCD_Dev.HEIGHT            = 320;
    if ( LCD_Dev.REG_ID == ST7789V_ID )
    {
      LCD_Dev.REG_GRAM        = 0X2C;
      LCD_Dev.REG_XPOSITION   = 0x2A;
      LCD_Dev.REG_YPOSITION   = 0x2B;
      LCD_Dev.SCAN_DIR        = ((1<<7)|(1<<6)|(0<<5));             // MY, MX, MV
    }
    else if ( LCD_Dev.REG_ID == SPFD5408_ID )
    {
      LCD_Dev.REG_GRAM        = 0X22;
      LCD_Dev.REG_XPOSITION   = 0x20;
      LCD_Dev.REG_YPOSITION   = 0x21;
      LCD_Dev.SCAN_DIR        = 0x1000 | ((1<<5)|(0<<4)|(0<<3));    // MY, MX, MV 
    }
  }
  else                                                              //DISPLAY_DIR_D
  {
    LCD_Dev.WIDTH             = 240;
    LCD_Dev.HEIGHT            = 320;
    if ( LCD_Dev.REG_ID == ST7789V_ID )
    {
      LCD_Dev.REG_GRAM        = 0X2C;
      LCD_Dev.REG_XPOSITION   = 0x2A;
      LCD_Dev.REG_YPOSITION   = 0x2B;
      LCD_Dev.SCAN_DIR        = ((0<<7)|(0<<6)|(0<<5));             // MY, MX, MV
    }
    else if ( LCD_Dev.REG_ID == SPFD5408_ID )
    {
      LCD_Dev.REG_GRAM        = 0X22;
      LCD_Dev.REG_XPOSITION   = 0x20;
      LCD_Dev.REG_YPOSITION   = 0x21;
      LCD_Dev.SCAN_DIR        = 0x1000 | ((0<<5)|(1<<4)|(0<<3));    // MY, MX, MV
    }
  }

}

//-----------------------------------------------------------------
// void LCD_Init(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 初始化
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_Init(void)
{
// 如果用CubeMx 配置  LCD_CtrlLinesConfig() 和 LCD_FSMCConfig() 可以不用添加
  LCD_CtrlLinesConfig();                                            // LCD 端口配置
  LCD_FSMCConfig();                                                 // FSMC 控制器配置

  LCD_Delay ( 10 );                                                 // delay 10 ms
  LCD_RST_Clr;                                                      // 下降沿复位
  LCD_Delay ( 10 );                                                 // delay 10 ms
  LCD_RST_Set;         // 退出复位
  LCD_Delay ( 10 );                                                 // delay 10 ms

  /* Read the LCD ID */
  //尝试 ST7789V  ID的读取
  LCD_ReadReg ( 0x04 );
  LCD_Dev.REG_ID =   LCD_ReadRAM();                                 // 无效数据
  LCD_Dev.REG_ID =   LCD_ReadRAM();                                 // 读到0X85
  LCD_Dev.REG_ID <<= 8;
  LCD_Dev.REG_ID |=  LCD_ReadRAM();                                 // 读取52

  if(LCD_Dev.REG_ID != ST7789V_ID)                                  // 驱动型号非 ST7789V
  {
    LCD_Dev.REG_ID = LCD_ReadReg ( 0x00 );                          // 尝试获取 5408 的 ID
  }

  if ( LCD_Dev.REG_ID == ST7789V_ID )                               // ST7789V
  {
    //-------------------------------------------------------------------------------------//
    LCD_WR_REG ( 0x11 );
    LCD_Delay  ( 120  );                                            // Delay 1200 ms
    //------------------------------display and color format setting-----------------------//
    LCD_WR_REG ( 0x36 );  LCD_WR_DATA ( 0x00 );
    LCD_WR_REG ( 0x3a );  LCD_WR_DATA ( 0x05 );
    //--------------------------------ST7789V Frame rate setting---------------------------//
    LCD_WR_REG ( 0xb2 );  LCD_WR_DATA ( 0x0c );  LCD_WR_DATA ( 0x0c );
                          LCD_WR_DATA ( 0x00 );  LCD_WR_DATA ( 0x33 );
                          LCD_WR_DATA ( 0x33 );
    LCD_WR_REG ( 0xb7 );  LCD_WR_DATA ( 0x35 );
    //---------------------------------ST7789V Power setting-------------------------------//
    LCD_WR_REG ( 0xbb );  LCD_WR_DATA ( 0x28 );
    LCD_WR_REG ( 0xc0 );  LCD_WR_DATA ( 0x2c );
    LCD_WR_REG ( 0xc2 );  LCD_WR_DATA ( 0x01 );
    LCD_WR_REG ( 0xc3 );  LCD_WR_DATA ( 0x0b );
    LCD_WR_REG ( 0xc4 );  LCD_WR_DATA ( 0x20 );
    LCD_WR_REG ( 0xc6 );  LCD_WR_DATA ( 0x0f );                     // Frame Rate Control in Normal Mode
    LCD_WR_REG ( 0xd0 );  LCD_WR_DATA ( 0xa4 );  LCD_WR_DATA ( 0xa1 );
    //--------------------------------ST7789V gamma setting--------------------------------//
    LCD_WR_REG ( 0xe0 );  LCD_WR_DATA ( 0xd0 );  LCD_WR_DATA ( 0x01 );
                          LCD_WR_DATA ( 0x08 );  LCD_WR_DATA ( 0x0f );
                          LCD_WR_DATA ( 0x11 );  LCD_WR_DATA ( 0x2a );
                          LCD_WR_DATA ( 0x36 );  LCD_WR_DATA ( 0x55 );
                          LCD_WR_DATA ( 0x44 );  LCD_WR_DATA ( 0x3a );
                          LCD_WR_DATA ( 0x0b );  LCD_WR_DATA ( 0x06 );
                          LCD_WR_DATA ( 0x11 );  LCD_WR_DATA ( 0x20 );
                          
    LCD_WR_REG ( 0xe1 );  LCD_WR_DATA ( 0xd0 );  LCD_WR_DATA ( 0x02 );
                          LCD_WR_DATA ( 0x07 );  LCD_WR_DATA ( 0x0a );
                          LCD_WR_DATA ( 0x0b );  LCD_WR_DATA ( 0x18 );
                          LCD_WR_DATA ( 0x34 );  LCD_WR_DATA ( 0x43 );
                          LCD_WR_DATA ( 0x4a );  LCD_WR_DATA ( 0x2b );
                          LCD_WR_DATA ( 0x1b );  LCD_WR_DATA ( 0x1c );
                          LCD_WR_DATA ( 0x22 );  LCD_WR_DATA ( 0x1f );
                          
    LCD_WR_REG ( 0x29 );
    LCD_WR_REG ( 0x2c );

    LCD_Display_Dir ( LCD_DIR );                                    // 设置显示方向

    LCD_WR_REG ( 0X36 );  LCD_WR_DATA ( LCD_Dev.SCAN_DIR );         // 设置扫描方向

    LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);     // 设置显示窗口
  }
  else if ( LCD_Dev.REG_ID == SPFD5408_ID )                         // SPFD5408
  {
    /* Start Initial Sequence ------------------------------------------------*/
    LCD_WriteReg ( LCD_REG_1,   0x0100 );  /* Set SS bit */
    LCD_WriteReg ( LCD_REG_2,   0x0700 );  /* Set 1 line inversion */
    LCD_WriteReg ( LCD_REG_3,   0x1030 );  /* Set GRAM write direction and BGR=1. */
    LCD_WriteReg ( LCD_REG_4,   0x0000 );  /* Resize register */

    LCD_WriteReg ( LCD_REG_8,   0x0202 );  /* Set the back porch and front porch */
    LCD_WriteReg ( LCD_REG_9,   0x0000 );  /* Set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg ( LCD_REG_10,  0x0000 );  /* FMARK function */
    LCD_WriteReg ( LCD_REG_12,  0x0000 );  /* RGB 18-bit System interface setting */
    LCD_WriteReg ( LCD_REG_13,  0x0000 );  /* Frame marker Position */
    LCD_WriteReg ( LCD_REG_15,  0x0000 );  /* RGB interface polarity, no impact */

    /* Power On sequence -----------------------------------------------------*/
    LCD_WriteReg ( LCD_REG_16,  0x0000 );  /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg ( LCD_REG_17,  0x0000 );  /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg ( LCD_REG_18,  0x0000 );  /* VREG1OUT voltage */
    LCD_WriteReg ( LCD_REG_19,  0x0000 );  /* VDV[4:0] for VCOM amplitude */
    LCD_Delay ( 200 );                     /* Dis-charge capacitor power voltage (200ms) */

    LCD_WriteReg ( LCD_REG_17,  0x0007 );  /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_Delay ( 50 );                      /* Delay 50 ms */
    LCD_WriteReg ( LCD_REG_16,  0x12B0 );  /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_Delay ( 50 );                      /* Delay 50 ms */
    LCD_WriteReg ( LCD_REG_18,  0x01BD );  /* External reference voltage= Vci */
    LCD_Delay ( 50 );
    LCD_WriteReg ( LCD_REG_19,  0x1400 );  /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg ( LCD_REG_41,  0x000E );  /* VCM[4:0] for VCOMH */
    LCD_Delay ( 50 );                      /* Delay 50 ms */
    LCD_WriteReg ( LCD_REG_32,  0x0000 );  /* GRAM horizontal Address */
    LCD_WriteReg ( LCD_REG_33,  0x013F );  /* GRAM Vertical Address */

    /* Adjust the Gamma Curve (SPFD5408B)-------------------------------------*/
    LCD_WriteReg ( LCD_REG_48,  0x0b0d );
    LCD_WriteReg ( LCD_REG_49,  0x1923 );
    LCD_WriteReg ( LCD_REG_50,  0x1c26 );
    LCD_WriteReg ( LCD_REG_51,  0x261c );
    LCD_WriteReg ( LCD_REG_52,  0x2419 );
    LCD_WriteReg ( LCD_REG_53,  0x0d0b );
    LCD_WriteReg ( LCD_REG_54,  0x1006 );
    LCD_WriteReg ( LCD_REG_55,  0x0610 );
    LCD_WriteReg ( LCD_REG_56,  0x0706 );
    LCD_WriteReg ( LCD_REG_57,  0x0304 );
    LCD_WriteReg ( LCD_REG_58,  0x0e05 );
    LCD_WriteReg ( LCD_REG_59,  0x0e01 );
    LCD_WriteReg ( LCD_REG_60,  0x010e );
    LCD_WriteReg ( LCD_REG_61,  0x050e );
    LCD_WriteReg ( LCD_REG_62,  0x0403 );
    LCD_WriteReg ( LCD_REG_63,  0x0607 );

    /* Set GRAM area ---------------------------------------------------------*/
    LCD_WriteReg ( LCD_REG_80,  0x0000 );  /* Horizontal GRAM Start Address */
    LCD_WriteReg ( LCD_REG_81,  0x00EF );  /* Horizontal GRAM End Address */
    LCD_WriteReg ( LCD_REG_82,  0x0000 );  /* Vertical GRAM Start Address */
    LCD_WriteReg ( LCD_REG_83,  0x013F );  /* Vertical GRAM End Address */

    LCD_WriteReg ( LCD_REG_96,  0xA700 );  /* Gate Scan Line */
    LCD_WriteReg ( LCD_REG_97,  0x0001 );  /* NDL, VLE, REV */
    LCD_WriteReg ( LCD_REG_106, 0x0000 );  /* set scrolling line */

    /* Partial Display Control -----------------------------------------------*/
    LCD_WriteReg ( LCD_REG_128, 0x0000 );
    LCD_WriteReg ( LCD_REG_129, 0x0000 );
    LCD_WriteReg ( LCD_REG_130, 0x0000 );
    LCD_WriteReg ( LCD_REG_131, 0x0000 );
    LCD_WriteReg ( LCD_REG_132, 0x0000 );
    LCD_WriteReg ( LCD_REG_133, 0x0000 );

    /* Panel Control ---------------------------------------------------------*/
    LCD_WriteReg ( LCD_REG_144, 0x0010 );
    LCD_WriteReg ( LCD_REG_146, 0x0000 );
    LCD_WriteReg ( LCD_REG_147, 0x0003 );
    LCD_WriteReg ( LCD_REG_149, 0x0110 );
    LCD_WriteReg ( LCD_REG_151, 0x0000 );
    LCD_WriteReg ( LCD_REG_152, 0x0000 );

    LCD_Display_Dir ( LCD_DIR );                                    // 设置显示方向

    /* Set GRAM write direction and BGR=1
       I/D=01 (Horizontal : increment, Vertical : decrement)
       AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg ( LCD_REG_3, LCD_Dev.SCAN_DIR );

    LCD_WriteReg ( LCD_REG_7,  0x0112 );  /* 262K color and display ON */

    LCD_SetDisplayWindow ( 0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH );   // 设置显示窗口
  }

  LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN);             // 设置字体
  LCD_LED_ON;                                                       // 开启背光
}

//-----------------------------------------------------------------
// void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置字体颜色及背景颜色
//  入口参数: _TextColor 字体颜色; _BackColor 背景颜色
//  返回参数: 无
//  全局变量: TextColor; BackColor
//  注意事项: RGB 565 格式
//
//-----------------------------------------------------------------
void LCD_SetColors(
  __IO uint16_t _TextColor,
  __IO uint16_t _BackColor
)
{
  TextColor = _TextColor;
  BackColor = _BackColor;
}

//-----------------------------------------------------------------
// void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
//-----------------------------------------------------------------
//
//  函数功能: LCD 获取字体颜色及背景颜色
//  入口参数: _TextColor 字体颜色指针; _BackColor 背景颜色指针
//  返回参数: 无
//  全局变量: TextColor; BackColor
//  注意事项: RGB 565 格式
//
//-----------------------------------------------------------------
void LCD_GetColors(
  __IO uint16_t *_TextColor,
  __IO uint16_t *_BackColor
)
{
  *_TextColor = TextColor; *_BackColor = BackColor;
}

//-----------------------------------------------------------------
// void LCD_SetTextColor(__IO uint16_t Color)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置字体颜色
//  入口参数: Color 字体颜色
//  返回参数: 无
//  全局变量: TextColor
//  注意事项: RGB 565 格式
//
//-----------------------------------------------------------------
void LCD_SetTextColor(__IO uint16_t Color)
{
  TextColor = Color;
}

//-----------------------------------------------------------------
// void LCD_SetBackColor(__IO uint16_t Color)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置背景颜色
//  入口参数: Color 背景颜色
//  返回参数: 无
//  全局变量: BackColor
//  注意事项: RGB 565 格式
//
//-----------------------------------------------------------------
void LCD_SetBackColor(__IO uint16_t Color)
{
  BackColor = Color;
}

//-----------------------------------------------------------------
// void LCD_SetFont(sFONT *fonts)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置字体  
//  入口参数: *fonts 字符字体信息指针  *fonts_cn 中文字体信息指针
//  返回参数: 无 
//  全局变量: LCD_Currentfonts
//  注意事项: 字体在 “fonts.h”中定义
//
//-----------------------------------------------------------------
void LCD_SetFont(sFONT *fonts, sFONT_CN *fonts_cn)
{
  LCD_Currentfonts    = fonts;
  LCD_Currentfonts_CN = fonts_cn;
}

//-----------------------------------------------------------------
// void LCD_Clear(uint16_t Color)
//-----------------------------------------------------------------
//
//  函数功能: LCD 清屏
//  入口参数: Color 清屏颜色
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_Clear(uint16_t Color)
{
  uint32_t index = 0;
  
  LCD_SetDisplayWindow ( 0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH );     // 设置窗口大小

  LCD_WriteRAM_Prepare ( );                                         /* Prepare to write GRAM */
  for(index = 0; index < LCD_Dev.WIDTH * LCD_Dev.HEIGHT; index++)   //
  {
    LCD->LCD_RAM = Color;                                           //
  }
}

//-----------------------------------------------------------------
// void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置显示坐标
//  入口参数: Color 清屏颜色
//  返回参数: 无
//  全局变量: 无
//  注意事项: 
//      ST7789V_ID  每次写入8bit
//      SPFD5408_ID 每次写入16bit
//
//-----------------------------------------------------------------
static void LCD_SetCursor(
  uint16_t Xpos,
  uint16_t Ypos
)
{
  if(LCD_Dev.REG_ID == ST7789V_ID)                                  // 寄存器每次写入 8bit, 16bit 数据需分两次写入
  {
    LCD_WR_REG ( LCD_Dev.REG_XPOSITION );
    LCD_WR_DATA ( Xpos >> 8 );       LCD_WR_DATA ( Xpos & 0XFF );
    LCD_WR_REG ( LCD_Dev.REG_YPOSITION );
    LCD_WR_DATA ( Ypos >> 8 );       LCD_WR_DATA ( Ypos & 0XFF );
  }
  if(LCD_Dev.REG_ID == SPFD5408_ID)                                 // 寄存器每次写入 16bit
  {
    LCD_WriteReg ( LCD_Dev.REG_XPOSITION, Xpos );
    LCD_WriteReg ( LCD_Dev.REG_YPOSITION, Ypos );
  }
} 

//-----------------------------------------------------------------
// void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
//-----------------------------------------------------------------
//
//  函数功能: LCD 对应坐标写入一个 ASCII字符 字符数据通过取模获取
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; *c 字模首地址
//  返回参数: 无
//  全局变量: 无
//  注意事项: 显示坐标为字符显示的左上角坐标, 左上角为坐标原点
//
//-----------------------------------------------------------------
static void LCD_DrawChar(
  uint16_t Xpos, 
  uint16_t Ypos, 
  const uint8_t *CharFont
)
{
  uint32_t  index = 0, i = 0;
  uint8_t   number_byte = ((LCD_Currentfonts->Width + 7) >> 3);

  LCD_SetDisplayWindow (Xpos, Ypos, LCD_Currentfonts->Height, LCD_Currentfonts->Width);
  LCD_WriteRAM_Prepare();                                               // 设置显存索引
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  {
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
      if( (CharFont[(index * number_byte) + (i >> 3)] & (0x80 >> (i % 8))) == 0x00 ) 
      {
          LCD_WriteRAM(BackColor);                                     // 写入背景色
      }
      else
      {
          LCD_WriteRAM(TextColor);                                     // 写入字体色
      }
    }
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
//-----------------------------------------------------------------
//
//  函数功能: LCD 显示字符
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Ascii 显示字符
//  返回参数: 无
//  全局变量: 无
//  注意事项: 显示坐标为字符显示的左上角坐标, 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DisplayChar(
  uint16_t  Xpos, 
  uint16_t  Ypos, 
  uint8_t   Ascii
)
{
  const uint8_t *FontAddr;

  // 获取字符字模地址
  FontAddr = &LCD_Currentfonts->table[(Ascii-' ') *  
             LCD_Currentfonts->Height * 
            ((LCD_Currentfonts->Width + 7) / 8)];
  LCD_DrawChar( Xpos, Ypos, FontAddr );                             // 显示字模数据
}

//-----------------------------------------------------------------
// void GetChineseCode( const uint8_t *pFontAddr, uint8_t *ChineseChar )
//-----------------------------------------------------------------
//
// 函数功能: 获取字模地址
// 入口参数: *pFontAddr: 汉字字模指针. *ChineseChar: 要查找字模的汉字编码指针.
// 返回参数: 无
// 全局变量: LCD_Currentfonts_CN 字体信息
// 调用模块: 无
// 注意事项: 输入一个中文，取得其字模地址,
// 
//-----------------------------------------------------------------
const uint8_t *GetChineseCode(
  uint8_t *ChineseChar                                              // 要查找的汉字
)
{
  uint16_t i;
  const uint8_t *pFontAddr;                                         // 汉字字模地址

  uint16_t GBXX_LENGTH = LCD_Currentfonts_CN->Length;               // 获取字模数据个数

  // 查找对应汉字的字模地址
  for (i = 0; i < GBXX_LENGTH; i ++)
  {
    // 对比输入的汉字和字模数组中的汉字是否一致
    if ((LCD_Currentfonts_CN->table[i].Index[0] == ChineseChar[0]) && 
        (LCD_Currentfonts_CN->table[i].Index[1] == ChineseChar[1]))
    {                                                               
      pFontAddr = &LCD_Currentfonts_CN->table[i].Msk[0];            // 获取字模地址
      return pFontAddr;                                             // 返回字模地址
    }
  }
  return 0;                                                         // 没有对应汉字字模, 返回0
}

//-----------------------------------------------------------------
// void LCD_DrawChineseChar(uint16_t Xpos, uint16_t Ypos, uint8_t *cn)
//-----------------------------------------------------------------
//
// 函数功能: LCD 对应坐标写入一个中文字符
// 入口参数: Xpos X轴坐标; Ypos Y轴坐标; *ChineseFont 字模首地址
// 返回参数: 无
// 全局变量: LCD_Currentfonts_CN 中文字符信息
// 调用模块: 无
// 注意事项: 
// 
//-----------------------------------------------------------------
static void LCD_DrawChineseChar(
  uint16_t Xpos,
  uint16_t Ypos,
  const uint8_t *ChineseFont
)
{
  uint16_t  index = 0, i = 0;
  uint8_t   number_byte = ((LCD_Currentfonts_CN->Width + 7) >> 3);  // 计算字模每行需要的字节数

  // 设置显示窗口
  LCD_SetDisplayWindow (Xpos, Ypos, LCD_Currentfonts_CN->Height, LCD_Currentfonts_CN->Width); 

  LCD_WriteRAM_Prepare();                                          /* Prepare to write GRAM */
  for(index = 0; index < LCD_Currentfonts_CN->Height; index++)
  {
    for(i = 0; i < LCD_Currentfonts_CN->Width; i++)
    {
      if (ChineseFont != 0)                                         // 有字模时, 正常写入
      {
        if ( (ChineseFont[(index * number_byte) + (i >> 3)] & (0x80 >> (i % 8))) == 0x00 )
        {
          LCD_WriteRAM(BackColor);                                  // 写入背景色
        }
        else
        {
          LCD_WriteRAM(TextColor);                                  // 写入字体色
        }
      }
      else                                                          // 没有字模时写入背景色
      {
        LCD_WriteRAM(BackColor);                                    // 写入背景色
      }
    }
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DisplayChineseChar(uint16_t Xpos, uint16_t Ypos, uint8_t ChineseCode[2])
//-----------------------------------------------------------------
//
//  函数功能: LCD 显示中文字符
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; ChineseCode[2] 显示中文编码
//  返回参数: 无
//  全局变量: 无
//  注意事项: 显示坐标为字符显示的左上角坐标, 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DisplayChineseChar(
  uint16_t  Xpos, 
  uint16_t  Ypos, 
  uint8_t   ChineseCode[2]
)
{
  const uint8_t *FontAddr = 0;

  // 获取中文字模地址
  FontAddr = GetChineseCode( ChineseCode );
  LCD_DrawChineseChar( Xpos, Ypos, FontAddr);                       // 显示字模数据
}

//-----------------------------------------------------------------
// void LCD_DisplayString(uint16_t Xpos, uint16_t  Ypos, uint8_t *ptr)
//-----------------------------------------------------------------
// 
//  函数功能: LCD 显示字符串-中英文混合
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; *ptr 显示字符串指针
//  返回参数: 无
//  全局变量: 无
//  注意事项: 显示坐标为字符显示的左上角坐标, 左上角为坐标原点
// 
//-----------------------------------------------------------------
void LCD_DisplayString(
  uint16_t Xpos,
  uint16_t Ypos,
  uint8_t *ptr
)
{
  uint16_t xAddr = Xpos;
  uint16_t yAddr = Ypos;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) )
  {
    if ( xAddr <= (LCD_Dev.WIDTH - LCD_Currentfonts->Width) )
    {
      if (*ptr < 128)                                                 // 字符
      {
        LCD_DisplayChar(xAddr, yAddr, *ptr);                          // 显示英文字符
        xAddr += LCD_Currentfonts->Width;
        ptr++;                                                        // 英文字符占用一个字节
      }
      else                                                            // 中文
      {
        LCD_DisplayChineseChar(xAddr, yAddr, ptr);                    // 显示中文字符
        xAddr += LCD_Currentfonts_CN->Width;
        ptr+= 2;                                                      // 中文字符占用两个字节
      }
    }
    else 
    {
      xAddr = 0;
      yAddr = yAddr + LCD_Currentfonts->Height;
    }
  }
}

//-----------------------------------------------------------------
// void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
//-----------------------------------------------------------------
//
//  函数功能: LCD 设置显示窗口
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度; Width 宽度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 显示坐标为字符显示的左上角坐标, 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_SetDisplayWindow(
  uint16_t Xpos,
  uint16_t Ypos,
  uint16_t Height,
  uint16_t Width
)
{
   uint16_t X_Start = 0, X_End = 0; 
   uint16_t Y_Start = 0, Y_End = 0; 
   
   uint16_t Xpos_Start = 0;
   uint16_t Ypos_Start = 0;
   
  if(LCD_Dev.REG_ID == ST7789V_ID)
  {
    X_Start     = Xpos;                               // Horizontal GRAM Start Address
    X_End       = Xpos + Width - 1;                   // Horizontal GRAM End Address
    Y_Start     = Ypos;                               // Vertical GRAM Start Address
    Y_End       = Ypos + Height - 1;                  // Vertical GRAM End Address 

    Xpos_Start  = X_Start;
    Ypos_Start  = Y_Start;

    /* Horizontal GRAM Start Address */
    LCD_WR_REG(LCD_Dev.REG_XPOSITION);
    LCD_WR_DATA ( X_Start >> 8 );                 LCD_WR_DATA ( X_Start & 0xFF );

    /* Horizontal GRAM End Address */
    LCD_WR_DATA ( X_End >> 8 );  LCD_WR_DATA ( X_End & 0XFF );

    /* Vertical GRAM Start Address */
    LCD_WR_REG(LCD_Dev.REG_YPOSITION);
    LCD_WR_DATA ( Y_Start >> 8 );                 LCD_WR_DATA ( Y_Start & 0xFF );

    /* Vertical GRAM End Address */
    LCD_WR_DATA ( Y_End >> 8); LCD_WR_DATA ( Y_End & 0XFF );

  }
  else if(LCD_Dev.REG_ID == SPFD5408_ID)
  {
    if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_L )
    {
      X_Start     = LCD_Dev.HEIGHT - Ypos - Height;   // Horizontal GRAM Start Address
      X_End       = LCD_Dev.HEIGHT - Ypos - 1;        // Horizontal GRAM End Address
      Y_Start     = LCD_Dev.WIDTH  - Xpos - Width;    // Vertical GRAM Start Address
      Y_End       = LCD_Dev.WIDTH  - Xpos - 1;        // Vertical GRAM End Address 

      Xpos_Start  = X_End;
      Ypos_Start  = Y_End;
    }
    else if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_R ) 
    {
      X_Start     = Ypos;                             // Horizontal GRAM Start Address
      X_End       = Ypos + Height - 1;                // Horizontal GRAM End Address
      Y_Start     = Xpos;                             // Vertical GRAM Start Address
      Y_End       = Xpos + Width - 1;                 // Vertical GRAM End Address 

      Xpos_Start  = X_Start;
      Ypos_Start  = Y_Start;
    }
    else if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_U )
    {
      X_Start     = LCD_Dev.WIDTH - Xpos - Width;     // Horizontal GRAM Start Address
      X_End       = LCD_Dev.WIDTH - Xpos - 1;         // Horizontal GRAM End Address
      Y_Start     = Ypos;                             // Vertical GRAM Start Address
      Y_End       = Ypos + Height;                    // Vertical GRAM End Address 

      Xpos_Start  = X_End;
      Ypos_Start  = Y_Start;
    }
    else 
    {
      X_Start     = Xpos;                             // Horizontal GRAM Start Address
      X_End       = Xpos + Width - 1;                 // Horizontal GRAM End Address
      Y_Start     = LCD_Dev.HEIGHT - Ypos - Height;   // Vertical GRAM Start Address
      Y_End       = LCD_Dev.HEIGHT - Ypos - 1;        // Vertical GRAM End Address 

      Xpos_Start  = X_Start;
      Ypos_Start  = Y_End;
    }
    LCD_WriteReg ( LCD_REG_80, X_Start ); 
    LCD_WriteReg ( LCD_REG_81, X_End );
    LCD_WriteReg ( LCD_REG_82, Y_Start );
    LCD_WriteReg ( LCD_REG_83, Y_End );
  }
  LCD_SetCursor(Xpos_Start, Ypos_Start);
}

//-----------------------------------------------------------------
// void LCD_WindowModeDisable(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 关闭窗口显示模式
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_WindowModeDisable(void)
{
  LCD_SetDisplayWindow(239, 0x13F, 240, 320);
//  LCD_WriteReg(LCD_REG_3, 0x1018);
  LCD_WriteReg(LCD_REG_3, 0x1018);
}

//-----------------------------------------------------------------
// void LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制水平直线
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Length 长度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawHLine(
  uint16_t Xpos,
  uint16_t Ypos,
  uint16_t Length
)
{
  uint32_t i = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, 1, Length );
  LCD_WriteRAM_Prepare();                       /* Prepare to write GRAM */
  for(i = 0; i < Length; i++)
  {
    LCD_WriteRAM(TextColor);
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制水平虚直线
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Length 长度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawDotHLine(
  uint16_t Xpos,
  uint16_t Ypos,
  uint16_t Length
)
{
  uint32_t i = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, 1, Length );
  LCD_WriteRAM_Prepare();                       /* Prepare to write GRAM */
  for(i = 0; i < Length; i++)
  {
    if ((i % DOT) == (DOT -1))
    {
      LCD_WriteRAM(BackColor);
    }
    else 
    {
      LCD_WriteRAM(TextColor);
    }
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Height)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制垂直直线
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawVLine(
  uint16_t Xpos,
  uint16_t Ypos,
  uint16_t Height
)
{
  uint32_t i = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, Height, 1 );
  LCD_WriteRAM_Prepare();                     /* Prepare to write GRAM */
  for(i = 0; i < Height; i++)
  {
    LCD_WriteRAM(TextColor);
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Height)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制垂直虚直线
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawDotVLine(
  uint16_t Xpos,
  uint16_t Ypos,
  uint16_t Height
)
{
  uint32_t i = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, Height, 1 );
  LCD_WriteRAM_Prepare();                     /* Prepare to write GRAM */
  for(i = 0; i < Height; i++)
  {
    if ((i % DOT) == (DOT -1))
    {
      LCD_WriteRAM(BackColor);
    }
    else 
    {
      LCD_WriteRAM(TextColor);
    }
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制矩形
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度; Width 宽度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点, 显示坐标为左上角
//
//-----------------------------------------------------------------
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
{
  LCD_DrawHLine(Xpos, Ypos, Width);
  LCD_DrawHLine(Xpos, (Ypos + Height -1), Width);

  LCD_DrawVLine(Xpos, Ypos, Height);
  LCD_DrawVLine((Xpos + Width - 1), Ypos, Height);
}

//-----------------------------------------------------------------
// void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
//-----------------------------------------------------------------
//
//  函数功能: LCD 显示单色图片   数据可以通过取模方式获取
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度; Width 宽度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawMonoPict(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width, const uint8_t *Pict)
{
  uint32_t index = 0, i = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, Height, Width );
  LCD_WriteRAM_Prepare();                     /* Prepare to write GRAM */

  for(index = 0; index < (Height * Width) >> 3; index++)
  {
    for (i = 0; i  < 8; i ++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
//-----------------------------------------------------------------
//
//  函数功能: LCD 显示RGB图片 
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度; Width 宽度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawRgbPict(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width, const uint8_t *Pict)
{
  uint32_t index = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, Height, Width );
  LCD_WriteRAM_Prepare();                     /* Prepare to write GRAM */

  for(index = 0; index < (Height * Width); index++)
  {
    LCD_WriteRAM (( Pict[ index << 1 ])+ (Pict[(index<<1) + 1]  << 8));
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
//-----------------------------------------------------------------
//
//  函数功能: LCD 绘制填充矩形 填充颜色为背景颜色
//  入口参数: Xpos X轴坐标; Ypos Y轴坐标; Height 高度; Width 宽度
//  返回参数: 无
//  全局变量: 无
//  注意事项: 左上角为坐标原点
//
//-----------------------------------------------------------------
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
{
  uint32_t index = 0;

  LCD_SetDisplayWindow (Xpos, Ypos, Height, Width );
  LCD_WriteRAM_Prepare();                     /* Prepare to write GRAM */

  for(index = 0; index < (Height * Width); index++)
  {
    LCD_WriteRAM ( BackColor );
  }
  LCD_SetDisplayWindow (0, 0, LCD_Dev.HEIGHT, LCD_Dev.WIDTH);         // 设置显示窗口
}

//-----------------------------------------------------------------
// void LCD_PowerOn(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 打开电源
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_PowerOn(void)
{
   if ( LCD_Dev.REG_ID == ST7789V_ID )
   {

   }
   else
   {
     /* Power On sequence ---------------------------------------------------------*/
     LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
     LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
     LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
     LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude*/
     LCD_Delay(200);                   /* Dis-charge capacitor power voltage (200ms) */
     LCD_WriteReg(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
     LCD_WriteReg(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
     LCD_Delay(50);                    /* Delay 50 ms */
     LCD_WriteReg(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
     LCD_Delay(50);                    /* Delay 50 ms */
     LCD_WriteReg(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
     LCD_WriteReg(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
     LCD_Delay(50);                    /* Delay 50 ms */
     LCD_WriteReg(LCD_REG_7, 0x0173);  /* 262K color and display ON */
   }
}

//-----------------------------------------------------------------
// void LCD_DisplayOn(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 关闭显示
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_DisplayOn(void)
{
  if ( LCD_Dev.REG_ID == ST7789V_ID )
  {
    LCD_WR_REG(0x29);
  }
  else
  {
    /* Display On */
    LCD_WriteReg(LCD_REG_7, 0x0173); /* 262K color and display ON */
  }
}

//-----------------------------------------------------------------
// void LCD_DisplayOff(void)
//-----------------------------------------------------------------
//
//  函数功能: LCD 开启显示
//  入口参数: 无
//  返回参数: 无
//  全局变量: 无
//  注意事项: 无
//
//-----------------------------------------------------------------
void LCD_DisplayOff(void)
{
  /* Display Off */
    if ( LCD_Dev.REG_ID == ST7789V_ID )
    {
        LCD_WR_REG(0x28);
    }
    else
    {
        LCD_WriteReg(LCD_REG_7, 0x0);
    }
}

// 绘制一个像素点
/**
  * @brief  Displays a pixel.
  * @param  x: pixel x.
  * @param  y: pixel y.
  * @retval None
  */
void PutPixel(int16_t x, int16_t y)
{
  if(x < 0 || x > (LCD_Dev.WIDTH - 1) || y < 0 || y > (LCD_Dev.HEIGHT - 1))
  {
    return;
  }
  LCD_DrawHLine(x, y, 1);
}


//-----------------------------------------------------------------
//void LCD_SetPoint(
//  uint16_t Xpos,                                                    // X轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t PointColor                                               // 像素点颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 用指定的颜色在指定的坐标位置上画出一个点
// 入口参数: 
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           PointColor:     像素点颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块: LCD_ColorBox().
// 注意事项:
//-----------------------------------------------------------------
void LCD_SetPoint(
  uint16_t Xpos,                                                    // X轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t PointColor                                               // 像素点颜色
)
{
  uint16_t x = Xpos;
  uint16_t y = Ypos;
  
  if ( LCD_Dev.REG_ID == SPFD5408_ID )                              // SPFD5408 需要做坐标变换
  {
    if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_L )
    {
      x  = LCD_Dev.HEIGHT - Ypos - 1;
      y  = LCD_Dev.WIDTH  - Xpos - 1;
    }
    else if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_R ) 
    {
      x  = Ypos;
      y  = Xpos;
    }
    else if ( LCD_Dev.DISPLAY_DIR == DISPLAY_DIR_U )
    {
      x  = LCD_Dev.WIDTH - Xpos - 1;
      y  = Ypos;
    }
    else 
    {
      x  = Xpos;
      y  = LCD_Dev.HEIGHT - Ypos - 1;
    }
  }
  LCD_SetCursor( x, y );
  LCD_WriteRAM_Prepare();                       /* Prepare to write GRAM */
  LCD_WriteRAM( PointColor );
}


//-----------------------------------------------------------------
//void LCD_WriteString(
//  uint16_t Xpos,                                                    // X轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t CharColor,                                               // 字体颜色
//  uint16_t CharBackColor,                                           // 背景颜色
//  uint8_t  *sp                                                      // 指向字符串的字符指针
//)
//-----------------------------------------------------------------
//
// 函数功能: LCD写字符串函数, 可以英文字符和汉字字符混合写.
// 入口参数:
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           CharColor:      字符颜色.
//           CharBackColor:  背景颜色.
//           *sp  :          显示的字符串指针.
// 返回参数: 无
// 全局变量: 无
// 调用模块: LCD_WriteASCII(),  LCD_WriteChinese().
// 注意事项:
//-----------------------------------------------------------------
void LCD_WriteString(
  uint16_t Xpos,                                                    // X轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  *sp                                                      // 指向字符串的字符指针
)
{
  LCD_SetColors(CharColor, CharBackColor);                          // 设置字体颜色和背景颜色
  LCD_DisplayString(Xpos, Ypos, sp);                                 // 显示字符串
}

//-----------------------------------------------------------------
//void LCD_WriteChinese24x24string(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t CharColor,                                               // 字体颜色
//  uint16_t CharBackColor,                                           // 背景颜色
//  uint8_t  *sp                                                      // 中文字符指针
//)
//-----------------------------------------------------------------
//
// 函数功能: LCD写中文字符串函数(24X24)
// 入口参数:
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           CharColor:      字符颜色.
//           CharBackColor:  背景颜色.
//           *sp  :          显示的中文字符串指针.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_WriteChinese24x24string(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  *sp                                                      // 中文字符指针
)
{
  LCD_SetColors(CharColor, CharBackColor);                          // 设置字体颜色和背景颜色
  LCD_SetFont(&Font16, &Font24_CN);                                 // 中文字体设置为24x24
  LCD_DisplayString(Xpos, Ypos, sp);                                // 显示字符串
  LCD_SetFont(&LCD_DEFAULT_FONT, &LCD_DEFAULT_FONT_CN);             // 默认字体
}


//-----------------------------------------------------------------
//void LCD_WriteNumChar (
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t CharColor,                                               // 字体颜色
//  uint16_t CharBackColor,                                           // 背景颜色
//  uint8_t  udata                                                    // 显示的数值
//)
//-----------------------------------------------------------------
//
// 函数功能: LCD显示字符数值
// 入口参数:
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           CharColor:      字符颜色.
//           CharBackColor:  背景颜色.
//           udata:          显示的数值.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 
//          (1) 显示的值为0~255之间
//          (2) 数据显示格式: 右对齐
//          (3) 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_WriteNumChar (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint8_t  udata                                                    // 显示的数值
)
{
  uint8_t display_buf[4];
  
  sprintf ((char *)display_buf, "%d", udata);
  
  LCD_WriteString(Xpos, Ypos, CharColor, CharBackColor, display_buf); 
}

//-----------------------------------------------------------------
//void LCD_WriteNumInt (
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t CharColor,                                               // 字体颜色
//  uint16_t CharBackColor,                                           // 背景颜色
//  uint16_t udata                                                    // 显示的数值
//)
//-----------------------------------------------------------------
//
// 函数功能: LCD显示整形数值
// 入口参数: 
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           CharColor:      字符颜色.
//           CharBackColor:  背景颜色.
//           udata:          显示的数值.
// 返回参数: 无
// 全局变量: 无
// 调用模块: LCD_WriteString().
// 注意事项: 
//          (1) 显示的值为0~65535之间
//          (2) 数据显示格式: 右对齐
//          (3) 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_WriteNumInt (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint16_t udata                                                    // 显示的数值
)
{
  uint8_t display_buf[6];
  
  sprintf ((char *)display_buf, "%5d", udata);
  
  LCD_WriteString(Xpos, Ypos, CharColor, CharBackColor, display_buf); 
}

//-----------------------------------------------------------------
//void LCD_WriteNumLong (
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t CharColor,                                               // 字体颜色
//  uint16_t CharBackColor,                                           // 背景颜色
//  uint32_t udata                                                    // 显示的数值
//)
//-----------------------------------------------------------------
//
// 函数功能: LCD显示整形数值
// 入口参数: 
//           Xpos:           左上角x起始坐标. 
//           Ypos:           左上角y起始坐标.
//           CharColor:      字符颜色.
//           CharBackColor:  背景颜色.
//           udata:          显示的数值.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 
//          (1) 显示的值为0~4294967296之间
//          (2) 数据显示格式: 右对齐
//          (3) 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_WriteNumLong (
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t CharColor,                                               // 字体颜色
  uint16_t CharBackColor,                                           // 背景颜色
  uint32_t udata                                                    // 显示的数值
)
{
  uint8_t display_buf[11];

  sprintf ((char *)display_buf, "%lld", (unsigned long long)udata);
  LCD_WriteString(Xpos, Ypos, CharColor, CharBackColor, display_buf); 
}


//-----------------------------------------------------------------
// void LCD_HLine(u16 x, u16 y, u16 Length, u16 Color)
//-----------------------------------------------------------------
//
// 函数功能: 绘制水平实线
// 入口参数: 
//           Xpos:        左上角x起始坐标. 
//           Ypos:        左上角y起始坐标.
//           Length:      线条长度.
//           LineColor:   线条颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_Line_H(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t Length,                                                  // 长度
  uint16_t LineColor                                                // 线条颜色
)
{
  LCD_SetColors ( LineColor, BackColor );
  LCD_DrawHLine ( Xpos, Ypos, Length );
}

//-----------------------------------------------------------------
//void LCD_DotLine_H(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t Length,                                                  // 长度
//  uint16_t LineColor,                                               // 线条颜色
//  uint16_t BackColor_                                               // 背景颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 绘制水平虚线
// 入口参数: 
//           Xpos:        左上角x起始坐标. 
//           Ypos:        左上角y起始坐标.
//           Length:      线条长度.
//           LineColor:   线条颜色.
//           BackColor_:  背景颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_DotLine_H(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t Length,                                                  // 长度
  uint16_t LineColor,                                               // 线条颜色
  uint16_t BackColor_                                               // 背景颜色
)
{
  LCD_SetColors ( LineColor, BackColor_ );
  LCD_DrawDotHLine ( Xpos, Ypos, Length );
}

//-----------------------------------------------------------------
//void LCD_Line_V(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t High,                                                    // 长度
//  uint16_t LineColor                                                // 线条颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 绘制垂直实线
// 入口参数: 
//           Xpos:       左上角x起始坐标. 
//           Ypos:       左上角y起始坐标.
//           High:       线条高度.
//           LineColor:  线条颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_Line_V(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 长度
  uint16_t LineColor                                                // 线条颜色
)
{
  LCD_SetColors ( LineColor, BackColor );
  LCD_DrawVLine ( Xpos, Ypos, High );
}

//-----------------------------------------------------------------
//void LCD_DotLine_V(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t High,                                                    // 长度
//  uint16_t LineColor,                                               // 线条颜色
//  uint16_t BackColor_                                               // 背景颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 绘制垂直虚线
// 入口参数: 
//           Xpos:       左上角x起始坐标. 
//           Ypos:       左上角y起始坐标.
//           High:       线条高度.
//           LineColor:  线条颜色.
//           BackColor_: 背景颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块: 
// 注意事项: 原点参考坐标为左上角, 显示坐标为左上角
//-----------------------------------------------------------------
void LCD_DotLine_V(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 长度
  uint16_t LineColor,                                               // 线条颜色
  uint16_t BackColor_                                               // 背景颜色
)
{
  LCD_SetColors ( LineColor, BackColor_ );
  LCD_DrawDotVLine ( Xpos, Ypos, High );
}


//-----------------------------------------------------------------
//void LCD_LineBox(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t Width,                                                   // 宽度
//  uint16_t High,                                                    // 高度
//  uint16_t LineColor                                                // 线条颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 绘制矩形边框
// 入口参数: 
//           Xpos:      左上角x起始坐标. 
//           Ypos:      左上角y起始坐标.
//           High:      矩形高度.
//           Width:     矩形宽度.
//           LineColor: 矩形线条颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块:
// 注意事项: 原点参考坐标为左上角, 矩形坐标为左上角
//-----------------------------------------------------------------
void LCD_LineBox(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 高度
  uint16_t Width,                                                   // 宽度
  uint16_t LineColor                                                // 线条颜色
)
{
  LCD_SetColors ( LineColor, BackColor );
  LCD_DrawRect ( Xpos, Ypos, High, Width );
}


//-----------------------------------------------------------------
//void LCD_FillBox(
//  uint16_t Xpos,                                                    // x轴坐标
//  uint16_t Ypos,                                                    // y轴坐标
//  uint16_t Width,                                                   // 宽度
//  uint16_t High,                                                    // 高度
//  uint16_t FillColor                                                // 线条颜色
//)
//-----------------------------------------------------------------
//
// 函数功能: 绘制实心矩形
// 入口参数: 
//           Xpos:      左上角x起始坐标. 
//           Ypos:      左上角y起始坐标.
//           High:      矩形高度.
//           Width:     矩形宽度.
//           FillColor: 矩形填充颜色.
// 返回参数: 无
// 全局变量: 无
// 调用模块:
// 注意事项:  原点参考坐标为左上角, 矩形坐标为左上角
//-----------------------------------------------------------------
void LCD_FillBox(
  uint16_t Xpos,                                                    // x轴坐标
  uint16_t Ypos,                                                    // y轴坐标
  uint16_t High,                                                    // 高度
  uint16_t Width,                                                   // 宽度
  uint16_t FillColor                                                // 线条颜色
)
{
  LCD_SetColors ( TextColor, FillColor );
  LCD_DrawFullRect ( Xpos, Ypos, High, Width );
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
