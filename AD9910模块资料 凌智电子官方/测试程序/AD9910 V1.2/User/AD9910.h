//-----------------------------------------------------------------
// AD9910程序
// 头文件名: AD9910.h
// 作    者: 凌智电子
// 开始日期: 2019-12-1
// 完成日期: 2019-12-25
// 当前版本: V1.0
// 历史版本:
//-----------------------------------------------------------------
#ifndef _AD9910_H
#define _AD9910_H

#include <stm32f10x.h>

#define AD9910_MRT_Set (GPIO_SetBits(GPIOB,GPIO_Pin_15))         
#define AD9910_MRT_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_15))

#define AD9910_CSN_Set (GPIO_SetBits(GPIOB,GPIO_Pin_14))         
#define AD9910_CSN_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_14))

#define AD9910_SCK_Set (GPIO_SetBits(GPIOB,GPIO_Pin_13))         
#define AD9910_SCK_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_13))

#define AD9910_SDI_Set (GPIO_SetBits(GPIOB,GPIO_Pin_12))         
#define AD9910_SDI_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_12))

#define AD9910_IUP_Set (GPIO_SetBits(GPIOB,GPIO_Pin_11))         
#define AD9910_IUP_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_11))

#define AD9910_DRH_Set (GPIO_SetBits(GPIOB,GPIO_Pin_10))         
#define AD9910_DRH_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_10))

#define AD9910_DRC_Set (GPIO_SetBits(GPIOB,GPIO_Pin_9))         
#define AD9910_DRC_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_9))

#define AD9910_PF0_Set (GPIO_SetBits(GPIOB,GPIO_Pin_8))         
#define AD9910_PF0_Clr (GPIO_ResetBits(GPIOB,GPIO_Pin_8))

#define AD9910_PF1_Set (GPIO_SetBits(GPIOA,GPIO_Pin_7))         
#define AD9910_PF1_Clr (GPIO_ResetBits(GPIOA,GPIO_Pin_7))

#define AD9910_PF2_Set (GPIO_SetBits(GPIOA,GPIO_Pin_6))         
#define AD9910_PF2_Clr (GPIO_ResetBits(GPIOA,GPIO_Pin_6))

#define AD9910_OSK_Set (GPIO_SetBits(GPIOC,GPIO_Pin_7))      
#define AD9910_OSK_Clr (GPIO_ResetBits(GPIOC,GPIO_Pin_7))

#define AD9910_TE_Set (GPIO_SetBits(GPIOA,GPIO_Pin_15))         
#define AD9910_TE_Clr (GPIO_ResetBits(GPIOA,GPIO_Pin_15))

#define AD9910_PC_Set (GPIO_SetBits(GPIOA,GPIO_Pin_14))         
#define AD9910_PC_Clr (GPIO_ResetBits(GPIOA,GPIO_Pin_14))

extern void GPIO_Init_AD9910(void);

extern void Write_8bit(u8 dat)	;
extern void Write_32bit(u32 dat)	;

extern void AD9910_Init(void);

extern void AD9910_Singal_Profile_Init(void);
extern void AD9910_Singal_Profile_Set(u8 addr,u32 Freq,u16 Amp ,u16 Pha);
extern void Set_Profile(u8 num);

extern void AD9910_Osk_Init(void);
extern void AD9910_Osk_Set(void);

extern void AD9910_DRG_Fre_Init(void);
extern void AD9910_DRG_Freq_set(u32 upper_limit , u32 lower_limit ,u32 dec_step , u32 inc_step , u16 neg_rate ,u16 pos_rate);

extern void AD9910_DRG_AMP_Init(void);
extern void AD9910_DRG_Amp_Set( u32 upper_limit , u32 lower_limit ,u32 dec_step , u32 inc_step , u16 neg_rate ,u16 pos_rate);

extern void AD9910_RAM_Init(void);
extern void AD9910_RAM_ZB_Fre_Init(void);
extern void AD9910_RAM_ZB_Fre_Set(u32 Freq);

extern void AD9910_RAM_Fre_W(void);
extern void AD9910_RAM_AMP_W(void);
extern void AD9910_WAVE_RAM_AMP_W(int mode);
extern void AD9910_RAM_DIR_Fre_R(void);
extern void AD9910_RAM_DIR_AMP_R(void);

extern void AD9910_RAM_RAMP_UP_ONE_Fre_R(void);
extern void AD9910_RAM_RAMP_UP_ONE_AMP_R(void);

extern void AD9910_RAM_RAMP_UP_TWO_Fre_R(void);
extern void AD9910_RAM_RAMP_UP_TWO_AMP_R(void);

extern void AD9910_RAM_BID_RAMP_Fre_R(void);
extern void AD9910_RAM_BID_RAMP_AMP_R(void);

extern void AD9910_RAM_CON_BID_RAMP_Fre_R(void);
extern void AD9910_RAM_CON_BID_RAMP_AMP_R(void);

extern void AD9910_RAM_CON_RECIR_Fre_R(void);
extern void AD9910_RAM_CON_RECIR_AMP_R(void);

extern void AD9910_DRG_Pha_Init(void);
extern void AD9910_DRG_Pha_Set( u32 upper_limit , u32 lower_limit ,u32 dec_step , u32 inc_step , u16 neg_rate ,u16 pos_rate);

extern void AD9910_RAM_Pha_W(void);
extern void AD9910_RAM_DIR_PHA_R(void);
extern void AD9910_RAM_RAMP_UP_ONE_PHA_R(void);
extern void AD9910_RAM_RAMP_UP_TWO_PHA_R(void);
extern void AD9910_RAM_BID_RAMP_PHA_R(void);
extern void AD9910_RAM_CON_BID_RAMP_PHA_R(void);
extern void AD9910_RAM_CON_RECIR_PHA_R(void);

void Par_mod(u8 des ,u16 FF);
void AD9910_Init_Sin(int gain);
void Freq_convert(u32 Freq);
#endif
