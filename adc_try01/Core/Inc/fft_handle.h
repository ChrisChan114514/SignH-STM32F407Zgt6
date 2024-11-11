#ifndef __FFT_HANDLE_H
#define __FFT_HANDLE_H

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


float fft_frequency_Vpp_check(uint16_t fft_index_max);
uint16_t fft_waveform_check(uint16_t fft_index_max);
void get_vpp(void);
void fftCalculate(void);
void SamplingFrequency_Check(float fft_frequency);


#endif


