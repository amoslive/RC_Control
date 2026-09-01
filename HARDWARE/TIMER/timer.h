#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
#include "stm32f4xx_hal.h"

// TIM2 PWM 初始化
void TIM2_PWM_Init(u16 arr, u16 psc);

// TIM4 PWM 初始化 (PD12~PD15)
void TIM4_PWM_Init(u16 arr, u16 psc);

// 设置 TIM2 通道占空比
void TIM_SetTIM2Compare1(u32 compare);
void TIM_SetTIM2Compare2(u32 compare);
void TIM_SetTIM2Compare3(u32 compare);
void TIM_SetTIM2Compare4(u32 compare);

// 设置 TIM4 通道占空比
void TIM_SetTIM4Compare1(u32 compare);
void TIM_SetTIM4Compare2(u32 compare);
void TIM_SetTIM4Compare3(u32 compare);
void TIM_SetTIM4Compare4(u32 compare);

#endif