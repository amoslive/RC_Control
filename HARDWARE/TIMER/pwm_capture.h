#ifndef __PWM_CAPTURE_H
#define __PWM_CAPTURE_H

#include "sys.h"
#include "stm32f4xx_hal.h"

// 8路总通道（TIM8 4路 + TIM5 4路）
typedef enum
{
    // TIM8 通道 → 电机转速
    MOTOR_CH0 = 0,  // TIM8_CH1 → PI5
    MOTOR_CH1 = 1,  // TIM8_CH2 → PI6
    MOTOR_CH2 = 2,  // TIM8_CH3 → PI7
    MOTOR_CH3 = 3,  // TIM8_CH4 → PI2

    // TIM5 通道 → 手柄输入
    HAND_CH0  = 4,  // TIM5_CH1 → PH10
    HAND_CH1  = 5,  // TIM5_CH2 → PH11
    HAND_CH2  = 6,  // TIM5_CH3 → PH12
    HAND_CH3  = 7,  // TIM5_CH4 → PI0

    CAP_MAX
} CAP_CH_E;

// 捕获数据结构体
typedef struct
{
    uint8_t  cap_ok;
    uint32_t period_us;
    uint32_t high_us;
} CAP_Data_t;

// 两组独立数据
extern CAP_Data_t Motor_Data[4];  // 电机转速（TIM8 4路）
extern CAP_Data_t Hand_Data[4];   // 手柄输入（TIM5 4路）

// 初始化全部8路
void PWM_CAP_Init_All(void);

#endif