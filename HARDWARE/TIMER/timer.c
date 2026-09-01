#include "timer.h"
#include "led.h"

TIM_HandleTypeDef TIM2_Handler;   // 保留TIM2做PWM输出
TIM_HandleTypeDef TIM4_Handler;  // 新增TIM4做舵机PWM输出

// TIM2 PWM 初始化（全部4个通道），适配舵机
void TIM2_PWM_Init(u16 arr, u16 psc)
{
    TIM_OC_InitTypeDef tim_oc_cfg = {0};

    TIM2_Handler.Instance = TIM2;
    TIM2_Handler.Init.Prescaler = psc;
    TIM2_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2_Handler.Init.Period = arr;
    TIM2_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);

    // 通用配置：PWM模式1、高电平有效
    tim_oc_cfg.OCMode = TIM_OCMODE_PWM1;
    tim_oc_cfg.OCPolarity = TIM_OCPOLARITY_HIGH;

    // 配置通道1（PA0）- 默认占空比750（90°）
    tim_oc_cfg.Pulse = 495+52;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &tim_oc_cfg, TIM_CHANNEL_1);

    // 配置通道2（PA1）
    tim_oc_cfg.Pulse = 495+52;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &tim_oc_cfg, TIM_CHANNEL_2);

    // 配置通道3（PA2）
    tim_oc_cfg.Pulse = 495+52;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &tim_oc_cfg, TIM_CHANNEL_3);

    // 配置通道4（PA3）
    tim_oc_cfg.Pulse = 495+52;
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler, &tim_oc_cfg, TIM_CHANNEL_4);

    // 启动所有4个通道的PWM输出
    HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM2_Handler, TIM_CHANNEL_4);
}

// ===================== 新增 TIM4 PWM 初始化 =====================
// 对应引脚：
// TIM4_CH1 -> PD12
// TIM4_CH2 -> PD13
// TIM4_CH3 -> PD14
// TIM4_CH4 -> PD15
void TIM4_PWM_Init(u16 arr, u16 psc)
{
    TIM_OC_InitTypeDef tim_oc_cfg = {0};

    TIM4_Handler.Instance = TIM4;
    TIM4_Handler.Init.Prescaler = psc;
    TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM4_Handler.Init.Period = arr;
    TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM4_Handler);

    // 通用配置：PWM模式1、高电平有效
    tim_oc_cfg.OCMode = TIM_OCMODE_PWM1;
    tim_oc_cfg.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_cfg.Pulse = 1500;   // 默认全部初始化为 750（中间值）

    // 配置 CH1 (PD12)
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &tim_oc_cfg, TIM_CHANNEL_1);
    // 配置 CH2 (PD13)
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &tim_oc_cfg, TIM_CHANNEL_2);
    // 配置 CH3 (PD14)
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &tim_oc_cfg, TIM_CHANNEL_3);
    // 配置 CH4 (PD15)
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &tim_oc_cfg, TIM_CHANNEL_4);

    // 启动 PWM 输出
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_4);
}

// 定时器底层驱动（HAL 回调）
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;

    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();      // 使能 TIM2 时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();     // 使能 GPIOA 时钟

        // 配置 PA0 PA1 PA2 PA3
        GPIO_Initure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_HIGH;
        GPIO_Initure.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    }

    // ===================== 新增 TIM4 的 MSP 初始化 =====================
    if(htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        // PD12(CH1) PD13(CH2) PD14(CH3) PD15(CH4)
        GPIO_Initure.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_HIGH;
        GPIO_Initure.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_Initure);
    }
}

// TIM2 通道设置占空比
void TIM_SetTIM2Compare1(u32 compare) { TIM2->CCR1 = compare; }
void TIM_SetTIM2Compare2(u32 compare) { TIM2->CCR2 = compare; }
void TIM_SetTIM2Compare3(u32 compare) { TIM2->CCR3 = compare; }
void TIM_SetTIM2Compare4(u32 compare) { TIM2->CCR4 = compare; }

// ===================== 新增 TIM4 通道设置占空比 =====================
void TIM_SetTIM4Compare1(u32 compare) { TIM4->CCR1 = compare; }
void TIM_SetTIM4Compare2(u32 compare) { TIM4->CCR2 = compare; }
void TIM_SetTIM4Compare3(u32 compare) { TIM4->CCR3 = compare; }
void TIM_SetTIM4Compare4(u32 compare) { TIM4->CCR4 = compare; }