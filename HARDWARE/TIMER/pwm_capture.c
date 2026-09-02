#include "pwm_capture.h"

// TIM8 + TIM5 两个定时器
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim5;

// 两组独立数据
CAP_Data_t Motor_Data[4] = {0};  // 电机转速
CAP_Data_t Hand_Data[4]  = {0};  // 手柄输入

static uint32_t cap_rise[CAP_MAX]   = {0};
static uint8_t  edge_sta[CAP_MAX]   = {0};
static uint32_t last_rise[CAP_MAX] = {0};

// 通道映射
static const uint32_t ch_tim[CAP_MAX] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,  // TIM8
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4   // TIM5
};

// 统一捕获处理
static void CAP_Process(TIM_HandleTypeDef *htim, CAP_CH_E ch)
{
    uint32_t tim_ch = ch_tim[ch];
    uint32_t val = HAL_TIM_ReadCapturedValue(htim, tim_ch);
    CAP_Data_t *data;

    // 指向对应数据组
    if(ch < 4) data = &Motor_Data[ch];
    else       data = &Hand_Data[ch - 4];

    if(edge_sta[ch] == 0)
    {
        // 上升沿：算周期
        cap_rise[ch] = val;
        if(val >= last_rise[ch])
            data->period_us = val - last_rise[ch];
        else
            data->period_us = (0xFFFF - last_rise[ch]) + val;

        last_rise[ch] = val;
        edge_sta[ch] = 1;

        // 切换下降沿
        TIM_RESET_CAPTUREPOLARITY(htim, tim_ch);
        TIM_SET_CAPTUREPOLARITY(htim, tim_ch, TIM_ICPOLARITY_FALLING);
    }
    else
    {
        // 下降沿：算高电平
        if(val >= cap_rise[ch])
            data->high_us = val - cap_rise[ch];
        else
        data->high_us = (0xFFFF - cap_rise[ch]) + val;

        data->cap_ok = 1;
        data->last_capture_ms = HAL_GetTick();
        edge_sta[ch] = 0;

        // 切回上升沿
        TIM_RESET_CAPTUREPOLARITY(htim, tim_ch);
        TIM_SET_CAPTUREPOLARITY(htim, tim_ch, TIM_ICPOLARITY_RISING);
    }
}

// ===================== TIM8 初始化（电机转速：PI5/PI6/PI7/PI2）=====================
void TIM8_Cap_Init(void)
{
    TIM_IC_InitTypeDef ic_conf = {0};
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 180 - 1;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFF;
    HAL_TIM_IC_Init(&htim8);

    ic_conf.ICPolarity  = TIM_ICPOLARITY_RISING;
    ic_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic_conf.ICPrescaler = TIM_ICPSC_DIV1;
    ic_conf.ICFilter    = 0x0F;

    HAL_TIM_IC_ConfigChannel(&htim8, &ic_conf, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim8, &ic_conf, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&htim8, &ic_conf, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&htim8, &ic_conf, TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
}

// ===================== TIM5 初始化（手柄输入：PH10/PH11/PH12/PI0）=====================
void TIM5_Cap_Init(void)
{
    TIM_IC_InitTypeDef ic_conf = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 90 - 1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFF;
    HAL_TIM_IC_Init(&htim5);

    ic_conf.ICPolarity  = TIM_ICPOLARITY_RISING;
    ic_conf.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic_conf.ICPrescaler = TIM_ICPSC_DIV1;
    ic_conf.ICFilter    = 0x0F;

    HAL_TIM_IC_ConfigChannel(&htim5, &ic_conf, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&htim5, &ic_conf, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&htim5, &ic_conf, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&htim5, &ic_conf, TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
}

// ===================== GPIO + 时钟 + 中断 =====================
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_conf = {0};

    if(htim->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();

        gpio_conf.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        gpio_conf.Mode = GPIO_MODE_AF_PP;
        gpio_conf.Pull = GPIO_PULLUP;
        gpio_conf.Speed = GPIO_SPEED_HIGH;
        gpio_conf.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOI, &gpio_conf);

        HAL_NVIC_SetPriority(TIM8_CC_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    }

    if(htim->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();

        // PH10/PH11/PH12
        gpio_conf.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
        gpio_conf.Mode = GPIO_MODE_AF_PP;
        gpio_conf.Pull = GPIO_PULLUP;
        gpio_conf.Speed = GPIO_SPEED_HIGH;
        gpio_conf.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &gpio_conf);

        // PI0
        gpio_conf.Pin = GPIO_PIN_0;
        gpio_conf.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOI, &gpio_conf);

        HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
    }
}

// ===================== 中断服务函数 =====================
void TIM8_CC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim8); }
void TIM5_IRQHandler(void)    { HAL_TIM_IRQHandler(&htim5); }

// ===================== 捕获回调 =====================
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM8)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) CAP_Process(htim, MOTOR_CH0);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) CAP_Process(htim, MOTOR_CH1);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) CAP_Process(htim, MOTOR_CH2);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) CAP_Process(htim, MOTOR_CH3);
    }

    if(htim->Instance == TIM5)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) CAP_Process(htim, HAND_CH0);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) CAP_Process(htim, HAND_CH1);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) CAP_Process(htim, HAND_CH2);
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) CAP_Process(htim, HAND_CH3);
    }
}

// ===================== 统一初始化 =====================
void PWM_CAP_Init_All(void)
{
    TIM8_Cap_Init();
    TIM5_Cap_Init();
}

// 溢出中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){} 
