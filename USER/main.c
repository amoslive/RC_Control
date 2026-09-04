#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "protocol.h"
#include "pwm_capture.h"

#define MOTOR_POLE_COUNT        14U
#define MOTOR_SIGNAL_TIMEOUT_MS 100U
#define CONTROL_SIGNAL_TIMEOUT_MS 1500U
#define ESC_NEUTRAL_PULSE_US      1500U
#define WHEEL_COMMAND_NEUTRAL     (ESC_NEUTRAL_PULSE_US / 2U)

MotorData_t tx_data;
int i;
uint16_t PWM_val1_Inital = 495+52;   // 舵机起始位置
uint16_t PWM_val2_Inital = WHEEL_COMMAND_NEUTRAL; // protocol value; doubled to 1500 us

static void Motor_OutputNeutral(void)
{
    TIM_SetTIM4Compare1(ESC_NEUTRAL_PULSE_US);
    TIM_SetTIM4Compare2(ESC_NEUTRAL_PULSE_US);
    TIM_SetTIM4Compare3(ESC_NEUTRAL_PULSE_US);
    TIM_SetTIM4Compare4(ESC_NEUTRAL_PULSE_US);
}

static uint16_t Motor_GetSpeedRPM(uint8_t motor_index)
{
    uint32_t period_us;

    if(motor_index >= 4U)
        return 0U;

    if((Motor_Data[motor_index].cap_ok == 0U) ||
       ((HAL_GetTick() - Motor_Data[motor_index].last_capture_ms) > MOTOR_SIGNAL_TIMEOUT_MS))
        return 0U;

    period_us = Motor_Data[motor_index].period_us;
    if(period_us == 0U)
        return 0U;

    // RPM = 频率 * 120 / 磁极数；先合并常数以减少整数截断误差
    return (uint16_t)((1000000UL * 120UL) /
                      (period_us * MOTOR_POLE_COUNT));
}

int main(void)
{
    uint32_t led_blink_tick;
    uint32_t last_command_tick;
    uint8_t command_active = 0U;
    HAL_Init();
    Stm32_Clock_Init(360,12,2,8);
    delay_init(180);
    uart_init(115200);
    LED_Init();
    HAL_GPIO_WritePin(GPIOG,
        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8,
        GPIO_PIN_SET);
    led_blink_tick = HAL_GetTick();
    TIM2_PWM_Init(999, 272); // ARR=999，PSC=272，90MHz时钟
    TIM4_PWM_Init(20000 - 1, 90 - 1);
    Motor_OutputNeutral();
    last_command_tick = HAL_GetTick();
    protocol_init();
    PWM_CAP_Init_All();

    while(1)
    {
        // 烧录指示：PG1~PG8 每 500 ms 翻转一次
        if((HAL_GetTick() - led_blink_tick) >= 500)
        {
            led_blink_tick = HAL_GetTick();
            HAL_GPIO_TogglePin(GPIOG,
                GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8);
        }

        // -------------- 上位机指令（固定为串口控制模式）--------------
        if(parse_complete_flag == 1)
        {
            parse_complete_flag = 0;
            last_command_tick = HAL_GetTick();
            command_active = 1U;

            for(i=0; i<4; i++)
            {
                if(rx_data.steer_angle[i] < 165) rx_data.steer_angle[i] = PWM_val1_Inital;
                if(rx_data.steer_angle[i] > 825) rx_data.steer_angle[i] = PWM_val1_Inital;

                if(rx_data.wheel_speed[i] < 250)  rx_data.wheel_speed[i] = PWM_val2_Inital;
                if(rx_data.wheel_speed[i] > 1250) rx_data.wheel_speed[i] = PWM_val2_Inital;
            }
            TIM_SetTIM2Compare1(rx_data.steer_angle[2]+52);
            TIM_SetTIM2Compare2(rx_data.steer_angle[3]+52);
            TIM_SetTIM2Compare3(rx_data.steer_angle[0]+52);
            TIM_SetTIM2Compare4(rx_data.steer_angle[1]+52);

            TIM_SetTIM4Compare1(2*rx_data.wheel_speed[3]); // 电机1
            TIM_SetTIM4Compare2(2*rx_data.wheel_speed[2]); // 电机2
            TIM_SetTIM4Compare3(2*rx_data.wheel_speed[1]); // 电机3
            TIM_SetTIM4Compare4(2*rx_data.wheel_speed[0]); // 电机4
        }

        // Stop all motors if valid serial commands disappear for 1.5 seconds.
        if((command_active != 0U) &&
           ((HAL_GetTick() - last_command_tick) > CONTROL_SIGNAL_TIMEOUT_MS))
        {
            Motor_OutputNeutral();
            command_active = 0U;
        }

        // -------------- 实时状态上传上位机 ------------------
        for(i = 0; i < 4; i++)
        {
            tx_data.wheel_speed[i] = Motor_GetSpeedRPM(i);
            tx_data.steer_angle[i] = ((uint32_t)(rx_data.steer_angle[i] - 165) * 180UL) / 660-90;
        }
        send_motor_data(&tx_data);
        delay_ms(10);
    }
}
