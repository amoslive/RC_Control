#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "protocol.h"
#include "pwm_capture.h"
#include "key.h"

MotorData_t tx_data;
uint16_t duty_angle[4];
uint32_t duty_speed[4];
uint16_t Duty_Steer;
uint16_t Duty_Wheel;
int i;
uint16_t PWM_val1_Inital = 495+52;   // 舵机起始位置
uint16_t PWM_val2_Inital = 1500;  // 电机起始位置
uint8_t ctrl_mode = 1;   // 0:手柄模式(默认)  1:上位机串口模式

int main(void)
{
    uint16_t target1;
    uint16_t target2;
    uint32_t led_blink_tick;
    u8 key;
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
    KEY_Scan(0);   // 清空一次按键状态，防止误触发
    KEY_Init();
    TIM2_PWM_Init(999, 272); // ARR=999，PSC=272，90MHz时钟
    TIM4_PWM_Init(20000 - 1, 90 - 1); 
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
      
 // -------------- 手柄平滑输出(采集输入部分) ------------------//
      target1 = Hand_Data[3].high_us / 2;
      Duty_Steer = ((target1 - 250) * 660) / 1000 + 165;// 标准 0.5~2.5ms → 输出 165 ~ 825

      target2 = Hand_Data[2].high_us / 2;
      Duty_Wheel = target2;      
      
 // --------------控制模式切换(LED全亮手柄控制模式、全灭为串口控制模式）-------//      
        key = KEY_Scan(0);  

        if(key == KEY0_PRES)  // PB2 按一下切换
        {          
            HAL_GPIO_TogglePin(GPIOG,
            GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
            GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8);
           ctrl_mode = !ctrl_mode;
            if(ctrl_mode == 1)
            {
              printf("已切换到：上位机模式\r\n");
            }
            else
            {
              printf("已切换到：手柄控制模式\r\n");
            }        
        }
     
        if(ctrl_mode == 1)
        {
            // --------------上位机指令(串口通讯部分)-------------------//
            if(parse_complete_flag == 1)
            {
                parse_complete_flag = 0;

                for(i=0; i<4; i++)
                {
                    if(rx_data.steer_angle[i] < 165) rx_data.steer_angle[i] = PWM_val1_Inital;   // 0.5ms ~ 2.5ms 对应 165 ~ 825
                    if(rx_data.steer_angle[i] > 825) rx_data.steer_angle[i] = PWM_val1_Inital;

                    if(rx_data.wheel_speed[i] < 250)  rx_data.wheel_speed[i] = PWM_val2_Inital;
                    if(rx_data.wheel_speed[i] > 1250) rx_data.wheel_speed[i] = PWM_val2_Inital;
                }
                TIM_SetTIM2Compare1(rx_data.steer_angle[2]+52);
                TIM_SetTIM2Compare2(rx_data.steer_angle[3]+52);
                TIM_SetTIM2Compare3(rx_data.steer_angle[0]+52);
                TIM_SetTIM2Compare4(rx_data.steer_angle[1]+52);
//              delay_ms(10);       //延时以防卡顿
                TIM_SetTIM4Compare1(2*rx_data.wheel_speed[3]); // 输出手柄通道2采集值 （电机1）
                TIM_SetTIM4Compare2(2*rx_data.wheel_speed[2]); //（电机2）
                TIM_SetTIM4Compare3(2*rx_data.wheel_speed[1]); //（电机3）
                TIM_SetTIM4Compare4(2*rx_data.wheel_speed[0]); //（电机4）
//              delay_ms(10);  //延时以防卡顿

                for(i = 0; i < 4; i++) //填充电机周期值
                {
                   tx_data.wheel_speed[i] = (1000000 / Motor_Data[i].period_us) * 120 / 14;
                   tx_data.steer_angle[i] = ((uint32_t)(rx_data.steer_angle[i] - 165) * 180UL) / 660-90;
                  
                }
//                   printf("freq_val=%d\r\n",  2*rx_data.wheel_speed[0]); 
//                   printf("freq_val=%d\r\n",  2*rx_data.wheel_speed[1]); 
                
            }
              send_motor_data(&tx_data); 
              delay_ms(10);  //延时以防卡顿            
        }
 
       else{ 

        TIM_SetTIM2Compare1(Duty_Steer+52);  //（舵机1）输出手柄通道1采集值
        TIM_SetTIM2Compare2(Duty_Steer+52);  //（舵机2）
        TIM_SetTIM2Compare3(Duty_Steer+52);  //（舵机3）
        TIM_SetTIM2Compare4(Duty_Steer+52);  //（舵机4）
        delay_ms(10);  //延时以防卡顿 
        TIM_SetTIM4Compare1(2*Duty_Wheel); //（电机1）输出手柄通道2采集值 
        TIM_SetTIM4Compare2(2*Duty_Wheel); //（电机2）
        TIM_SetTIM4Compare3(2*Duty_Wheel); //（电机3）
        TIM_SetTIM4Compare4(2*Duty_Wheel); //（电机4）
        delay_ms(10);  //延时以防卡顿
         
        // // -------------- 手柄数据打包上传上位机 ------------------//
        for(i = 0; i < 4; i++) //填充电机周期值
        {
          tx_data.wheel_speed[i] = (1000000 / Motor_Data[i].period_us) * 120 / 14;
          tx_data.steer_angle[i] =  ((uint32_t)(Duty_Steer - 165) * 180UL) / 660-90;
        }       
//        printf("freq_val=%d\r\n", 2*target2); 
             
       }
        send_motor_data(&tx_data); 
        delay_ms(10);  //延时以防卡顿         
    }
}
