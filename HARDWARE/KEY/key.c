#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//KEY驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/5
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//按键初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();           // 【新增】开启GPIOB时钟（PB2需要）    
    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    
    GPIO_Initure.Pin=GPIO_PIN_13;           //PC13
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
    
    GPIO_Initure.Pin=GPIO_PIN_2;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;
    GPIO_Initure.Pull=GPIO_PULLUP;      // 上拉（按下=0，松开=1）
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

// 标准单击扫描：按下立刻检测，松手不重复触发
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;
    if(mode) key_up=1;
    
    // 按键按下瞬间触发
    if(key_up && KEY0 == 0)
    {
        delay_ms(10);
        if(KEY0 == 0)
        {
            key_up = 0;
            return KEY0_PRES;
        }
    }
    // 按键松开恢复标志
    else if(KEY0 == 1)
    {
        key_up = 1;
    }
    return 0;
}