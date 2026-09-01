#include "usart.h"
#include "delay.h"
#include "protocol.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//串口2初始化 (PD5, PD6)		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2025/01/XX
//版本：V1.0 (修改为USART2 + 修复逐字节接收BUG)
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数，输出到USART2
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR & 0X40) == 0); //循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART2_RX   //如果使能了接收

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_STA = 0;       //接收状态标记	

u8 aRxBuffer[1]; // 【修复】每次只接收1个字节
UART_HandleTypeDef UART2_Handler; //UART句柄

//初始化IO 串口2 
//bound:波特率
void uart_init(u32 bound)
{	
	//UART 初始化设置
	UART2_Handler.Instance = USART2;					    //USART2
	UART2_Handler.Init.BaudRate = bound;				    //波特率
	UART2_Handler.Init.WordLength = UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits = UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity = UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode = UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART2
	
	// 【修复】开启 1字节 接收中断
	HAL_UART_Receive_IT(&UART2_Handler, aRxBuffer, 1); 
}

//UART底层初始化，时钟使能，引脚配置，中断配置
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance == USART2)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();			
		__HAL_RCC_USART2_CLK_ENABLE();			
	
		//PD5 TX
		GPIO_Initure.Pin = GPIO_PIN_5;			
		GPIO_Initure.Mode = GPIO_MODE_AF_PP;
		GPIO_Initure.Pull = GPIO_PULLUP;		
		GPIO_Initure.Speed = GPIO_SPEED_FAST;
		GPIO_Initure.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_Initure);	

		//PD6 RX
		GPIO_Initure.Pin = GPIO_PIN_6;			
		HAL_GPIO_Init(GPIOD, &GPIO_Initure);	
		
		HAL_NVIC_EnableIRQ(USART2_IRQn);		
		HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);
	}
}

// 【接收回调函数：正确逐字节解析】
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        parse_byte(aRxBuffer[0]);   // 解析收到的字节
        
        // 重新开启1字节接收，保证下一个字节能进来
        HAL_UART_Receive_IT(&UART2_Handler, aRxBuffer, 1);
      
//              // 原样回传可以
//        while((USART2->SR & 0X40) == 0);
//        USART2->DR = aRxBuffer[0];
//        HAL_UART_Receive_IT(&UART2_Handler, aRxBuffer, 1);
    }
}
 
// 串口2中断服务程序（极简版，无冲突）
void USART2_IRQHandler(void)                	
{ 
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART2_Handler);
	
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
} 
#endif  
