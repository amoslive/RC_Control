#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "stdint.h"

// 参数结构体（MCU上传 或 miniPC下发共用）
typedef struct {
    uint16_t wheel_speed[4];   // 索引0-3: 左前,右前,左后,右后 电机转速
    uint16_t steer_angle[4];   // 索引0-3: 左前,右前,左后,右后 转向角
} MotorData_t;

// 外部变量：接收完成标志，主循环检测到此标志为1时，读取 rx_data 中的数据
extern volatile uint8_t parse_complete_flag;
extern MotorData_t rx_data;    // 最新接收到的有效数据

// 函数声明
void protocol_init(void);                      // 初始化协议（清空状态）
void send_motor_data(const MotorData_t *data); // 发送一帧数据（MCU上传）
void parse_byte(uint8_t byte);                 // 解析单个字节（放在串口接收回调中）

#endif

