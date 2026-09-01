#include "protocol.h"
#include "usart.h"   // 使用 USART2 发送

// 接收状态机状态
typedef enum {
    STATE_IDLE,          // 空闲，等待帧头0x5A
    STATE_HEADER2,       // 收到第一个0x5A，等待第二个0x5A
    STATE_DATA           // 收数据（包含2个帧头 + 16个数据字节 + 1校验和，共19字节）
} RxState_t;

static RxState_t rx_state = STATE_IDLE;
static uint8_t rx_buffer[19];        // 完整帧缓冲区：2帧头 + 8参数(2字节/参数) + 1校验和 = 2+16+1=19
static uint8_t rx_index = 0;
static uint8_t calc_checksum(uint8_t *data, uint8_t len);  // 计算校验和

volatile uint8_t parse_complete_flag = 0;
MotorData_t rx_data = {0};

// 初始化协议
void protocol_init(void)
{
    rx_state = STATE_IDLE;
    rx_index = 0;
    parse_complete_flag = 0;
}

// 计算校验和（不包含最后的校验和字节）
static uint8_t calc_checksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    uint8_t i;
    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    return ~sum;  // 取反后返回低8位
}

// 发送一帧数据（数据部分低位在前，高位在后）
void send_motor_data(const MotorData_t *data)
{
    uint8_t tx_buf[19];
    uint8_t idx = 0;
    uint8_t i;
    uint8_t chksum;
    
    // 帧头
    tx_buf[idx++] = 0x5A;
    tx_buf[idx++] = 0x5A;
    
    // 参数：4个转速 + 4个转向角，共16字节（低字节在前）
    // 转速部分
    for (i = 0; i < 4; i++) {
        tx_buf[idx++] = (uint8_t)(data->wheel_speed[i] & 0xFF);      // 低位
        tx_buf[idx++] = (uint8_t)((data->wheel_speed[i] >> 8) & 0xFF); // 高位
    }
    // 转向角部分
    for (i = 0; i < 4; i++) {
        tx_buf[idx++] = (uint8_t)(data->steer_angle[i] & 0xFF);
        tx_buf[idx++] = (uint8_t)((data->steer_angle[i] >> 8) & 0xFF);
    }
    
    // 计算校验和（字节0 ~ 18-1 = 前18个字节）
    chksum = calc_checksum(tx_buf, 18);
    tx_buf[idx++] = chksum;   // 最后一位校验和
    
    // 通过 USART2 发送
    for (i = 0; i < 19; i++) {
        while((USART2->SR & 0X40) == 0); // 等待发送完成
        USART2->DR = tx_buf[i];
    }
}

// 接收单个字节解析（放在串口接收中断的回调中）
void parse_byte(uint8_t byte)
{
    uint8_t recv_chksum;
    uint8_t calc_chksum;
    uint8_t *ptr;
    uint8_t i;
    
    switch (rx_state) {
        case STATE_IDLE:
            if (byte == 0x5A) {
                rx_state = STATE_HEADER2;
                rx_index = 0;
                rx_buffer[rx_index++] = byte;
            }
            break;
            
        case STATE_HEADER2:
            rx_buffer[rx_index++] = byte;
            if (byte == 0x5A) {
                rx_state = STATE_DATA;
            } else {
                rx_state = STATE_IDLE;   // 第二个字节不是0x5A，放弃
                rx_index = 0;
            }
            break;
            
        case STATE_DATA:
            rx_buffer[rx_index++] = byte;
            if (rx_index == 19) {  // 收满19字节（2头+16数据+1校验）
                // 验证校验和
                recv_chksum = rx_buffer[18];
                calc_chksum = calc_checksum(rx_buffer, 18);
                if (recv_chksum == calc_chksum) {
                    // 解析数据：跳过2字节帧头，读取16字节参数
                    ptr = &rx_buffer[2];
                    for (i = 0; i < 4; i++) {
                        // 低位在前，高位在后
                        rx_data.wheel_speed[i] = ptr[0] | (ptr[1] << 8);
                        ptr += 2;
                    }
                    for (i = 0; i < 4; i++) {
                        rx_data.steer_angle[i] = ptr[0] | (ptr[1] << 8);
                        ptr += 2;
                    }
                    parse_complete_flag = 1;  // 通知主循环接收到新数据
                }
                // 无论校验成功与否，都回到空闲状态
                rx_state = STATE_IDLE;
                rx_index = 0;
            }
            break;
            
        default:
            rx_state = STATE_IDLE;
            rx_index = 0;
            break;
    }
}