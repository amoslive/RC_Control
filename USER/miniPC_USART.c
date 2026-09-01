#include "miniPC_USART.h"

// 引用外部定义的 huart2（在 main.c 或 CubeMX 生成的代码中已初始化）
extern UART_HandleTypeDef huart2;

// ---------- 接收缓冲区（静态变量，仅本文件内可见）----------
static uint8_t rx_buf[MINIPC_FRAME_LEN];   // 存储接收到的原始字节
static uint8_t rx_idx = 0;                 // 当前接收位置索引
static uint8_t rx_state = 0;               // 接收状态机：0=空闲,1=已收到第一个0x5A,2=已收到帧头且正在收数据
static uint8_t frame_ready = 0;            // 标志位：1表示已收到完整且校验正确的一帧
static uint16_t rx_params[MINIPC_PARAM_NUM]; // 存储解析后的参数值（16位整数）

// 校验和计算函数
// 参数：data 要计算校验和的字节数组，len 数组长度（字节数）
// 返回值：对前 len 个字节求和后取反（只有低8位有效）
static uint8_t calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];      // 逐字节累加
    }
    return ~sum;             // 取反并返回（自动截断为低8位）
}

// 解析一个接收到的字节（在串口中断中被调用）
static void parse_byte(uint8_t data) {
    // ---------- 状态机 ----------
    // 状态0：等待帧头第一个字节 0x5A
    if (rx_state == 0) {
        if (data == 0x5A) {                // 找到了第一个帧头字节
            rx_state = 1;                  // 进入状态1，等待第二个帧头
            rx_idx = 0;                    // 清空索引
            rx_buf[rx_idx++] = data;       // 保存第一个字节
        }
    } 
    // 状态1：已经收到 0x5A，等待第二个 0x5A
    else if (rx_state == 1) {
        if (data == 0x5A) {                // 连续收到第二个 0x5A，帧头正确
            rx_state = 2;                  // 进入状态2，开始接收数据部分
            rx_buf[rx_idx++] = data;       // 保存第二个帧头字节
        } 
        else {
            // 第二个字节不是 0x5A，说明不是有效帧头
            // 但有可能当前字节恰好是下一个帧的起始？需要重新评估
            if (data == 0x5A) {
                // 如果这个字节是 0x5A，则重新开始但保留这个字节作为第一个帧头
                rx_state = 1;
                rx_idx = 0;
                rx_buf[rx_idx++] = data;
            } else {
                rx_state = 0;              // 完全丢弃，回到空闲状态
                rx_idx = 0;
            }
        }
    } 
    // 状态2：帧头已收完，正在接收剩余的数据（参数+校验和）
    else if (rx_state == 2) {
        rx_buf[rx_idx++] = data;            // 保存当前字节
        // 当收集的字节数等于一帧总长度时，表示一帧接收完成
        if (rx_idx == MINIPC_FRAME_LEN) {
            // 校验：对前 18 个字节（不含最后一个校验和字节）计算校验和
            uint8_t calc_cs = calc_checksum(rx_buf, MINIPC_FRAME_LEN - 1);
            // 如果计算出的校验和等于帧的最后一个字节，说明数据正确
            if (calc_cs == rx_buf[MINIPC_FRAME_LEN - 1]) {
                // 解析参数：每个参数两个字节，低字节在前，高字节在后
                for (uint8_t i = 0; i < MINIPC_PARAM_NUM; i++) {
                    uint8_t lo = rx_buf[2 + i*2];      // 低字节
                    uint8_t hi = rx_buf[2 + i*2 + 1];  // 高字节
                    rx_params[i] = (uint16_t)(lo | (hi << 8)); // 组合成16位整数
                }
                frame_ready = 1;  // 标记新帧到达
            }
            // 重置状态机，准备接收下一帧
            rx_state = 0;
            rx_idx = 0;
        }
    }
}

// 串口2的中断服务函数（需要在启动文件中关联，或直接放在这里）
void USART2_IRQHandler(void) {
    // 检查是否发生了接收数据寄存器非空中断
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        // 读取接收到的字节（读取 DR 寄存器会自动清除标志）
        uint8_t data = (uint8_t)(huart2.Instance->DR & 0xFF);
        // 交给状态机解析
        parse_byte(data);
        // 清除标志（上面读取 DR 后硬件已清除，但再手动清除一次也无妨）
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
    }
    // 调用 HAL 库默认的中断处理（以便处理错误等）
    HAL_UART_IRQHandler(&huart2);
}

// 初始化函数：使能串口2的接收中断，并设置中断优先级
void MiniPC_Init(void) {
    // 使能接收中断（RXNE 中断）
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    // 设置中断优先级
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    // 使能 NVIC 中的 USART2 中断
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// 发送一帧数据
// 参数：params 指向包含8个 uint16_t 数据的数组
void MiniPC_SendFrame(uint16_t *params) {
    uint8_t tx_buf[MINIPC_FRAME_LEN];  // 发送缓冲区
    // 填充帧头
    tx_buf[0] = 0x5A;
    tx_buf[1] = 0x5A;
    // 填充 8 个参数（每个参数低字节在前，高字节在后）
    for (uint8_t i = 0; i < MINIPC_PARAM_NUM; i++) {
        tx_buf[2 + i*2]     = (uint8_t)(params[i] & 0xFF);      // 低8位
        tx_buf[2 + i*2 + 1] = (uint8_t)((params[i] >> 8) & 0xFF); // 高8位
    }
    // 计算校验和（对前 18 个字节计算）
    tx_buf[MINIPC_FRAME_LEN - 1] = calc_checksum(tx_buf, MINIPC_FRAME_LEN - 1);
    // 通过串口2发送数据（阻塞方式，超时100ms）
    HAL_UART_Transmit(&huart2, tx_buf, MINIPC_FRAME_LEN, 100);
}

// 获取接收到的上一帧数据
// 参数：out_params 用于存放接收到的8个参数
// 返回值：1 表示有新的帧数据并被复制到 out_params；0 表示没有新帧
uint8_t MiniPC_GetReceivedFrame(uint16_t *out_params) {
    if (frame_ready) {
        // 将内部保存的参数复制到用户提供的缓冲区
        for (uint8_t i = 0; i < MINIPC_PARAM_NUM; i++) {
            out_params[i] = rx_params[i];
        }
        frame_ready = 0;   // 清除标志，表示已读取
        return 1;
    }
    return 0;
}