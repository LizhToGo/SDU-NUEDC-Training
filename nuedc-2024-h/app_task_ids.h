#ifndef APP_TASK_IDS_H
#define APP_TASK_IDS_H

#include <stdint.h>

/**
 * @brief UART、按键和任务调度器共用的公开任务编号。
 */
typedef enum {
    TASK_ID_NONE = 0,
    TASK_ID_1 = 1,
    TASK_ID_2 = 2,
    TASK_ID_3 = 3,
    TASK_ID_4 = 4,
    TASK_ID_STOP = 255
} task_id_t;

/* 将 1、10 这类十进制命令号转换为任务编号。 */
task_id_t task_uart_command_from_number(uint8_t number);

/* 将 0x01、0x10 这类原始二进制命令字节转换为任务编号。 */
task_id_t task_uart_command_from_hex_byte(uint8_t value);

/* 轮询一次 UART0，返回解析出的任务命令；没有命令时返回 TASK_ID_NONE。 */
task_id_t task_uart_read_command(uint8_t allow_binary_stop);

/* 等待 UART 或按键选择任务，等待期间保持刹车。 */
task_id_t wait_task_uart_command(void);

#endif /* APP_TASK_IDS_H */
