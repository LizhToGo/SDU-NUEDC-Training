#ifndef APP_TASK_IDS_H
#define APP_TASK_IDS_H

#include <stdint.h>

typedef enum {
    TASK_ID_NONE = 0,
    TASK_ID_1 = 1,
    TASK_ID_2 = 2,
    TASK_ID_3 = 3,
    TASK_ID_4 = 4,
    TASK_ID_5 = 5,
    TASK_ID_6 = 6,
    TASK_ID_7 = 7,
    TASK_ID_10 = 10,
    TASK_ID_11 = 11,
    TASK_ID_STOP = 255
} task_id_t;

task_id_t task_uart_command_from_number(uint8_t number);
task_id_t task_uart_command_from_hex_byte(uint8_t value);
task_id_t task_uart_read_command(uint8_t allow_binary_stop);
task_id_t wait_task_uart_command(void);

#endif /* APP_TASK_IDS_H */
