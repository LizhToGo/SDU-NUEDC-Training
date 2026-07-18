#include "app_task_ids.h"

#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"

/*
 * UART0 支持两类命令格式：
 * - 二进制字节：0x01..0x04
 * - ASCII 十进制： "01".."04"，也允许前缀 't'
 * 只有 allow_binary_stop 置位时，字节 0x00 才会被当成 STOP。
 */
typedef struct {
    uint8_t value;
    task_id_t task_id;
} task_uart_command_map_t;

/**
 * @brief 解析 "03"、"t10" 这类 ASCII 命令的小缓冲区。
 */
typedef struct {
    uint8_t frame_buf[3];
    uint8_t frame_len;
} task_uart_parse_state_t;

/* 文本输入路径共用的十进制命令映射表。 */
static const task_uart_command_map_t g_task_uart_number_map[] = {
    {0U, TASK_ID_STOP},
    {1U, TASK_ID_1},
    {2U, TASK_ID_2},
    {3U, TASK_ID_3},
    {4U, TASK_ID_4},
};

/* 原始 UART 字节使用的二进制命令映射表。 */
static const task_uart_command_map_t g_task_uart_binary_map[] = {
    {0x01U, TASK_ID_1},
    {0x02U, TASK_ID_2},
    {0x03U, TASK_ID_3},
    {0x04U, TASK_ID_4},
};

/**
 * @brief 在静态 UART 命令映射表中查找任务编号。
 */
static task_id_t task_uart_lookup_command(
    const task_uart_command_map_t *map,
    uint32_t map_len,
    uint8_t value)
{
    uint32_t i;

    for (i = 0U; i < map_len; i++) {
        if (map[i].value == value) {
            return map[i].task_id;
        }
    }

    return TASK_ID_NONE;
}

/**
 * @brief 复位增量式 ASCII 解析器状态。
 */
static void task_uart_parse_reset(task_uart_parse_state_t *state)
{
    state->frame_len = 0U;
}

/**
 * @brief 遇到可结束 ASCII 命令的空白字符时返回 1。
 */
static uint8_t task_uart_is_terminator(uint8_t ch)
{
    return ((ch == '\r') || (ch == '\n') || (ch == ' ') || (ch == '\t')) ? 1U : 0U;
}

/**
 * @brief 解析器当前只看到可选前缀 't' 时返回 1。
 */
static uint8_t task_uart_is_prefix_wait(const task_uart_parse_state_t *state)
{
    return ((state->frame_len == 1U) &&
        ((state->frame_buf[0] == 't') || (state->frame_buf[0] == 'T'))) ? 1U : 0U;
}

/**
 * @brief 将两个 ASCII 数字转换为任务编号。
 */
static task_id_t task_uart_finish_decimal(uint8_t tens, uint8_t ones)
{
    uint8_t value = (uint8_t)(((tens - '0') * 10U) + (ones - '0'));

    return task_uart_command_from_number(value);
}

/**
 * @brief 将十进制命令号转换为任务编号。
 */
task_id_t task_uart_command_from_number(uint8_t number)
{
    return task_uart_lookup_command(g_task_uart_number_map,
        (uint32_t)(sizeof(g_task_uart_number_map) / sizeof(g_task_uart_number_map[0])),
        number);
}

/**
 * @brief 将原始二进制命令字节转换为任务编号。
 */
task_id_t task_uart_command_from_hex_byte(uint8_t value)
{
    return task_uart_lookup_command(g_task_uart_binary_map,
        (uint32_t)(sizeof(g_task_uart_binary_map) / sizeof(g_task_uart_binary_map[0])),
        value);
}

/**
 * @brief 解析不可打印 UART 字节，包括二进制任务命令。
 */
static task_id_t task_uart_parse_control_byte(task_uart_parse_state_t *state,
    uint8_t ch)
{
    task_id_t command;

    command = task_uart_command_from_hex_byte(ch);
    if (command != TASK_ID_NONE) {
        task_uart_parse_reset(state);
        return command;
    }

    if ((ch >= 0x01U) && (ch <= 0x1FU)) {
        task_uart_parse_reset(state);
    }

    return TASK_ID_NONE;
}

/**
 * @brief 在 ASCII 任务命令解析器中消费一个十进制数字。
 */
static task_id_t task_uart_parse_decimal_digit(task_uart_parse_state_t *state,
    uint8_t ch)
{
    if (state->frame_len == 0U) {
        state->frame_buf[0] = ch;
        state->frame_len = 1U;
        return TASK_ID_NONE;
    }

    if (task_uart_is_prefix_wait(state) != 0U) {
        state->frame_buf[1] = ch;
        state->frame_len = 2U;
        return TASK_ID_NONE;
    }

    if (state->frame_len == 1U) {
        task_id_t command = task_uart_finish_decimal(state->frame_buf[0], ch);
        task_uart_parse_reset(state);
        return command;
    }

    if ((state->frame_len == 2U) &&
        ((state->frame_buf[0] == 't') || (state->frame_buf[0] == 'T'))) {
        task_id_t command = task_uart_finish_decimal(state->frame_buf[1], ch);
        task_uart_parse_reset(state);
        return command;
    }

    state->frame_buf[0] = ch;
    state->frame_len = 1U;
    return TASK_ID_NONE;
}

/**
 * @brief 解析一个 UART 字节，如形成完整命令则返回该命令。
 */
static task_id_t task_uart_parse_byte(task_uart_parse_state_t *state,
    uint8_t ch,
    uint8_t allow_binary_stop)
{
    if (ch == 0x00U) {
        task_uart_parse_reset(state);
        return (allow_binary_stop != 0U) ? TASK_ID_STOP : TASK_ID_NONE;
    }

    if (task_uart_is_terminator(ch) != 0U) {
        if (task_uart_is_prefix_wait(state) != 0U) {
            return TASK_ID_NONE;
        }
        task_uart_parse_reset(state);
        return TASK_ID_NONE;
    }

    if ((ch < '0') || (ch > '9')) {
        if ((ch == 't') || (ch == 'T')) {
            state->frame_buf[0] = ch;
            state->frame_len = 1U;
            return TASK_ID_NONE;
        }

        return task_uart_parse_control_byte(state, ch);
    }

    return task_uart_parse_decimal_digit(state, ch);
}

/**
 * @brief 轮询 UART0 直到 FIFO 为空，并返回遇到的第一个完整命令。
 */
task_id_t task_uart_read_command(uint8_t allow_binary_stop)
{
    static task_uart_parse_state_t parse_state = {{0U, 0U, 0U}, 0U};

    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);
        task_id_t command = task_uart_parse_byte(&parse_state, ch, allow_binary_stop);

        if (command != TASK_ID_NONE) {
            return command;
        }
    }

    return TASK_ID_NONE;
}

/**
 * @brief 读取一个低电平有效按键引脚。
 */
static uint8_t task_button_pin_is_pressed(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) == 0U) ? 1U : 0U;
}

/**
 * @brief 将四个实体任务按键转换为任务编号。
 */
static task_id_t task_button_read(void)
{
    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY1_PIN) != 0U) {
        return TASK_ID_1;
    }

    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY2_PIN) != 0U) {
        return TASK_ID_2;
    }

    if (task_button_pin_is_pressed(KEYS_B_PORT, KEYS_B_KEY3_PIN) != 0U) {
        return TASK_ID_3;
    }

    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY4_PIN) != 0U) {
        return TASK_ID_4;
    }

    return TASK_ID_NONE;
}

/**
 * @brief 阻塞等待 UART 命令或完成防抖的按键选择任务。
 */
task_id_t wait_task_uart_command(void)
{
    task_id_t task_id;

    while (1) {
        task_id = task_uart_read_command(0U);
        if (task_id != TASK_ID_NONE) {
            if (task_id == TASK_ID_STOP) {
                TB6612_Brake();
                lc_printf("TASK UART command accepted: id=0 stop\r\n");
            } else {
                lc_printf("TASK UART command accepted: id=%u\r\n", task_id);
            }
            return task_id;
        }

        task_id = task_button_read();
        if (task_id != TASK_ID_NONE) {
            delay_ms_with_st011(TASK_BUTTON_DEBOUNCE_MS);
            if (task_button_read() == task_id) {
                while (task_button_read() == task_id) {
                    (void)task_uart_read_command(0U);
                    delay_ms_with_st011(TASK_BUTTON_IDLE_MS);
                }
                delay_ms_with_st011(TASK_BUTTON_DEBOUNCE_MS);
                return task_id;
            }
        }

        TB6612_Brake();
        delay_ms_with_st011(TASK_BUTTON_IDLE_MS);
    }
}
