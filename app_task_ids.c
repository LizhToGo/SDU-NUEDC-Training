#include "app_task_ids.h"

#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"

/*
 * UART0 accepts two command styles:
 * - binary bytes: 0x01..0x04
 * - ASCII decimal pairs: "01".."04", optionally prefixed with 't'
 * Byte 0x00 is treated as STOP only when allow_binary_stop is enabled.
 */
typedef struct {
    uint8_t value;
    task_id_t task_id;
} task_uart_command_map_t;

/**
 * @brief Small parser buffer for ASCII commands such as "03" and "t10".
 */
typedef struct {
    uint8_t frame_buf[3];
    uint8_t frame_len;
} task_uart_parse_state_t;

/* Decimal command table shared by ASCII input paths. */
static const task_uart_command_map_t g_task_uart_number_map[] = {
    {0U, TASK_ID_STOP},
    {1U, TASK_ID_1},
    {2U, TASK_ID_2},
    {3U, TASK_ID_3},
    {4U, TASK_ID_4},
};

/* Binary command table for raw UART bytes. */
static const task_uart_command_map_t g_task_uart_binary_map[] = {
    {0x01U, TASK_ID_1},
    {0x02U, TASK_ID_2},
    {0x03U, TASK_ID_3},
    {0x04U, TASK_ID_4},
};

/**
 * @brief Find a task id in one of the static UART command tables.
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
 * @brief Reset the incremental ASCII parser state.
 */
static void task_uart_parse_reset(task_uart_parse_state_t *state)
{
    state->frame_len = 0U;
}

/**
 * @brief Return 1 for whitespace characters that terminate an ASCII command.
 */
static uint8_t task_uart_is_terminator(uint8_t ch)
{
    return ((ch == '\r') || (ch == '\n') || (ch == ' ') || (ch == '\t')) ? 1U : 0U;
}

/**
 * @brief Return 1 while the parser has only seen the optional 't' prefix.
 */
static uint8_t task_uart_is_prefix_wait(const task_uart_parse_state_t *state)
{
    return ((state->frame_len == 1U) &&
        ((state->frame_buf[0] == 't') || (state->frame_buf[0] == 'T'))) ? 1U : 0U;
}

/**
 * @brief Convert two ASCII digits to a task id.
 */
static task_id_t task_uart_finish_decimal(uint8_t tens, uint8_t ones)
{
    uint8_t value = (uint8_t)(((tens - '0') * 10U) + (ones - '0'));

    return task_uart_command_from_number(value);
}

/**
 * @brief Convert a decimal command number into a task id.
 */
task_id_t task_uart_command_from_number(uint8_t number)
{
    return task_uart_lookup_command(g_task_uart_number_map,
        (uint32_t)(sizeof(g_task_uart_number_map) / sizeof(g_task_uart_number_map[0])),
        number);
}

/**
 * @brief Convert a raw binary command byte into a task id.
 */
task_id_t task_uart_command_from_hex_byte(uint8_t value)
{
    return task_uart_lookup_command(g_task_uart_binary_map,
        (uint32_t)(sizeof(g_task_uart_binary_map) / sizeof(g_task_uart_binary_map[0])),
        value);
}

/**
 * @brief Parse non-printable UART bytes, including binary task commands.
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
 * @brief Consume one decimal digit in the ASCII task command parser.
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
 * @brief Parse one UART byte and return a completed command if available.
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
 * @brief Poll UART0 until its FIFO is empty and return the first full command.
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
 * @brief Read one active-low button pin.
 */
static uint8_t task_button_pin_is_pressed(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) == 0U) ? 1U : 0U;
}

/**
 * @brief Convert the four physical task buttons into task ids.
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
 * @brief Block until a UART command or debounced button press selects a task.
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
