#include "app_task_ids.h"

#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_services.h"
#include "board.h"
#include "bsp_tb6612.h"

task_id_t task_uart_command_from_number(uint8_t number)
{
    switch (number) {
    case 0:
        return TASK_ID_STOP;
    case 1:
        return TASK_ID_1;
    case 2:
        return TASK_ID_2;
    case 3:
        return TASK_ID_3;
    case 4:
        return TASK_ID_4;
    case 5:
        return TASK_ID_5;
    case 6:
        return TASK_ID_6;
    case 7:
        return TASK_ID_7;
    case 10:
        return TASK_ID_10;
    case 11:
        return TASK_ID_11;
    default:
        return TASK_ID_NONE;
    }
}

task_id_t task_uart_command_from_hex_byte(uint8_t value)
{
    switch (value) {
    case 0x01U:
        return TASK_ID_1;
    case 0x02U:
        return TASK_ID_2;
    case 0x03U:
        return TASK_ID_3;
    case 0x04U:
        return TASK_ID_4;
    case 0x05U:
        return TASK_ID_5;
    case 0x06U:
        return TASK_ID_6;
    case 0x07U:
        return TASK_ID_7;
    case 0x10U:
        return TASK_ID_10;
    case 0x11U:
        return TASK_ID_11;
    default:
        return TASK_ID_NONE;
    }
}

task_id_t task_uart_read_command(uint8_t allow_binary_stop)
{
    static uint8_t frame_buf[3] = {0U};
    static uint8_t frame_len = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);

        if (ch == 0x00U) {
            frame_len = 0U;
            if (allow_binary_stop != 0U) {
                return TASK_ID_STOP;
            }
            continue;
        }

        if ((ch == '\r') || (ch == '\n') || (ch == ' ') || (ch == '\t')) {
            if ((frame_len == 1U) &&
                ((frame_buf[0] == 't') || (frame_buf[0] == 'T'))) {
                continue;
            }
            frame_len = 0U;
            continue;
        }

        {
            task_id_t command = task_uart_command_from_hex_byte(ch);
            if (command != TASK_ID_NONE) {
                frame_len = 0U;
                return command;
            }
        }

        if ((ch >= 0x01U) && (ch <= 0x1FU)) {
            frame_len = 0U;
            continue;
        }

        if ((ch == 't') || (ch == 'T')) {
            frame_buf[0] = ch;
            frame_len = 1U;
            continue;
        }

        if ((ch >= '0') && (ch <= '9')) {
            if (frame_len == 0U) {
                frame_buf[0] = ch;
                frame_len = 1U;
                continue;
            }

            if ((frame_len == 1U) &&
                ((frame_buf[0] == 't') || (frame_buf[0] == 'T'))) {
                frame_buf[1] = ch;
                frame_len = 2U;
                continue;
            }

            if (frame_len == 1U) {
                task_id_t command;
                uint8_t value = (uint8_t)(((frame_buf[0] - '0') * 10U) +
                    (ch - '0'));

                command = task_uart_command_from_number(value);
                frame_len = 0U;
                if (command != TASK_ID_NONE) {
                    return command;
                }
                continue;
            }

            if ((frame_len == 2U) &&
                ((frame_buf[0] == 't') || (frame_buf[0] == 'T'))) {
                task_id_t command;
                uint8_t value = (uint8_t)(((frame_buf[1] - '0') * 10U) +
                    (ch - '0'));

                command = task_uart_command_from_number(value);
                frame_len = 0U;
                if (command != TASK_ID_NONE) {
                    return command;
                }
                continue;
            }

            frame_buf[0] = ch;
            frame_len = 1U;
            continue;
        }

        frame_len = 0U;
    }

    return TASK_ID_NONE;
}

static uint8_t task_button_pin_is_pressed(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) == 0U) ? 1U : 0U;
}

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
