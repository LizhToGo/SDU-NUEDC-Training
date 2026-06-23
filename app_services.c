#include "app_services.h"

#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_task_ids.h"
#include "board.h"

/* 当前 ST011 非阻塞脉冲还需要保持的时间。 */
static uint32_t g_st011_pulse_remaining_ms;

/**
 * @brief 按配置的触发极性驱动 ST011 引脚。
 */
void st011_set_active(uint8_t active)
{
#if ST011_ACTIVE_LOW
    if (active != 0U) {
        DL_GPIO_clearPins(ST011_PORT, ST011_TRIG_PIN);
    } else {
        DL_GPIO_setPins(ST011_PORT, ST011_TRIG_PIN);
    }
#else
    if (active != 0U) {
        DL_GPIO_setPins(ST011_PORT, ST011_TRIG_PIN);
    } else {
        DL_GPIO_clearPins(ST011_PORT, ST011_TRIG_PIN);
    }
#endif
}

/**
 * @brief 扣减待完成脉冲时间，归零时关闭 ST011 输出。
 */
void st011_service(uint32_t elapsed_ms)
{
    if (g_st011_pulse_remaining_ms == 0U) {
        return;
    }

    if (elapsed_ms >= g_st011_pulse_remaining_ms) {
        g_st011_pulse_remaining_ms = 0U;
        st011_set_active(0U);
    } else {
        g_st011_pulse_remaining_ms -= elapsed_ms;
    }
}

/**
 * @brief 分片延时，确保等待期间 ST011 非阻塞脉冲能准时结束。
 */
void delay_ms_with_st011(uint32_t total_ms)
{
    while (total_ms > 0U) {
        uint32_t step_ms = total_ms;

        if ((g_st011_pulse_remaining_ms != 0U) &&
            (step_ms > g_st011_pulse_remaining_ms)) {
            step_ms = g_st011_pulse_remaining_ms;
        }

        delay_ms((int)step_ms);
        st011_service(step_ms);
        total_ms -= step_ms;
    }
}

/**
 * @brief 启动一次非阻塞声光脉冲。
 */
void st011_start_pulse(uint32_t pulse_ms)
{
    if (pulse_ms == 0U) {
        return;
    }

    g_st011_pulse_remaining_ms = pulse_ms;
    st011_set_active(1U);
}

/**
 * @brief 启动一次阻塞声光脉冲，并等待其结束。
 */
void st011_pulse(uint32_t pulse_ms)
{
    st011_start_pulse(pulse_ms);
    delay_ms_with_st011(pulse_ms);
}

/**
 * @brief 等待当前非阻塞声光脉冲结束。
 */
void st011_finish_pending_pulse(void)
{
    if (g_st011_pulse_remaining_ms != 0U) {
        delay_ms_with_st011(g_st011_pulse_remaining_ms);
    }
}

/**
 * @brief 检查 UART0 是否收到运行中停车命令，不消费新的任务启动命令。
 */
uint8_t task_uart_stop_requested(void)
{
    return (task_uart_read_command(1U) == TASK_ID_STOP) ? 1U : 0U;
}
