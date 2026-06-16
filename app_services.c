#include "app_services.h"

#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_task_ids.h"
#include "board.h"

/* Remaining duration for the currently active ST011 pulse. */
static uint32_t g_st011_pulse_remaining_ms;

/**
 * @brief Drive the ST011 trigger pin according to the configured polarity.
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
 * @brief Decrease pending pulse time and switch the ST011 output off at zero.
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
 * @brief Delay in chunks so a pending ST011 pulse can finish at the right time.
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
 * @brief Begin a non-blocking sound/light pulse.
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
 * @brief Emit one blocking sound/light pulse.
 */
void st011_pulse(uint32_t pulse_ms)
{
    st011_start_pulse(pulse_ms);
    delay_ms_with_st011(pulse_ms);
}

/**
 * @brief Block until the current non-blocking sound/light pulse has completed.
 */
void st011_finish_pending_pulse(void)
{
    if (g_st011_pulse_remaining_ms != 0U) {
        delay_ms_with_st011(g_st011_pulse_remaining_ms);
    }
}

/**
 * @brief Check UART0 for an in-run stop command without consuming task starts.
 */
uint8_t task_uart_stop_requested(void)
{
    return (task_uart_read_command(1U) == TASK_ID_STOP) ? 1U : 0U;
}
