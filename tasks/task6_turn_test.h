#ifndef TASK6_TURN_TEST_H
#define TASK6_TURN_TEST_H

/**
 * @file task6_turn_test.h
 * @brief UART 06 fast C-point turn test sequence.
 *
 * Task6 reuses the heading straight segment to drive A->C, then enables the
 * C-point fast turn hook inside heading_straight.h when the line is detected.
 */

#include <stdint.h>

#include "app_config.h"
#include "app_services.h"
#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "heading/heading_straight.h"

/**
 * @brief Run only the AC approach and trigger the C-point fast-turn hook.
 *
 * This is a tuning aid for choosing the immediate C-point turn behavior. The
 * hook request is cleared on every exit path so later tasks do not inherit it.
 */
static void run_task6_ac_c_turn_test(void)
{
    uint8_t reason;

    lc_printf("TASK6 start: UART 06, run task3 AC only, then fast left turn at C target=%d pwm=%d/%d\r\n",
        TASK6_C_TURN_TARGET_CDEG,
        TASK6_C_TURN_B_PWM,
        TASK6_C_TURN_A_PWM);

    g_task6_c_turn_requested = 1U;
    {
        const heading_straight_segment_config_t config = {
            .tag = "TASK6_AC",
            .zero_heading = 0U,
            .heading_target_cdeg = TASK3_AC_HEADING_TARGET_CDEG,
            .heading_only = 1U,
            .fast_correction = 0U,
            .line_search_protect = 2U,
            .start_alarm_ms = TASK1_START_ALARM_MS,
            .stop_alarm_ms = 0U
        };

        reason = run_heading_straight_to_line_segment(&config);
    }
    g_task6_c_turn_requested = 0U;

    if (reason != 1U) {
        TB6612_Brake();
        encoder_reset_distance_counts();
        lc_printf("TASK6 abort after AC: stop_reason=%u\r\n", reason);
        return;
    }

    TB6612_Brake();
    encoder_reset_distance_counts();
    lc_printf("TASK6 complete: AC line detected, C fast turn test finished\r\n");
}

#endif /* TASK6_TURN_TEST_H */
