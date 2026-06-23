#ifndef TASK8_EXIT_TURN_CALIBRATION_H
#define TASK8_EXIT_TURN_CALIBRATION_H

/**
 * @file task8_exit_turn_calibration.h
 * @brief UART 08 repeated Task4-style exit-turn calibration.
 */

#include <stdint.h>

#include "app_config.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "app_task_ids.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_tb6612.h"

/**
 * @brief Return the most recent gyro-turn stop event from the race RAM log.
 */
static const race_event_log_t *task8_last_turn_stop_event(void)
{
#if RACE_RAM_LOG_ENABLE
    uint16_t i = g_race_event_log_count;

    while (i > 0U) {
        i--;
        if (g_race_event_log[i].event == RACE_RAM_EVENT_TURN_STOP) {
            return &g_race_event_log[i];
        }
    }
#endif

    return 0;
}

/**
 * @brief Wait between turns while still honoring UART stop.
 */
static uint8_t task8_wait_or_stop(uint32_t wait_ms)
{
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < wait_ms) {
        uint32_t step_ms = wait_ms - elapsed_ms;

        if (step_ms > RACE_TASK4_CONTROL_PERIOD_MS) {
            step_ms = RACE_TASK4_CONTROL_PERIOD_MS;
        }
        delay_ms_with_st011(step_ms);
        elapsed_ms += step_ms;
        if (task_uart_stop_requested() != 0U) {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief Run one relative 36.8-degree exit-turn using Task4 gyro-turn settings.
 */
static uint8_t task8_run_one_exit_turn(uint8_t pair_index,
    uint8_t phase,
    int32_t turn_dir,
    uint32_t total_elapsed_ms)
{
    const char *tag = (turn_dir < 0) ? "TASK8_RIGHT_36P8" : "TASK8_LEFT_36P8";
    const char *dir_name = (turn_dir < 0) ? "right" : "left";
    int32_t yaw0_cdeg = 0;
    int32_t gz0_mdps = 0;
    int32_t target_cdeg;
    uint8_t nav_ok;
    uint8_t ok;
    jy62_navigation_t nav = {0};
    const race_event_log_t *stop_log;
    const gyro_turn_config_t turn_config = {
        .tag = tag,
        .motor_b_pwm = (turn_dir < 0) ?
            RACE_EXIT_RIGHT_TURN_B_PWM : RACE_EXIT_LEFT_TURN_B_PWM,
        .motor_a_pwm = (turn_dir < 0) ?
            RACE_EXIT_RIGHT_TURN_A_PWM : RACE_EXIT_LEFT_TURN_A_PWM,
        .slow_motor_b_pwm = (turn_dir < 0) ?
            RACE_EXIT_RIGHT_TURN_SLOW_B_PWM : RACE_EXIT_LEFT_TURN_SLOW_B_PWM,
        .slow_motor_a_pwm = (turn_dir < 0) ?
            RACE_EXIT_RIGHT_TURN_SLOW_A_PWM : RACE_EXIT_LEFT_TURN_SLOW_A_PWM,
        .yaw_stop_target_cdeg = 0,
        .predictive_stop_enable = RACE_TASK4_EXIT_TURN_PREDICT_ENABLE,
        .predictive_stop_ms = RACE_TASK4_EXIT_TURN_PREDICT_MS,
        .predictive_stop_min_gz_mdps =
            RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS,
        .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
    };
    gyro_turn_config_t active_config = turn_config;

    nav_ok = race_peek_yaw(&yaw0_cdeg, &gz0_mdps);
    if (nav_ok == 0U) {
        lc_printf("TASK8_TURN pair=%u ph=%u dir=%s abort=nav0\r\n",
            (unsigned int)pair_index,
            (unsigned int)phase,
            dir_name);
        return 0U;
    }

    target_cdeg = normalize_cdeg(yaw0_cdeg +
        (turn_dir * TASK8_EXIT_TURN_TEST_ANGLE_CDEG));
    active_config.yaw_stop_target_cdeg = target_cdeg;

    race_ram_log_set_context(pair_index, phase);
    race_post_point_context_begin(total_elapsed_ms, 0);
    lc_printf("TASK8_TURN_BEGIN pair=%u ph=%u dir=%s yaw0=%ld target=%ld angle=%d gzlp=%ld pwm=%d/%d slow=%d/%d pred=%u/%d arm=%d min_gz=%d\r\n",
        (unsigned int)pair_index,
        (unsigned int)phase,
        dir_name,
        (long)yaw0_cdeg,
        (long)target_cdeg,
        TASK8_EXIT_TURN_TEST_ANGLE_CDEG,
        (long)gz0_mdps,
        active_config.motor_b_pwm,
        active_config.motor_a_pwm,
        active_config.slow_motor_b_pwm,
        active_config.slow_motor_a_pwm,
        active_config.predictive_stop_enable,
        active_config.predictive_stop_ms,
        RACE_TASK4_EXIT_TURN_PREDICT_ARM_CDEG,
        active_config.predictive_stop_min_gz_mdps);

    ok = race_gyro_turn_to_yaw(&active_config);
    nav_ok = JY62_PeekNavigation(&nav);
    stop_log = task8_last_turn_stop_event();
    lc_printf("TASK8_TURN_END pair=%u ph=%u dir=%s ok=%u yaw0=%ld target=%ld yaw1=%ld err=%ld actual=%ld t=%lu dist=%u fl=0x%02X gz100=%d B=%d A=%d nav=%u\r\n",
        (unsigned int)pair_index,
        (unsigned int)phase,
        dir_name,
        (unsigned int)ok,
        (long)yaw0_cdeg,
        (long)target_cdeg,
        (nav_ok != 0U) ? (long)nav.yaw_relative_cdeg : 0L,
        (nav_ok != 0U) ? (long)normalize_cdeg(nav.yaw_relative_cdeg -
            target_cdeg) : 0L,
        (nav_ok != 0U) ? (long)normalize_cdeg(nav.yaw_relative_cdeg -
            yaw0_cdeg) : 0L,
        (stop_log != 0) ? (unsigned long)stop_log->t_ms : 0UL,
        (stop_log != 0) ? (unsigned int)stop_log->dist_count : 0U,
        (stop_log != 0) ? (unsigned int)stop_log->flags : 0U,
        (stop_log != 0) ? (int)stop_log->gzlp_x100_mdps : 0,
        (stop_log != 0) ? (int)stop_log->motor_b_total : 0,
        (stop_log != 0) ? (int)stop_log->motor_a_total : 0,
        (unsigned int)nav_ok);

    return ok;
}

/**
 * @brief UART 08: alternate right/left 36.8-degree Task4-style exit turns.
 */
static void run_task8_exit_turn_calibration(void)
{
    uint8_t pair_count = TASK8_EXIT_TURN_TEST_PAIR_COUNT;
    uint8_t pair_index;
    uint32_t total_elapsed_ms = 0U;
    uint8_t final_reason = 1U;
    jy62_navigation_t nav = {0};
    ir_tracking_sample_t sample = {0};
    uint8_t nav_ok;
    uint8_t ir_ok;

    if (pair_count > RACE_RAM_LOG_MAX_LAPS) {
        pair_count = RACE_RAM_LOG_MAX_LAPS;
    }

    TB6612_Brake();
    encoder_reset_distance_counts();
    race_ram_log_reset();
    lc_printf("TASK8 start: UART 08, task4 exit-turn calibration angle=%d pair=%u wait=%u control=%u pred=%u ms=%d arm=%d min_gz=%d tol=%d right=%d/%d slow=%d/%d left=%d/%d slow=%d/%d\r\n",
        TASK8_EXIT_TURN_TEST_ANGLE_CDEG,
        (unsigned int)pair_count,
        (unsigned int)TASK8_EXIT_TURN_TEST_WAIT_MS,
        (unsigned int)RACE_TASK4_CONTROL_PERIOD_MS,
        RACE_TASK4_EXIT_TURN_PREDICT_ENABLE,
        RACE_TASK4_EXIT_TURN_PREDICT_MS,
        RACE_TASK4_EXIT_TURN_PREDICT_ARM_CDEG,
        RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS,
        RACE_TURN_YAW_STOP_TOL_CDEG,
        RACE_EXIT_RIGHT_TURN_B_PWM,
        RACE_EXIT_RIGHT_TURN_A_PWM,
        RACE_EXIT_RIGHT_TURN_SLOW_B_PWM,
        RACE_EXIT_RIGHT_TURN_SLOW_A_PWM,
        RACE_EXIT_LEFT_TURN_B_PWM,
        RACE_EXIT_LEFT_TURN_A_PWM,
        RACE_EXIT_LEFT_TURN_SLOW_B_PWM,
        RACE_EXIT_LEFT_TURN_SLOW_A_PWM);

    for (pair_index = 0U; pair_index < pair_count; pair_index++) {
        if (task_uart_stop_requested() != 0U) {
            final_reason = 3U;
            break;
        }
        if (task8_run_one_exit_turn(pair_index, 0U, -1, total_elapsed_ms) == 0U) {
            final_reason = 2U;
            break;
        }
        {
            const race_event_log_t *stop_log = task8_last_turn_stop_event();
            total_elapsed_ms = (stop_log != 0) ? stop_log->t_ms : total_elapsed_ms;
        }
        if (task8_wait_or_stop(TASK8_EXIT_TURN_TEST_WAIT_MS) == 0U) {
            final_reason = 3U;
            break;
        }
        total_elapsed_ms += TASK8_EXIT_TURN_TEST_WAIT_MS;

        if (task8_run_one_exit_turn(pair_index, 1U, 1, total_elapsed_ms) == 0U) {
            final_reason = 2U;
            break;
        }
        {
            const race_event_log_t *stop_log = task8_last_turn_stop_event();
            total_elapsed_ms = (stop_log != 0) ? stop_log->t_ms : total_elapsed_ms;
        }
        if ((uint8_t)(pair_index + 1U) < pair_count) {
            if (task8_wait_or_stop(TASK8_EXIT_TURN_TEST_WAIT_MS) == 0U) {
                final_reason = 3U;
                break;
            }
            total_elapsed_ms += TASK8_EXIT_TURN_TEST_WAIT_MS;
        }
    }

    TB6612_Brake();
    nav_ok = JY62_PeekNavigation(&nav);
    ir_ok = IRTracking_ReadSample(&sample);
    race_ram_log_event(RACE_RAM_EVENT_COMPLETE,
        final_reason,
        0U,
        0U,
        0U,
        total_elapsed_ms,
        0,
        0,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        0,
        0,
        0,
        0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        0,
        0);
    lc_printf("TASK8 complete: reason=%u elapsed=%lu yaw=%ld gzlp=%ld ram_dump=1\r\n",
        (unsigned int)final_reason,
        (unsigned long)total_elapsed_ms,
        (nav_ok != 0U) ? (long)nav.yaw_relative_cdeg : 0L,
        (nav_ok != 0U) ? (long)nav.gyro_z_filtered_mdps : 0L);
    race_ram_log_dump(TASK4_LAP_COUNT);
}

#endif /* TASK8_EXIT_TURN_CALIBRATION_H */
