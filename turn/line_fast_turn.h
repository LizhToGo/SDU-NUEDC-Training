#ifndef LINE_FAST_TURN_H
#define LINE_FAST_TURN_H

/**
 * @file line_fast_turn.h
 * @brief Fast line-triggered turn helper.
 *
 * Used by the C/D point fast-turn tests and handoff logic after a straight
 * segment reaches a line. The implementation remains header-only for the
 * current CCS generated build layout.
 */

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_tb6612.h"

/**
 * @brief Configuration for the quick C/D line handoff turn.
 */
typedef struct {
    const char *tag;
    uint32_t ac_elapsed_ms;
    int32_t ac_distance_count;
    const ir_tracking_sample_t *line_sample;
    int32_t line_yaw_cdeg;
    int32_t target_cdeg;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int32_t handoff_turn_dir;
    uint8_t target_is_absolute;
    uint8_t enable_line_stop;
    uint8_t brake_after_turn;
    uint8_t report_samples;
} line_fast_turn_config_t;

/**
 * @brief Check whether yaw progress has reached the configured turn target.
 */
static uint8_t line_fast_turn_target_reached(int32_t turn_target_cdeg,
    int32_t turn_cdeg)
{
    return (((turn_target_cdeg >= 0) && (turn_cdeg >= turn_target_cdeg)) ||
        ((turn_target_cdeg < 0) && (turn_cdeg <= turn_target_cdeg))) ? 1U : 0U;
}

/**
 * @brief Check whether line-stop criteria are satisfied during a fast turn.
 */
static uint8_t line_fast_turn_line_stop_ready(
    const line_fast_turn_config_t *config,
    int32_t turn_abs,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    return ((config->enable_line_stop != 0U) &&
        (turn_abs >= TASK6_C_TURN_LINE_ARM_CDEG) &&
        (ir_ok != 0U) &&
        (sample->line_lost == 0U) &&
        ((sample->line_mask & TASK6_C_TURN_LINE_STOP_MASK) != 0U) &&
        (sample->active_count >= TASK6_C_TURN_LINE_STOP_MIN_COUNT)) ? 1U : 0U;
}

/**
 * @brief Decide whether the turn routine should brake when it finishes.
 */
static uint8_t line_fast_turn_brake_required(
    const line_fast_turn_config_t *config,
    uint8_t stop_reason)
{
    return ((config->brake_after_turn != 0U) ||
        ((stop_reason != 1U) && (stop_reason != 5U))) ? 1U : 0U;
}

/**
 * @brief Apply the handoff PWM command used when finishing without braking.
 */
static void line_fast_turn_apply_finish_drive(
    const line_fast_turn_config_t *config,
    uint8_t stop_reason)
{
    int32_t handoff_turn;
    int32_t handoff_b_pwm;
    int32_t handoff_a_pwm;

    if (line_fast_turn_brake_required(config, stop_reason) != 0U) {
        TB6612_Brake();
    } else if (config->handoff_turn_dir == 0) {
        TB6612_SetDifferential(TASK1_RAMP_B_START_PWM, TASK1_RAMP_A_START_PWM);
    } else {
        handoff_turn = config->handoff_turn_dir * TASK3_ARC_ENTRY_TURN;
        handoff_b_pwm = clamp_i32((TASK3_ARC_B_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP) + handoff_turn,
            TASK3_ARC_MIN_PWM,
            TASK3_ARC_MAX_PWM);
        handoff_a_pwm = clamp_i32((TASK3_ARC_A_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP) - handoff_turn,
            TASK3_ARC_MIN_PWM,
            TASK3_ARC_MAX_PWM);
        TB6612_SetDifferential((int16_t)handoff_b_pwm, (int16_t)handoff_a_pwm);
    }
}

/**
 * @brief Execute a timed/yaw-measured fast turn with optional IR line stop.
 */
static uint8_t run_line_fast_turn(const line_fast_turn_config_t *config)
{
    const char *tag = config->tag;
    uint32_t ac_elapsed_ms = config->ac_elapsed_ms;
    int32_t ac_distance_count = config->ac_distance_count;
    const ir_tracking_sample_t *line_sample = config->line_sample;
    int32_t line_yaw_cdeg = config->line_yaw_cdeg;
    int32_t target_cdeg = config->target_cdeg;
    int16_t motor_b_pwm = config->motor_b_pwm;
    int16_t motor_a_pwm = config->motor_a_pwm;
    uint8_t target_is_absolute = config->target_is_absolute;
    uint8_t report_samples = config->report_samples;
    static uint32_t log_t[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_yaw[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_turn[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_gz[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_b_total[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_a_total[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_raw[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_mask[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_count[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_lost[TASK6_C_TURN_SAMPLE_MAX];
    jy62_navigation_t nav = {0};
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint8_t sample_index = 0U;
    uint8_t stop_reason = 0U;
    uint8_t nav_ok;
    uint8_t ir_ok;
    uint8_t line_stop_ready;
    int32_t start_yaw;
    int32_t yaw = 0;
    int32_t turn_cdeg = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t overshoot;
    int32_t turn_abs;
    int32_t turn_target_cdeg;
    uint8_t index;

    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok == 0U) {
        TB6612_Brake();
        lc_printf("%s abort: nav invalid before turn ac_t=%lu ac_dist=%ld\r\n",
            tag,
            ac_elapsed_ms,
            ac_distance_count);
        return 0U;
    }

    start_yaw = nav.yaw_relative_cdeg;
    turn_target_cdeg = (target_is_absolute != 0U) ?
        normalize_cdeg(target_cdeg - start_yaw) : target_cdeg;
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential(motor_b_pwm, motor_a_pwm);

    while (elapsed_ms < TASK6_C_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok == 0U) {
            stop_reason = 4U;
            break;
        }

        yaw = nav.yaw_relative_cdeg;
        turn_cdeg = normalize_cdeg(yaw - start_yaw);
        turn_abs = abs_i32(turn_cdeg);
        ir_ok = IRTracking_ReadSample(&sample);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        line_stop_ready = line_fast_turn_line_stop_ready(config,
            turn_abs,
            ir_ok,
            &sample);

        if ((report_elapsed_ms >= TASK6_C_TURN_REPORT_PERIOD_MS) &&
            (sample_index < TASK6_C_TURN_SAMPLE_MAX)) {
            report_elapsed_ms = 0;
            log_t[sample_index] = elapsed_ms;
            log_yaw[sample_index] = yaw;
            log_turn[sample_index] = turn_cdeg;
            log_gz[sample_index] = nav.gyro_z_filtered_mdps;
            log_b_total[sample_index] = motor_b_total;
            log_a_total[sample_index] = motor_a_total;
            log_raw[sample_index] = (ir_ok != 0U) ? sample.raw : 0xFFU;
            log_mask[sample_index] = (ir_ok != 0U) ? sample.line_mask : 0U;
            log_count[sample_index] = (ir_ok != 0U) ? sample.active_count : 0U;
            log_lost[sample_index] = (ir_ok != 0U) ? sample.line_lost : 1U;
            sample_index++;
        }

        if (line_fast_turn_target_reached(turn_target_cdeg, turn_cdeg) != 0U) {
            stop_reason = 1U;
            break;
        }

        if (line_stop_ready != 0U) {
            stop_reason = 5U;
            break;
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok != 0U) {
        yaw = nav.yaw_relative_cdeg;
        turn_cdeg = normalize_cdeg(yaw - start_yaw);
    }
    ir_ok = IRTracking_ReadSample(&sample);
    overshoot = (turn_target_cdeg >= 0) ?
        (turn_cdeg - turn_target_cdeg) : (turn_target_cdeg - turn_cdeg);

    line_fast_turn_apply_finish_drive(config, stop_reason);

    if (report_samples != 0U) {
        lc_printf("%s start: ac_t=%lu ac_dist=%ld line_yaw=%ld line_raw=0x%02X line_mask=0x%02X line_cnt=%u line_err=%ld target=%d turn_target=%ld pwm=%d/%d yaw0=%ld\r\n",
            tag,
            ac_elapsed_ms,
            ac_distance_count,
            line_yaw_cdeg,
            (line_sample != 0) ? line_sample->raw : 0xFFU,
            (line_sample != 0) ? line_sample->line_mask : 0U,
            (line_sample != 0) ? line_sample->active_count : 0U,
            (line_sample != 0) ? line_sample->error : 0,
            target_cdeg,
            turn_target_cdeg,
            motor_b_pwm,
            motor_a_pwm,
            start_yaw);

        for (index = 0U; index < sample_index; index++) {
            lc_printf("%s sample n=%u t=%lu yaw=%ld turn=%ld gzlp=%ld B=%ld A=%ld ir=0x%02X/0x%02X/%u lost=%u\r\n",
                tag,
                index,
                log_t[index],
                log_yaw[index],
                log_turn[index],
                log_gz[index],
                log_b_total[index],
                log_a_total[index],
                log_raw[index],
                log_mask[index],
                log_count[index],
                log_lost[index]);
        }
    }

    lc_printf("%s stop: reason=%s t=%lu yaw=%ld turn=%ld target=%d turn_target=%ld overshoot=%ld nav=%u brake=%u B_total=%ld A_total=%ld ir=0x%02X/0x%02X/%u lost=%u err=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "target" :
            ((stop_reason == 2U) ? "timeout" :
            ((stop_reason == 3U) ? "uart_stop" :
            ((stop_reason == 5U) ? "line" : "nav_invalid"))),
        elapsed_ms,
        yaw,
        turn_cdeg,
        target_cdeg,
        turn_target_cdeg,
        overshoot,
        nav_ok,
        line_fast_turn_brake_required(config, stop_reason),
        motor_b_total,
        motor_a_total,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0);

    encoder_reset_distance_counts();
    return ((stop_reason == 1U) || (stop_reason == 5U)) ? 1U : 0U;
}

#endif /* LINE_FAST_TURN_H */
