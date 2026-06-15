#ifndef RACE_LAPS_H
#define RACE_LAPS_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_tb6612.h"

typedef struct {
    uint8_t reason;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    uint8_t nav_ok;
    ir_tracking_sample_t sample;
} line_result_t;

typedef struct {
    const char *phase_name;
    const char *point_name;
    const char *force_name;
    int32_t point_arm_count;
    int32_t force_count;
    int32_t phase_turn_dir;
    int32_t straight_target_cdeg;
    uint8_t arc_mode;
} race_phase_config_t;

typedef struct {
    line_result_t result;
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    straight_pid_t diff_pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms;
    uint32_t report_elapsed_ms;
    uint32_t nav_frame_delta;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t total_distance_count;
    int32_t phase_distance_count;
    int32_t phase_start_count;
    int32_t filtered_error;
    int32_t last_filtered_error;
    int32_t last_turn;
    int32_t yaw_start;
    int32_t yaw_cdeg;
    int32_t yaw_raw_cdeg;
    int32_t phase_yaw_cdeg;
    int32_t gyro_z_mdps;
    int32_t gyro_z_filtered_mdps;
    int32_t roll_cdeg;
    int32_t pitch_cdeg;
    int32_t yaw_progress_cdeg;
    int32_t raw_error;
    int32_t derivative;
    int32_t base_pwm;
    int32_t left_pwm;
    int32_t right_pwm;
    int32_t target_speed_diff;
    int32_t line_turn;
    int32_t nav_turn;
    int32_t control_turn;
    int32_t heading_error_cdeg;
    int32_t expected_yaw_cdeg;
    int32_t arc_actual_yaw_cdeg;
    uint8_t lap_count;
    uint8_t phase;
    uint8_t target_laps;
    uint8_t nav_ok;
    uint8_t nav_update_flags;
    uint8_t straight_point_count;
    uint16_t straight_line_seen_count;
    uint8_t ir_ok;
    uint8_t line_valid;
    uint8_t line_lost_seen;
    uint8_t straight_point_candidate;
    uint8_t edge_point_seen;
    uint8_t point_log_flags;
    uint8_t point_ready;
    uint8_t stop_reason;
} race_context_t;

static const char *race_reason_name(uint8_t reason)
{
    if (reason == 0U) {
        return "none";
    }
    if (reason == 1U) {
        return "point";
    }
    if (reason == 2U) {
        return "force";
    }
    if (reason == 3U) {
        return "uart_stop";
    }
    if (reason == 5U) {
        return "nav_invalid";
    }
    if (reason == 6U) {
        return "yaw";
    }
    return "timeout";
}

static const char *race_phase_name(uint8_t phase)
{
    if (phase == 0U) {
        return "AC";
    }
    if (phase == 1U) {
        return "CB";
    }
    if (phase == 2U) {
        return "BD";
    }
    return "DA";
}

#include "race_log.h"

static uint8_t race_peek_yaw(int32_t *yaw_cdeg, int32_t *gyro_z_filtered_mdps)
{
    jy62_navigation_t nav;
    uint8_t nav_ok = JY62_PeekNavigation(&nav);

    if (nav_ok != 0U) {
        *yaw_cdeg = nav.yaw_relative_cdeg;
        *gyro_z_filtered_mdps = nav.gyro_z_filtered_mdps;
    } else {
        *yaw_cdeg = 0;
        *gyro_z_filtered_mdps = 0;
    }

    return nav_ok;
}

static void race_print_point(const char *name, const line_result_t *result)
{
    race_log_printf("RACE_POINT %s reason=%s t=%lu dist=%ld yaw=%ld mask=0x%02X err=%ld\r\n",
        name,
        race_reason_name(result->reason),
        result->elapsed_ms,
        result->distance_count,
        result->yaw_cdeg,
        (result->ir_ok != 0U) ? result->sample.line_mask : 0U,
        (result->ir_ok != 0U) ? result->sample.error : 0);
}

static void race_diff_pid_reset(straight_pid_t *pid)
{
    straight_pid_reset(pid);
    pid->kp = TASK11_DIFF_KP;
    pid->ki = 0;
    pid->kd = TASK11_DIFF_KD;
    pid->i_limit = 0;
    pid->corr_max = TASK11_DIFF_CORR_MAX;
    pid->integral = 0;
    pid->last_error = 0;
}

static void race_drive_config(straight_drive_config_t *config,
    int32_t base_pwm,
    int32_t target_speed_diff)
{
    config->base_b_pwm = base_pwm;
    config->base_a_pwm = base_pwm;
    config->target_speed_diff = target_speed_diff;
    config->diff_ff_gain = TASK11_DIFF_FF_GAIN;
    config->distance_corr_divisor = 1;
    config->distance_corr_max = 0;
    config->correction_max = TASK11_DIFF_CORR_MAX;
    config->min_pwm = TASK11_LINE_MIN_PWM;
    config->max_pwm = TASK11_LINE_MAX_PWM;
}

static int32_t race_heading_turn_from_error(int32_t heading_error_cdeg,
    int32_t gyro_z_filtered_mdps,
    int32_t corr_divisor,
    int32_t gyro_damp_divisor,
    int32_t corr_max)
{
    int32_t correction;

    if (corr_divisor == 0) {
        corr_divisor = 1;
    }
    if (gyro_damp_divisor == 0) {
        gyro_damp_divisor = 1;
    }

    correction = (heading_error_cdeg * TASK1_HEADING_CORR_SIGN) / corr_divisor;
    correction -= gyro_z_filtered_mdps / gyro_damp_divisor;

    return clamp_i32(-correction, -corr_max, corr_max);
}

static int32_t race_arc_expected_yaw_cdeg(int32_t phase_distance_count,
    int32_t phase_turn_dir)
{
    int32_t progress_cdeg;

    if (phase_distance_count <= 0) {
        return 0;
    }

    progress_cdeg = (phase_distance_count * 18000L) / TASK3_ARC_LENGTH_COUNT;
    progress_cdeg = clamp_i32(progress_cdeg, 0, 18000);

    return -phase_turn_dir * progress_cdeg;
}

static uint8_t race_ir_mask_seen(const ir_tracking_sample_t *sample,
    uint8_t seen_mask,
    uint8_t forbid_mask)
{
    if ((sample == 0) || (sample->line_lost != 0U)) {
        return 0U;
    }

    if ((sample->line_mask & seen_mask) == 0U) {
        return 0U;
    }

    return ((sample->line_mask & forbid_mask) == 0U) ? 1U : 0U;
}

static uint8_t race_left_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_right_clear)
{
    return race_ir_mask_seen(sample,
        TASK11_IR_LEFT_EDGE_MASK,
        (require_right_clear != 0U) ? TASK11_IR_RIGHT_EDGE_MASK : 0U);
}

static uint8_t race_right_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_left_clear)
{
    return race_ir_mask_seen(sample,
        TASK11_IR_RIGHT_EDGE_MASK,
        (require_left_clear != 0U) ? TASK11_IR_LEFT_EDGE_MASK : 0U);
}

static uint8_t race_turn_crossed_target(uint8_t error_valid,
    int32_t last_error_cdeg,
    int32_t current_error_cdeg)
{
    return ((error_valid != 0U) &&
        (((last_error_cdeg < 0) && (current_error_cdeg >= 0)) ||
         ((last_error_cdeg > 0) && (current_error_cdeg <= 0)))) ? 1U : 0U;
}

static uint8_t race_sensor_fast_turn_line_seen(uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    return ((ir_ok != 0U) &&
        (sample->line_lost == 0U) &&
        (sample->active_count >= TASK11_IR_TURN_STOP_MIN_COUNT)) ? 1U : 0U;
}

static uint8_t race_sensor_fast_turn_line_ready(
    const sensor_fast_turn_config_t *config,
    uint8_t line_seen,
    const ir_tracking_sample_t *sample,
    uint8_t *center_ready,
    uint8_t *wide_ready,
    uint8_t *err_ready)
{
    *center_ready = ((line_seen != 0U) &&
        ((sample->line_mask & config->stop_mask) != 0U) &&
        ((sample->line_mask & config->forbid_mask) == 0U)) ? 1U : 0U;
    *wide_ready = ((line_seen != 0U) && (sample->line_mask == 0xFFU)) ? 1U : 0U;
    *err_ready = ((line_seen != 0U) &&
        (abs_i32(sample->error) <= config->stop_error_max)) ? 1U : 0U;

    return (((*center_ready) != 0U) ||
        ((*wide_ready) != 0U) ||
        ((*err_ready) != 0U)) ? 1U : 0U;
}

static uint8_t race_sensor_fast_turn_yaw_ready(
    const sensor_fast_turn_config_t *config,
    uint8_t nav_ok,
    int32_t yaw_stop_error_cdeg,
    uint8_t yaw_cross_ready)
{
    return ((config->yaw_stop_enable != 0U) &&
        (nav_ok != 0U) &&
        ((abs_i32(yaw_stop_error_cdeg) <= TASK11_TURN_YAW_STOP_TOL_CDEG) ||
         (yaw_cross_ready != 0U))) ? 1U : 0U;
}

static uint8_t race_sensor_fast_turn_should_slow(
    const sensor_fast_turn_config_t *config,
    uint8_t line_seen,
    int32_t turn_yaw_progress,
    int32_t yaw_stop_error_cdeg,
    uint8_t slow_mode)
{
    return (((line_seen != 0U) ||
        ((TASK11_FAST_TURN_GYRO_SLOW_ENABLE != 0) &&
         ((turn_yaw_progress >= TASK11_FAST_TURN_GYRO_SLOW_CDEG) ||
          ((config->yaw_stop_enable != 0U) &&
           (abs_i32(yaw_stop_error_cdeg) <= TASK11_TURN_YAW_SLOW_ZONE_CDEG))))) &&
        (slow_mode == 0U)) ? 1U : 0U;
}

static uint8_t race_sensor_fast_turn(
    const sensor_fast_turn_config_t *config)
{
    const char *tag = config->tag;
    int16_t motor_b_pwm = config->motor_b_pwm;
    int16_t motor_a_pwm = config->motor_a_pwm;
    int16_t slow_motor_b_pwm = config->slow_motor_b_pwm;
    int16_t slow_motor_a_pwm = config->slow_motor_a_pwm;
    uint8_t stop_mask = config->stop_mask;
    uint8_t forbid_mask = config->forbid_mask;
    int32_t stop_error_max = config->stop_error_max;
    uint8_t yaw_stop_enable = config->yaw_stop_enable;
    int32_t yaw_stop_target_cdeg = config->yaw_stop_target_cdeg;
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t turn_yaw_start = 0;
    int32_t turn_yaw_delta = 0;
    int32_t turn_yaw_progress = 0;
    int32_t yaw_stop_error_cdeg = 0;
    int32_t last_yaw_stop_error_cdeg = 0;
    int32_t turn_gyro_z_filtered_mdps = 0;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t turn_nav_ok = 0U;
    uint8_t line_stop_ready = 0U;
    uint8_t line_seen = 0U;
    uint8_t slow_mode = 0U;
    uint8_t center_ready = 0U;
    uint8_t wide_ready = 0U;
    uint8_t err_ready = 0U;
    uint8_t yaw_stop_ready = 0U;
    uint8_t yaw_error_valid = 0U;
    uint8_t yaw_cross_ready = 0U;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    turn_nav_ok = race_peek_yaw(&turn_yaw_start, &turn_gyro_z_filtered_mdps);
    if ((yaw_stop_enable != 0U) && (turn_nav_ok != 0U)) {
        last_yaw_stop_error_cdeg = normalize_cdeg(turn_yaw_start -
            yaw_stop_target_cdeg);
        yaw_error_valid = 1U;
        if (TASK11_FAST_TURN_GYRO_SLOW_ENABLE != 0) {
            slow_mode = 1U;
        }
    }
    TB6612_SetDifferential((slow_mode != 0U) ? slow_motor_b_pwm : motor_b_pwm,
        (slow_mode != 0U) ? slow_motor_a_pwm : motor_a_pwm);
    race_ram_log_event(TASK11_RAM_EVENT_TURN_START,
        0U,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(0U),
        0,
        g_race_post_point_phase_dist_count,
        (turn_nav_ok != 0U) ? turn_yaw_start : 0,
        0,
        0,
        yaw_stop_target_cdeg,
        0,
        0,
        (turn_nav_ok != 0U) ? turn_gyro_z_filtered_mdps : 0,
        0U,
        &sample,
        0,
        0);
    race_log_printf("%s start: sensor_fast_turn pwm=%d/%d slow=%d/%d stop_mask=0x%02X forbid=0x%02X err_max=%ld yaw_stop=%u target=%ld\r\n",
        tag,
        motor_b_pwm,
        motor_a_pwm,
        slow_motor_b_pwm,
        slow_motor_a_pwm,
        stop_mask,
        forbid_mask,
        stop_error_max,
        yaw_stop_enable,
        yaw_stop_target_cdeg);

    while (elapsed_ms < TASK11_FAST_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        nav_ok = JY62_PeekNavigation(&nav);
        turn_yaw_delta = ((turn_nav_ok != 0U) && (nav_ok != 0U)) ?
            normalize_cdeg(nav.yaw_relative_cdeg - turn_yaw_start) : 0;
        turn_yaw_progress = abs_i32(turn_yaw_delta);
        yaw_stop_error_cdeg = ((yaw_stop_enable != 0U) && (nav_ok != 0U)) ?
            normalize_cdeg(nav.yaw_relative_cdeg - yaw_stop_target_cdeg) : 0;
        yaw_cross_ready = 0U;
        if ((yaw_stop_enable != 0U) && (nav_ok != 0U)) {
            yaw_cross_ready = race_turn_crossed_target(yaw_error_valid,
                last_yaw_stop_error_cdeg,
                yaw_stop_error_cdeg);
            last_yaw_stop_error_cdeg = yaw_stop_error_cdeg;
            yaw_error_valid = 1U;
        }
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        line_seen = race_sensor_fast_turn_line_seen(ir_ok, &sample);
        yaw_stop_ready = race_sensor_fast_turn_yaw_ready(config,
            nav_ok,
            yaw_stop_error_cdeg,
            yaw_cross_ready);
        if (race_sensor_fast_turn_should_slow(config,
            line_seen,
            turn_yaw_progress,
            yaw_stop_error_cdeg,
            slow_mode) != 0U) {
            slow_mode = 1U;
            TB6612_SetDifferential(slow_motor_b_pwm, slow_motor_a_pwm);
        }
        line_stop_ready = race_sensor_fast_turn_line_ready(config,
            line_seen,
            &sample,
            &center_ready,
            &wide_ready,
            &err_ready);

        if (line_stop_ready != 0U) {
            stop_reason = 1U;
            break;
        }
        if (yaw_stop_ready != 0U) {
            stop_reason = 6U;
            break;
        }

        if (report_elapsed_ms >= TASK11_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            race_log_printf("%s t=%lu nav=%u yaw=%ld yprog=%ld target=%ld yerr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u seen=%u center=%u wide=%u err_ok=%u line_ready=%u yaw_ready=%u\r\n",
                tag,
                elapsed_ms,
                nav_ok,
                (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
                turn_yaw_progress,
                yaw_stop_target_cdeg,
                yaw_stop_error_cdeg,
                (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                motor_b_total,
                motor_a_total,
                slow_mode,
                line_seen,
                center_ready,
                wide_ready,
                err_ready,
                line_stop_ready,
                yaw_stop_ready);
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    TB6612_Brake();
    ir_ok = IRTracking_ReadSample(&sample);
    nav_ok = JY62_PeekNavigation(&nav);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    race_ram_log_event(TASK11_RAM_EVENT_TURN_STOP,
        stop_reason,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(elapsed_ms),
        motion_distance_count(motor_b_total, motor_a_total),
        g_race_post_point_phase_dist_count,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        turn_yaw_progress,
        turn_yaw_delta,
        yaw_stop_target_cdeg,
        ((yaw_stop_enable != 0U) && (nav_ok != 0U)) ?
            normalize_cdeg(nav.yaw_relative_cdeg - yaw_stop_target_cdeg) : 0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
    race_log_printf("%s stop: reason=%s t=%lu nav=%u yaw=%ld target=%ld yerr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u yaw_ready=%u\r\n",
        tag,
        (stop_reason == 1U) ? "line" :
            ((stop_reason == 4U) ? "wide" :
            ((stop_reason == 6U) ? "yaw" :
            ((stop_reason == 2U) ? "timeout" : "uart_stop"))),
        elapsed_ms,
        nav_ok,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        yaw_stop_target_cdeg,
        ((yaw_stop_enable != 0U) && (nav_ok != 0U)) ?
            normalize_cdeg(nav.yaw_relative_cdeg - yaw_stop_target_cdeg) : 0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0,
        motor_b_total,
        motor_a_total,
        slow_mode,
        yaw_stop_ready);

    encoder_reset_distance_counts();
    g_race_post_point_elapsed_ms += elapsed_ms;
    return ((stop_reason == 1U) || (stop_reason == 4U) ||
        (stop_reason == 6U)) ? 1U : 0U;
}

static uint8_t race_gyro_turn_to_yaw(
    const gyro_turn_config_t *config)
{
    const char *tag = config->tag;
    int16_t motor_b_pwm = config->motor_b_pwm;
    int16_t motor_a_pwm = config->motor_a_pwm;
    int16_t slow_motor_b_pwm = config->slow_motor_b_pwm;
    int16_t slow_motor_a_pwm = config->slow_motor_a_pwm;
    int32_t yaw_stop_target_cdeg = config->yaw_stop_target_cdeg;
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t turn_yaw_start = 0;
    int32_t turn_yaw_delta = 0;
    int32_t yaw_stop_error_cdeg = 0;
    int32_t last_yaw_stop_error_cdeg = 0;
    int32_t turn_gyro_z_filtered_mdps = 0;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t slow_mode = 0U;
    uint8_t yaw_error_valid = 0U;
    uint8_t yaw_cross_ready = 0U;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = race_peek_yaw(&turn_yaw_start, &turn_gyro_z_filtered_mdps);
    if (nav_ok == 0U) {
        TB6612_Brake();
        race_log_printf("%s abort: gyro_turn nav=0 target=%ld\r\n",
            tag,
            yaw_stop_target_cdeg);
        return 0U;
    }

    last_yaw_stop_error_cdeg = normalize_cdeg(turn_yaw_start -
        yaw_stop_target_cdeg);
    yaw_error_valid = 1U;
    if (abs_i32(last_yaw_stop_error_cdeg) <= TASK11_TURN_YAW_SLOW_ZONE_CDEG) {
        slow_mode = 1U;
    }
    TB6612_SetDifferential((slow_mode != 0U) ? slow_motor_b_pwm : motor_b_pwm,
        (slow_mode != 0U) ? slow_motor_a_pwm : motor_a_pwm);
    race_ram_log_event(TASK11_RAM_EVENT_TURN_START,
        0U,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(0U),
        0,
        g_race_post_point_phase_dist_count,
        turn_yaw_start,
        0,
        0,
        yaw_stop_target_cdeg,
        last_yaw_stop_error_cdeg,
        0,
        turn_gyro_z_filtered_mdps,
        0U,
        &sample,
        0,
        0);
    race_log_printf("%s start: gyro_turn pwm=%d/%d slow=%d/%d yaw0=%ld target=%ld yerr=%ld timeout=%d\r\n",
        tag,
        motor_b_pwm,
        motor_a_pwm,
        slow_motor_b_pwm,
        slow_motor_a_pwm,
        turn_yaw_start,
        yaw_stop_target_cdeg,
        last_yaw_stop_error_cdeg,
        TASK11_GYRO_TURN_TIMEOUT_MS);

    while (elapsed_ms < TASK11_GYRO_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok == 0U) {
            stop_reason = 5U;
            break;
        }

        turn_yaw_delta = normalize_cdeg(nav.yaw_relative_cdeg - turn_yaw_start);
        yaw_stop_error_cdeg = normalize_cdeg(nav.yaw_relative_cdeg -
            yaw_stop_target_cdeg);
        yaw_cross_ready = race_turn_crossed_target(yaw_error_valid,
            last_yaw_stop_error_cdeg,
            yaw_stop_error_cdeg);
        last_yaw_stop_error_cdeg = yaw_stop_error_cdeg;
        yaw_error_valid = 1U;

        if ((slow_mode == 0U) &&
            (abs_i32(yaw_stop_error_cdeg) <= TASK11_TURN_YAW_SLOW_ZONE_CDEG)) {
            slow_mode = 1U;
            TB6612_SetDifferential(slow_motor_b_pwm, slow_motor_a_pwm);
        }
        if ((abs_i32(yaw_stop_error_cdeg) <= TASK11_TURN_YAW_STOP_TOL_CDEG) ||
            (yaw_cross_ready != 0U)) {
            stop_reason = 6U;
            break;
        }

        if (report_elapsed_ms >= TASK11_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            encoder_get_total_counts(&motor_b_total, &motor_a_total);
            race_log_printf("%s t=%lu nav=%u yaw=%ld target=%ld yerr=%ld ydelta=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u\r\n",
                tag,
                elapsed_ms,
                nav_ok,
                nav.yaw_relative_cdeg,
                yaw_stop_target_cdeg,
                yaw_stop_error_cdeg,
                turn_yaw_delta,
                nav.gyro_z_filtered_mdps,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                motor_b_total,
                motor_a_total,
                slow_mode);
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    TB6612_Brake();
    ir_ok = IRTracking_ReadSample(&sample);
    nav_ok = JY62_PeekNavigation(&nav);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    race_ram_log_event(TASK11_RAM_EVENT_TURN_STOP,
        stop_reason,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(elapsed_ms),
        motion_distance_count(motor_b_total, motor_a_total),
        g_race_post_point_phase_dist_count,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? abs_i32(turn_yaw_delta) : 0,
        (nav_ok != 0U) ? turn_yaw_delta : 0,
        yaw_stop_target_cdeg,
        (nav_ok != 0U) ? normalize_cdeg(nav.yaw_relative_cdeg -
            yaw_stop_target_cdeg) : 0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
    race_log_printf("%s stop: reason=%s t=%lu nav=%u yaw=%ld target=%ld yerr=%ld ydelta=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u\r\n",
        tag,
        race_reason_name(stop_reason),
        elapsed_ms,
        nav_ok,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        yaw_stop_target_cdeg,
        (nav_ok != 0U) ? normalize_cdeg(nav.yaw_relative_cdeg -
            yaw_stop_target_cdeg) : 0,
        (nav_ok != 0U) ? turn_yaw_delta : 0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0,
        motor_b_total,
        motor_a_total,
        slow_mode);

    encoder_reset_distance_counts();
    g_race_post_point_elapsed_ms += elapsed_ms;
    return (stop_reason == 6U) ? 1U : 0U;
}

static uint8_t race_advance_after_point(const char *tag, int32_t advance_count)
{
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;

    if (advance_count <= 0) {
        race_ram_log_event(TASK11_RAM_EVENT_ADVANCE_STOP,
            1U,
            g_race_log_lap,
            g_race_log_phase,
            0U,
            race_post_point_event_ms(0U),
            0,
            g_race_post_point_phase_dist_count,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0U,
            &sample,
            0,
            0);
        race_log_printf("%s skip: advance_count=%ld\r\n", tag, advance_count);
        return 1U;
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)TASK11_POINT_ADVANCE_PWM,
        (int16_t)TASK11_POINT_ADVANCE_PWM);
    race_ram_log_event(TASK11_RAM_EVENT_ADVANCE_START,
        0U,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(0U),
        0,
        g_race_post_point_phase_dist_count,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0U,
        &sample,
        0,
        0);
    race_log_printf("%s start: advance_count=%ld pwm=%d\r\n",
        tag,
        advance_count,
        TASK11_POINT_ADVANCE_PWM);

    while (elapsed_ms < TASK11_POINT_ADVANCE_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        nav_ok = JY62_PeekNavigation(&nav);
        ir_ok = IRTracking_ReadSample(&sample);

        if (distance_count >= advance_count) {
            stop_reason = 1U;
            break;
        }

        if (report_elapsed_ms >= TASK11_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            race_log_printf("%s t=%lu dist=%ld nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                nav_ok,
                (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
                (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                motor_b_total,
                motor_a_total);
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    distance_count = motion_distance_count(motor_b_total, motor_a_total);
    nav_ok = JY62_PeekNavigation(&nav);
    ir_ok = IRTracking_ReadSample(&sample);
    race_ram_log_event(TASK11_RAM_EVENT_ADVANCE_STOP,
        stop_reason,
        g_race_log_lap,
        g_race_log_phase,
        0U,
        race_post_point_event_ms(elapsed_ms),
        distance_count,
        g_race_post_point_phase_dist_count,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        0,
        0,
        0,
        0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
    g_race_post_point_elapsed_ms += elapsed_ms;
    race_log_printf("%s stop: reason=%s t=%lu dist=%ld nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "distance" : ((stop_reason == 2U) ? "timeout" : "uart_stop"),
        elapsed_ms,
        distance_count,
        nav_ok,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0,
        motor_b_total,
        motor_a_total);

    return (stop_reason == 1U) ? 1U : 0U;
}

static void race_read_navigation_state(race_context_t *ctx, uint8_t reset_phase)
{
    ctx->nav_frame_delta = JY62_GetNavigation(&ctx->nav);
    ctx->nav_ok = ctx->nav.valid;
    ctx->nav_update_flags = ctx->nav.update_flags;

    if (ctx->nav_ok != 0U) {
        if (reset_phase != 0U) {
            ctx->yaw_start = ctx->nav.yaw_relative_cdeg;
        }
        ctx->yaw_cdeg = ctx->nav.yaw_relative_cdeg;
        ctx->yaw_raw_cdeg = ctx->nav.yaw_cdeg;
        ctx->phase_yaw_cdeg = (reset_phase != 0U) ? 0 :
            normalize_cdeg(ctx->yaw_cdeg - ctx->yaw_start);
        ctx->yaw_progress_cdeg = abs_i32(ctx->phase_yaw_cdeg);
        ctx->gyro_z_mdps = ctx->nav.gyro_z_mdps;
        ctx->gyro_z_filtered_mdps = ctx->nav.gyro_z_filtered_mdps;
        ctx->roll_cdeg = ctx->nav.roll_cdeg;
        ctx->pitch_cdeg = ctx->nav.pitch_cdeg;
    } else {
        if (reset_phase != 0U) {
            ctx->yaw_start = 0;
        }
        ctx->yaw_cdeg = 0;
        ctx->yaw_raw_cdeg = 0;
        ctx->phase_yaw_cdeg = 0;
        ctx->yaw_progress_cdeg = 0;
        ctx->gyro_z_mdps = 0;
        ctx->gyro_z_filtered_mdps = 0;
        ctx->roll_cdeg = 0;
        ctx->pitch_cdeg = 0;
    }
}

static void race_configure_phase(const race_context_t *ctx,
    race_phase_config_t *config)
{
    if (ctx->phase == 0U) {
        config->phase_name = "AC";
        config->point_name = "C_LINE";
        config->force_name = "C_FORCE";
        config->arc_mode = 0U;
        config->phase_turn_dir = 0;
        config->straight_target_cdeg = TASK11_AC_HEADING_TARGET_CDEG;
        config->point_arm_count = TASK11_AC_POINT_ARM_COUNT;
        config->force_count = TASK11_STRAIGHT_FORCE_COUNT;
    } else if (ctx->phase == 1U) {
        config->phase_name = "CB";
        config->point_name = "B_EXIT";
        config->force_name = "B_FORCE";
        config->arc_mode = 1U;
        config->phase_turn_dir = TASK3_ARC_TURN_LEFT;
        config->straight_target_cdeg = 0;
        config->point_arm_count = TASK3_ARC_EXIT_IGNORE_COUNT;
        config->force_count = TASK3_ARC_FORCE_STOP_COUNT;
    } else if (ctx->phase == 2U) {
        config->phase_name = "BD";
        config->point_name = "D_LINE";
        config->force_name = "D_FORCE";
        config->arc_mode = 0U;
        config->phase_turn_dir = 0;
        config->straight_target_cdeg = TASK11_BD_HEADING_TARGET_CDEG;
        config->point_arm_count = TASK11_BD_POINT_ARM_COUNT;
        config->force_count = TASK11_STRAIGHT_FORCE_COUNT;
    } else {
        config->phase_name = "DA";
        config->point_name = ((uint8_t)(ctx->lap_count + 1U) < ctx->target_laps) ?
            "A_EXIT" : "A_FINISH";
        config->force_name = "A_FORCE";
        config->arc_mode = 1U;
        config->phase_turn_dir = TASK3_ARC_TURN_RIGHT;
        config->straight_target_cdeg = 0;
        config->point_arm_count = TASK3_ARC_EXIT_IGNORE_COUNT;
        config->force_count = TASK3_ARC_FORCE_STOP_COUNT;
    }
}

static void race_update_loop_state(race_context_t *ctx,
    const race_phase_config_t *config)
{
    encoder_get_delta_counts(&ctx->motor_b_delta, &ctx->motor_a_delta);
    encoder_get_total_counts(&ctx->motor_b_total, &ctx->motor_a_total);
    ctx->total_distance_count = motion_distance_count(ctx->motor_b_total,
        ctx->motor_a_total);
    ctx->phase_distance_count = ctx->total_distance_count - ctx->phase_start_count;

    race_read_navigation_state(ctx, 0U);

    ctx->ir_ok = IRTracking_ReadSample(&ctx->sample);
    ctx->line_valid = ((ctx->ir_ok != 0U) &&
        (ctx->sample.line_lost == 0U)) ? 1U : 0U;
    ctx->line_lost_seen = ((ctx->ir_ok != 0U) &&
        (ctx->sample.line_lost != 0U)) ? 1U : 0U;

    if ((config->arc_mode == 0U) && (ctx->line_valid != 0U)) {
        if (ctx->straight_line_seen_count < 65535U) {
            ctx->straight_line_seen_count++;
        }
    }

    if (ctx->phase == 0U) {
        ctx->edge_point_seen = ((ctx->ir_ok != 0U) &&
            (race_left_edge_seen(&ctx->sample, 1U) != 0U)) ? 1U : 0U;
    } else if (ctx->phase == 2U) {
        ctx->edge_point_seen = ((ctx->ir_ok != 0U) &&
            (race_right_edge_seen(&ctx->sample, 0U) != 0U)) ? 1U : 0U;
    } else {
        ctx->edge_point_seen = 0U;
    }

    ctx->point_log_flags = (uint8_t)((ctx->ir_ok != 0U) ?
        TASK11_LOG_FLAG_IR_OK : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->line_lost_seen != 0U) ?
        TASK11_LOG_FLAG_LINE_LOST : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->edge_point_seen != 0U) ?
        TASK11_LOG_FLAG_EDGE_SEEN : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->straight_line_seen_count != 0U) ?
        TASK11_LOG_FLAG_GUIDE_SEEN : 0U);
    ctx->point_log_flags |= (uint8_t)((config->arc_mode != 0U) ?
        TASK11_LOG_FLAG_ARC_MODE : 0U);
}

static void race_compute_loop_control(race_context_t *ctx,
    const race_phase_config_t *config)
{
    ctx->raw_error = 0;
    ctx->derivative = 0;
    ctx->line_turn = 0;
    ctx->nav_turn = 0;
    ctx->control_turn = 0;
    ctx->heading_error_cdeg = 0;
    ctx->expected_yaw_cdeg = 0;
    ctx->arc_actual_yaw_cdeg = 0;

    if (config->arc_mode != 0U) {
        ctx->base_pwm = TASK11_ARC_BASE_PWM;
        if (ctx->phase == 1U) {
            ctx->target_speed_diff =
                (ctx->phase_distance_count < TASK3_ARC_ENTRY_COUNT) ?
                    TASK11_CB_ARC_ENTRY_TARGET_DIFF :
                    TASK11_CB_ARC_CRUISE_TARGET_DIFF;
        } else {
            ctx->target_speed_diff =
                (ctx->phase_distance_count < TASK3_ARC_ENTRY_COUNT) ?
                    TASK11_DA_ARC_ENTRY_TARGET_DIFF :
                    TASK11_DA_ARC_CRUISE_TARGET_DIFF;
        }
    } else {
        ctx->base_pwm = TASK11_STRAIGHT_BASE_PWM;
        ctx->target_speed_diff = TASK11_STRAIGHT_TARGET_DIFF;
    }

    if ((ctx->line_valid == 0U) &&
        ((config->arc_mode != 0U) || (TASK11_STRAIGHT_GYRO_NAV_ENABLE == 0))) {
        ctx->base_pwm -= TASK11_LINE_LOST_BASE_DROP;
    }

    race_drive_config(&ctx->drive_config, ctx->base_pwm, ctx->target_speed_diff);
    straight_drive_update(&ctx->diff_pid,
        &ctx->drive_config,
        ctx->motor_b_delta,
        ctx->motor_a_delta,
        ctx->motor_b_total,
        ctx->motor_a_total,
        &ctx->drive);

    if (ctx->line_valid != 0U) {
        ctx->raw_error = ctx->sample.error;
        ctx->filtered_error += (ctx->raw_error - ctx->filtered_error) /
            TASK11_LINE_ERROR_FILTER_DIVISOR;
        ctx->derivative = clamp_i32(ctx->filtered_error - ctx->last_filtered_error,
            -TASK11_LINE_DERIV_LIMIT,
            TASK11_LINE_DERIV_LIMIT);
        ctx->last_filtered_error = ctx->filtered_error;
        ctx->line_turn = (ctx->filtered_error / TASK11_LINE_TURN_DIVISOR) +
            (ctx->derivative / TASK11_LINE_KD_DIVISOR);
        ctx->line_turn = clamp_i32(ctx->line_turn,
            -TASK11_LINE_TURN_LIMIT,
            TASK11_LINE_TURN_LIMIT);
        ctx->last_turn = ctx->line_turn;
    } else if (ctx->last_turn != 0) {
        ctx->line_turn = clamp_i32(ctx->last_turn,
            -TASK11_LINE_LOST_TURN,
            TASK11_LINE_LOST_TURN);
    } else {
        ctx->line_turn = config->phase_turn_dir * TASK11_LINE_LOST_TURN;
    }

    if (config->arc_mode != 0U) {
        ctx->expected_yaw_cdeg = race_arc_expected_yaw_cdeg(
            ctx->phase_distance_count,
            config->phase_turn_dir);
    } else {
        ctx->expected_yaw_cdeg = config->straight_target_cdeg;
    }

    if (ctx->nav_ok != 0U) {
        if (config->arc_mode == 0U) {
            ctx->heading_error_cdeg =
                normalize_cdeg(ctx->yaw_cdeg - ctx->expected_yaw_cdeg);
            if (TASK11_STRAIGHT_GYRO_NAV_ENABLE != 0) {
                ctx->nav_turn = race_heading_turn_from_error(
                    ctx->heading_error_cdeg,
                    ctx->gyro_z_filtered_mdps,
                    TASK11_STRAIGHT_HEADING_CORR_DIVISOR,
                    TASK11_STRAIGHT_GYRO_DAMP_DIVISOR,
                    TASK11_STRAIGHT_HEADING_CORR_MAX);
            }
        } else {
            ctx->arc_actual_yaw_cdeg = ctx->phase_yaw_cdeg;
            ctx->heading_error_cdeg = normalize_cdeg(ctx->arc_actual_yaw_cdeg -
                ctx->expected_yaw_cdeg);
            if (TASK11_ARC_YAW_NAV_ENABLE != 0) {
                ctx->nav_turn = race_heading_turn_from_error(
                    ctx->heading_error_cdeg,
                    ctx->gyro_z_filtered_mdps,
                    TASK11_ARC_YAW_CORR_DIVISOR,
                    TASK11_ARC_GYRO_DAMP_DIVISOR,
                    TASK11_ARC_YAW_CORR_MAX);
            }
        }
    }

    if (config->arc_mode != 0U) {
        ctx->control_turn = clamp_i32(ctx->line_turn + ctx->nav_turn,
            -TASK11_LINE_TURN_LIMIT,
            TASK11_LINE_TURN_LIMIT);
    } else if ((ctx->nav_ok != 0U) && (TASK11_STRAIGHT_GYRO_NAV_ENABLE != 0)) {
        ctx->control_turn = ctx->nav_turn;
#if TASK11_STRAIGHT_IR_ASSIST_ENABLE
        if (ctx->line_valid != 0U) {
            ctx->control_turn = clamp_i32(ctx->control_turn +
                (ctx->line_turn / TASK11_STRAIGHT_IR_ASSIST_DIVISOR),
                -TASK11_LINE_TURN_LIMIT,
                TASK11_LINE_TURN_LIMIT);
        }
#endif
    } else {
        ctx->control_turn = ctx->line_turn;
    }

    ctx->left_pwm = clamp_i32(ctx->drive.motor_b_pwm + ctx->control_turn,
        TASK11_LINE_MIN_PWM,
        TASK11_LINE_MAX_PWM);
    ctx->right_pwm = clamp_i32(ctx->drive.motor_a_pwm - ctx->control_turn,
        TASK11_LINE_MIN_PWM,
        TASK11_LINE_MAX_PWM);
}

static uint8_t race_check_phase_point(race_context_t *ctx,
    const race_phase_config_t *config)
{
    if (config->arc_mode == 0U) {
        ctx->straight_point_candidate =
            ((ctx->phase_distance_count >= config->point_arm_count) &&
             (ctx->line_valid != 0U)) ? 1U : 0U;
        if (ctx->straight_point_candidate != 0U) {
            if (ctx->straight_point_count < TASK11_STRAIGHT_POINT_CONFIRM_COUNT) {
                ctx->straight_point_count++;
            }
        } else {
            ctx->straight_point_count = 0U;
        }
        ctx->point_ready = (ctx->straight_point_count >=
            TASK11_STRAIGHT_POINT_CONFIRM_COUNT) ? 1U : 0U;
    } else if ((ctx->phase == 1U) || (ctx->phase == 3U)) {
        ctx->point_ready = (ctx->line_lost_seen != 0U) ? 1U : 0U;
    } else {
        ctx->point_ready = ((ctx->phase_distance_count >= config->point_arm_count) &&
            (ctx->yaw_progress_cdeg >= TASK11_ARC_POINT_YAW_ARM_CDEG) &&
            (ctx->edge_point_seen != 0U)) ? 1U : 0U;
    }

    return ctx->point_ready;
}

static void race_log_loop_samples(race_context_t *ctx,
    const race_phase_config_t *config)
{
    race_ram_log_segment_sample(ctx->ir_ok,
        ctx->nav_ok,
        &ctx->sample,
        ctx->phase_distance_count,
        ctx->line_turn,
        ctx->nav_turn,
        ctx->control_turn,
        ctx->heading_error_cdeg,
        ctx->gyro_z_mdps,
        ctx->gyro_z_filtered_mdps,
        ctx->nav_frame_delta,
        ctx->nav_update_flags);

    if (race_ram_window_should_log(ctx->lap_count,
        ctx->phase_distance_count,
        config->point_arm_count) != 0U) {
        uint8_t window_log_flags = ctx->point_log_flags;

        if (ctx->phase_distance_count <= TASK11_RAM_WINDOW_AFTER_START_COUNT) {
            window_log_flags |= TASK11_LOG_FLAG_START_WINDOW;
        }
        race_ram_log_window_sample(ctx->lap_count,
            ctx->phase,
            ctx->ir_ok,
            &ctx->sample,
            ctx->edge_point_seen,
            window_log_flags,
            ctx->elapsed_ms,
            ctx->phase_distance_count,
            ctx->yaw_cdeg,
            ctx->yaw_raw_cdeg,
            ctx->phase_yaw_cdeg,
            ctx->yaw_progress_cdeg,
            ctx->expected_yaw_cdeg,
            ctx->heading_error_cdeg,
            ctx->nav_turn,
            ctx->control_turn,
            ctx->gyro_z_mdps,
            ctx->gyro_z_filtered_mdps,
            ctx->roll_cdeg,
            ctx->pitch_cdeg,
            ctx->nav_frame_delta,
            ctx->nav_update_flags);
    }
}

static void race_log_point_state(const race_context_t *ctx,
    const race_phase_config_t *config,
    uint8_t reason,
    uint8_t event_type)
{
    const char *event_name = (event_type == TASK11_RAM_EVENT_POINT) ?
        "point" : "force";
    const char *name = (event_type == TASK11_RAM_EVENT_POINT) ?
        config->point_name : config->force_name;

    race_ram_log_segment_finish(reason,
        ctx->elapsed_ms,
        ctx->phase_distance_count,
        ctx->yaw_cdeg,
        ctx->yaw_progress_cdeg,
        ctx->heading_error_cdeg,
        ctx->ir_ok,
        &ctx->sample,
        ctx->point_log_flags);
    race_ram_log_event(event_type,
        reason,
        ctx->lap_count,
        ctx->phase,
        ctx->point_log_flags,
        ctx->elapsed_ms,
        ctx->phase_distance_count,
        ctx->phase_distance_count,
        ctx->yaw_cdeg,
        ctx->yaw_progress_cdeg,
        ctx->phase_yaw_cdeg,
        ctx->expected_yaw_cdeg,
        ctx->heading_error_cdeg,
        ctx->nav_turn,
        ctx->gyro_z_filtered_mdps,
        ctx->ir_ok,
        &ctx->sample,
        ctx->motor_b_total,
        ctx->motor_a_total);
    race_event_printf("RACE_EVT_RT lap=%u seg=%s event=%s t=%lu dist=%ld mask=0x%02X\r\n",
        ctx->lap_count,
        config->phase_name,
        event_name,
        ctx->elapsed_ms,
        ctx->phase_distance_count,
        (ctx->ir_ok != 0U) ? ctx->sample.line_mask : 0U);
    race_print_point(name, &ctx->result);
    st011_start_pulse(TASK11_POINT_ALARM_MS);
    race_log_printf("RACE_POINT_STATE seg=%s edge=%u line_seen_n=%u pflags=0x%02X nav=%u nav_fd=%lu upd=0x%02X yaw=%ld yaw_raw=%ld pyaw=%ld yprog=%ld exp=%ld herr=%ld gz=%ld gzlp=%ld roll=%ld pitch=%ld line_turn=%ld nav_turn=%ld turn=%ld tdiff=%ld ff=%ld raw=0x%02X mask=0x%02X cnt=%u\r\n",
        config->phase_name,
        ctx->edge_point_seen,
        ctx->straight_line_seen_count,
        ctx->point_log_flags,
        ctx->nav_ok,
        ctx->nav_frame_delta,
        ctx->nav_update_flags,
        ctx->yaw_cdeg,
        ctx->yaw_raw_cdeg,
        ctx->phase_yaw_cdeg,
        ctx->yaw_progress_cdeg,
        ctx->expected_yaw_cdeg,
        ctx->heading_error_cdeg,
        ctx->gyro_z_mdps,
        ctx->gyro_z_filtered_mdps,
        ctx->roll_cdeg,
        ctx->pitch_cdeg,
        ctx->line_turn,
        ctx->nav_turn,
        ctx->control_turn,
        ctx->target_speed_diff,
        ctx->drive.feedforward_correction,
        (ctx->ir_ok != 0U) ? ctx->sample.raw : 0xFFU,
        (ctx->ir_ok != 0U) ? ctx->sample.line_mask : 0U,
        (ctx->ir_ok != 0U) ? ctx->sample.active_count : 0U);

    if (event_type == TASK11_RAM_EVENT_POINT) {
        race_post_point_context_begin(ctx->elapsed_ms, ctx->phase_distance_count);
    }
}

static void race_capture_result(race_context_t *ctx, uint8_t reason)
{
    ctx->result.reason = reason;
    ctx->result.elapsed_ms = ctx->elapsed_ms;
    ctx->result.distance_count = ctx->phase_distance_count;
    ctx->result.yaw_cdeg = ctx->yaw_cdeg;
    ctx->result.yaw_progress_cdeg = ctx->yaw_progress_cdeg;
    ctx->result.ir_ok = ctx->ir_ok;
    ctx->result.sample = ctx->sample;
}

static uint8_t race_execute_point_action(const race_context_t *ctx)
{
    uint8_t turn_success = 1U;

    if (ctx->phase == 0U) {
        const sensor_fast_turn_config_t turn_config = {
            .tag = "RACE_C_LEFT_TURN",
            .motor_b_pwm = TASK11_LEFT_TURN_B_PWM,
            .motor_a_pwm = TASK11_LEFT_TURN_A_PWM,
            .slow_motor_b_pwm = TASK11_LEFT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = TASK11_LEFT_TURN_SLOW_A_PWM,
            .stop_mask = TASK11_IR_CENTER_6_MASK,
            .forbid_mask = TASK11_IR_CENTER_6_FORBID_MASK,
            .stop_error_max = TASK11_TURN_CENTER6_ERROR_MAX,
            .yaw_stop_enable = 0U,
            .yaw_stop_target_cdeg = 0
        };
        turn_success = race_advance_after_point("RACE_C_ADVANCE",
            TASK11_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_sensor_fast_turn(&turn_config);
        }
    } else if (ctx->phase == 1U) {
        const gyro_turn_config_t turn_config = {
            .tag = "RACE_B_GYRO_TO_BD",
            .motor_b_pwm = TASK11_EXIT_LEFT_TURN_B_PWM,
            .motor_a_pwm = TASK11_EXIT_LEFT_TURN_A_PWM,
            .slow_motor_b_pwm = TASK11_EXIT_LEFT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = TASK11_EXIT_LEFT_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = TASK11_BD_HEADING_TARGET_CDEG
        };
        turn_success = race_advance_after_point("RACE_B_ADVANCE",
            TASK11_ARC_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_gyro_turn_to_yaw(&turn_config);
        }
    } else if (ctx->phase == 2U) {
        const sensor_fast_turn_config_t turn_config = {
            .tag = "RACE_D_RIGHT_TURN",
            .motor_b_pwm = TASK11_RIGHT_TURN_B_PWM,
            .motor_a_pwm = TASK11_RIGHT_TURN_A_PWM,
            .slow_motor_b_pwm = TASK11_RIGHT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = TASK11_RIGHT_TURN_SLOW_A_PWM,
            .stop_mask = TASK11_IR_CENTER_6_MASK,
            .forbid_mask = TASK11_IR_CENTER_6_FORBID_MASK,
            .stop_error_max = TASK11_TURN_CENTER6_ERROR_MAX,
            .yaw_stop_enable = 0U,
            .yaw_stop_target_cdeg = 0
        };
        turn_success = race_advance_after_point("RACE_D_ADVANCE",
            TASK11_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_sensor_fast_turn(&turn_config);
        }
    } else if ((uint8_t)(ctx->lap_count + 1U) < ctx->target_laps) {
        const gyro_turn_config_t turn_config = {
            .tag = "RACE_A_GYRO_TO_AC",
            .motor_b_pwm = TASK11_EXIT_RIGHT_TURN_B_PWM,
            .motor_a_pwm = TASK11_EXIT_RIGHT_TURN_A_PWM,
            .slow_motor_b_pwm = TASK11_EXIT_RIGHT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = TASK11_EXIT_RIGHT_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = TASK11_AC_HEADING_TARGET_CDEG
        };
        turn_success = race_advance_after_point("RACE_A_ADVANCE",
            TASK11_ARC_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_gyro_turn_to_yaw(&turn_config);
        }
    }

    return turn_success;
}

static void race_reset_segment_control(race_context_t *ctx)
{
    ctx->straight_point_count = 0U;
    ctx->straight_line_seen_count = 0U;
    ctx->filtered_error = 0;
    ctx->last_filtered_error = 0;
    ctx->last_turn = 0;
    ctx->report_elapsed_ms = 0;
    race_diff_pid_reset(&ctx->diff_pid);
}

static void race_advance_segment(race_context_t *ctx, uint8_t point_event)
{
    if (ctx->phase == 3U) {
        ctx->lap_count++;
        if (ctx->lap_count >= ctx->target_laps) {
            ctx->stop_reason = (point_event != 0U) ? 1U : 2U;
            return;
        }
        ctx->phase = 0U;
    } else {
        ctx->phase++;
    }

    if (point_event != 0U) {
        ctx->phase_start_count = 0;
        race_reset_segment_control(ctx);
        race_read_navigation_state(ctx, 1U);
    } else {
        ctx->phase_start_count = ctx->total_distance_count;
        ctx->yaw_start = ctx->yaw_cdeg;
        race_reset_segment_control(ctx);
    }

    race_ram_log_segment_reset(ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->yaw_start);
    race_log_segment_start_snapshot(ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->nav_ok,
        ctx->yaw_cdeg,
        ctx->gyro_z_filtered_mdps);
}

static void race_log_periodic_data(race_context_t *ctx,
    const race_phase_config_t *config)
{
    if (ctx->report_elapsed_ms < TASK11_LINE_REPORT_PERIOD_MS) {
        return;
    }

    ctx->report_elapsed_ms = 0;
    race_log_printf("RACE_DATA lap=%u seg=%s phase=%u t=%lu dist=%ld edge=%u line_seen_n=%u pflags=0x%02X nav=%u nav_fd=%lu upd=0x%02X yaw=%ld yaw_raw=%ld pyaw=%ld yprog=%ld exp=%ld herr=%ld gz=%ld gzlp=%ld roll=%ld pitch=%ld raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld filt=%ld der=%ld line_turn=%ld nav_turn=%ld turn=%ld tdiff=%ld ff=%ld fb=%ld corr=%ld base=%ld pwm=%ld/%ld\r\n",
        ctx->lap_count,
        config->phase_name,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->phase_distance_count,
        ctx->edge_point_seen,
        ctx->straight_line_seen_count,
        ctx->point_log_flags,
        ctx->nav_ok,
        ctx->nav_frame_delta,
        ctx->nav_update_flags,
        ctx->yaw_cdeg,
        ctx->yaw_raw_cdeg,
        ctx->phase_yaw_cdeg,
        ctx->yaw_progress_cdeg,
        ctx->expected_yaw_cdeg,
        ctx->heading_error_cdeg,
        ctx->gyro_z_mdps,
        ctx->gyro_z_filtered_mdps,
        ctx->roll_cdeg,
        ctx->pitch_cdeg,
        (ctx->ir_ok != 0U) ? ctx->sample.raw : 0xFFU,
        (ctx->ir_ok != 0U) ? ctx->sample.line_mask : 0U,
        (ctx->ir_ok != 0U) ? ctx->sample.active_count : 0U,
        (ctx->ir_ok != 0U) ? ctx->sample.line_lost : 1U,
        (ctx->ir_ok != 0U) ? ctx->sample.error : 0,
        ctx->filtered_error,
        ctx->derivative,
        ctx->line_turn,
        ctx->nav_turn,
        ctx->control_turn,
        ctx->target_speed_diff,
        ctx->drive.feedforward_correction,
        ctx->drive.feedback_correction,
        ctx->drive.correction,
        ctx->base_pwm,
        ctx->left_pwm,
        ctx->right_pwm);
}

static void race_init_lap_context(race_context_t *ctx, uint8_t target_laps)
{
    ctx->target_laps = (target_laps == 0U) ? 1U : target_laps;

    TB6612_Brake();
    delay_ms_with_st011(TASK11_POINT_SETTLE_MS);
    (void)jy62_zero_to_current("RACE_AC_ZERO", 0U);
    delay_ms_with_st011(TASK11_POINT_SETTLE_MS);
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    race_diff_pid_reset(&ctx->diff_pid);
    race_read_navigation_state(ctx, 1U);

    race_ram_log_reset();
    race_ram_log_segment_reset(ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->yaw_start);
    race_ram_log_event(TASK11_RAM_EVENT_START,
        0U,
        ctx->lap_count,
        ctx->phase,
        0U,
        ctx->elapsed_ms,
        0,
        0,
        ctx->yaw_cdeg,
        0,
        0,
        0,
        0,
        0,
        ctx->gyro_z_filtered_mdps,
        0U,
        &ctx->sample,
        0,
        0);
    race_log_segment_start_snapshot(ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->nav_ok,
        ctx->yaw_cdeg,
        ctx->gyro_z_filtered_mdps);

    race_log_printf("RACE start: ac_zero_collect laps=%u yaw=%ld nav=%u nav_fd=%lu upd=0x%02X base=%d arc_base=%d ac_tgt=%d bd_tgt=%d gyro_to=%d cb_diff=%d/%d da_diff=%d/%d ff_gain=%d gyro_st=%u arc_yaw=%u yaw_stop=%u b_exit=%d a_exit=%d report=%d\r\n",
        ctx->target_laps,
        ctx->yaw_cdeg,
        ctx->nav_ok,
        ctx->nav_frame_delta,
        ctx->nav_update_flags,
        TASK11_LINE_BASE_PWM,
        TASK11_ARC_BASE_PWM,
        TASK11_AC_HEADING_TARGET_CDEG,
        TASK11_BD_HEADING_TARGET_CDEG,
        TASK11_GYRO_TURN_TIMEOUT_MS,
        TASK11_CB_ARC_ENTRY_TARGET_DIFF,
        TASK11_CB_ARC_CRUISE_TARGET_DIFF,
        TASK11_DA_ARC_ENTRY_TARGET_DIFF,
        TASK11_DA_ARC_CRUISE_TARGET_DIFF,
        TASK11_DIFF_FF_GAIN,
        TASK11_STRAIGHT_GYRO_NAV_ENABLE,
        TASK11_ARC_YAW_NAV_ENABLE,
        TASK11_EXIT_TURN_YAW_STOP_ENABLE,
        TASK11_BD_HEADING_TARGET_CDEG,
        TASK11_AC_HEADING_TARGET_CDEG,
        TASK11_LINE_REPORT_PERIOD_MS);
}

static void race_finish_lap_context(race_context_t *ctx)
{
    TB6612_Brake();
    st011_finish_pending_pulse();
    encoder_reset_distance_counts();

    if (ctx->stop_reason == 0U) {
        ctx->stop_reason = (ctx->lap_count >= ctx->target_laps) ? 1U :
            ((ctx->elapsed_ms >= TASK11_TOTAL_MAX_RUN_MS) ? 4U : 2U);
    }
    race_ram_log_event(TASK11_RAM_EVENT_COMPLETE,
        ctx->stop_reason,
        ctx->lap_count,
        ctx->phase,
        0U,
        ctx->elapsed_ms,
        0,
        0,
        ctx->yaw_cdeg,
        0,
        0,
        0,
        0,
        0,
        ctx->gyro_z_filtered_mdps,
        ctx->ir_ok,
        &ctx->sample,
        0,
        0);
    race_event_printf("RACE_EVT_RT event=complete reason=%s t=%lu lap=%u phase=%u\r\n",
        race_reason_name(ctx->stop_reason),
        ctx->elapsed_ms,
        ctx->lap_count,
        ctx->phase);
    race_log_printf("RACE complete: ac_zero_collect reason=%s t=%lu lap=%u phase=%u\r\n",
        race_reason_name(ctx->stop_reason),
        ctx->elapsed_ms,
        ctx->lap_count,
        ctx->phase);
    race_ram_log_dump();
}

static uint8_t run_task2_cd_exit_angle_straight(const char *tag)
{
    const straight_line_segment_config_t config = {
        .tag = tag,
        .zero_heading = 0U,
        .start_alarm_ms = 0U,
        .stop_alarm_ms = 0U,
        .line_arm_count = 0,
        .force_stop_count = TASK11_STRAIGHT_FORCE_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .entry_brake_enable = 0U,
        .fixed_yaw_target_enable = 1U,
        .fixed_yaw_target_cdeg = TASK2_CD_STRAIGHT_TARGET_CDEG
    };

    return run_straight_to_line_segment(&config);
}

static void run_task10_ab_zero_test(void)
{
    uint32_t elapsed_ms = 0;
    int32_t yaw_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    uint8_t ok;
    uint8_t nav_ok;

    TB6612_Brake();
    ok = jy62_zero_to_current("TASK10_AB_ZERO", 0U);
    nav_ok = race_peek_yaw(&yaw_cdeg, &gyro_z_filtered_mdps);
    lc_printf("TASK10 yaw_monitor start: zero_ok=%u nav=%u rel=%ld gzlp=%ld period=200ms\r\n",
        ok,
        nav_ok,
        yaw_cdeg,
        gyro_z_filtered_mdps);

    while (task_uart_stop_requested() == 0U) {
        delay_ms_with_st011(200U);
        elapsed_ms += 200U;
        nav_ok = race_peek_yaw(&yaw_cdeg, &gyro_z_filtered_mdps);
        lc_printf("TASK10_YAW t=%lu nav=%u rel=%ld deg_x100=%ld gzlp=%ld\r\n",
            elapsed_ms,
            nav_ok,
            yaw_cdeg,
            yaw_cdeg,
            gyro_z_filtered_mdps);
    }

    lc_printf("TASK10 yaw_monitor stop: t=%lu\r\n", elapsed_ms);
}

static void run_race_laps(uint8_t target_laps)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;

    race_init_lap_context(&ctx, target_laps);

    while ((ctx.elapsed_ms < TASK11_TOTAL_MAX_RUN_MS) &&
        (ctx.lap_count < ctx.target_laps)) {
        race_configure_phase(&ctx, &phase_config);

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        ctx.elapsed_ms += CONTROL_PERIOD_MS;
        ctx.report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            ctx.stop_reason = 3U;
            break;
        }

        race_update_loop_state(&ctx, &phase_config);
        race_compute_loop_control(&ctx, &phase_config);
        race_log_loop_samples(&ctx, &phase_config);

        if (race_check_phase_point(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 1U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                TASK11_RAM_EVENT_POINT);

            if (race_execute_point_action(&ctx) == 0U) {
                ctx.stop_reason = 2U;
                break;
            }

            race_advance_segment(&ctx, 1U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        if (ctx.phase_distance_count >= phase_config.force_count) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                TASK11_RAM_EVENT_FORCE);

            race_advance_segment(&ctx, 0U);
            if (ctx.stop_reason != 0U) {
                break;
            }
            continue;
        }

        TB6612_SetDifferential((int16_t)ctx.left_pwm, (int16_t)ctx.right_pwm);
        race_log_periodic_data(&ctx, &phase_config);
    }

    race_finish_lap_context(&ctx);
}

#endif
