#ifndef ARC_SEGMENT_H
#define ARC_SEGMENT_H

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
    const char *tag;
    uint8_t post_exit_arc_enable;
    uint8_t point_alarm_enable;
    uint8_t ignore_wide_before_arc_arm;
} task2_race_arc_segment_config_t;

static uint8_t run_task2_race_arc_segment(
    const task2_race_arc_segment_config_t *config)
{
    const char *tag = config->tag;
    uint8_t post_exit_arc_enable = config->post_exit_arc_enable;
    uint8_t point_alarm_enable = config->point_alarm_enable;
    uint8_t ignore_wide_before_arc_arm =
        config->ignore_wide_before_arc_arm;
    straight_pid_t diff_pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint32_t nav_frame_delta = 0U;
    int32_t motor_b_delta = 0;
    int32_t motor_a_delta = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t filtered_error = 0;
    int32_t last_filtered_error = 0;
    int32_t last_turn = 0;
    int32_t yaw_start = 0;
    int32_t yaw_cdeg = 0;
    int32_t phase_yaw_cdeg = 0;
    int32_t yaw_progress_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    uint8_t nav_ok = 0U;
    uint8_t ir_ok = 0U;
    uint8_t exit_line_seen = 0U;
    uint8_t exit_point_seen = 0U;
    uint8_t stop_reason = 0U;

    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    race_diff_pid_reset(&diff_pid);
    nav_frame_delta = JY62_GetNavigation(&nav);
    nav_ok = nav.valid;
    if (nav_ok != 0U) {
        yaw_start = nav.yaw_relative_cdeg;
        yaw_cdeg = yaw_start;
        gyro_z_filtered_mdps = nav.gyro_z_filtered_mdps;
    }

    task2_ram_log_event(tag,
        TASK2_EVT_START,
        0U,
        0U,
        0,
        yaw_start,
        0,
        0U,
        &sample,
        nav_ok,
        0U,
        0,
        0);

    while (elapsed_ms < TASK2_ARC_MAX_RUN_MS) {
        int32_t distance_count;
        int32_t raw_error = 0;
        int32_t derivative = 0;
        int32_t base_pwm = TASK11_ARC_BASE_PWM;
        int32_t target_speed_diff;
        int32_t line_turn = 0;
        int32_t nav_turn = 0;
        int32_t control_turn;
        int32_t expected_yaw_cdeg;
        int32_t heading_error_cdeg = 0;
        int32_t left_pwm;
        int32_t right_pwm;
        uint8_t line_valid;
        uint8_t line_control_valid;
        uint8_t wide_line;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        nav_frame_delta = JY62_GetNavigation(&nav);
        nav_ok = nav.valid;
        if (nav_ok != 0U) {
            yaw_cdeg = nav.yaw_relative_cdeg;
            phase_yaw_cdeg = normalize_cdeg(yaw_cdeg - yaw_start);
            yaw_progress_cdeg = abs_i32(phase_yaw_cdeg);
            gyro_z_filtered_mdps = nav.gyro_z_filtered_mdps;
        } else {
            phase_yaw_cdeg = 0;
            yaw_progress_cdeg = 0;
            gyro_z_filtered_mdps = 0;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        line_valid = ((ir_ok != 0U) && (sample.line_lost == 0U)) ? 1U : 0U;
        wide_line = ((line_valid != 0U) &&
            (sample.active_count >= TASK3_ARC_WIDE_LINE_MIN_COUNT)) ? 1U : 0U;
        if ((line_valid != 0U) &&
            (yaw_progress_cdeg >= TASK11_ARC_POINT_YAW_ARM_CDEG)) {
            exit_line_seen = 1U;
        }
        if ((exit_point_seen == 0U) &&
            (exit_line_seen != 0U) &&
            ((line_valid == 0U) || (wide_line != 0U))) {
            exit_point_seen = 1U;
            if (point_alarm_enable != 0U) {
                st011_start_pulse(TASK2_POINT_ALARM_MS);
            }
            task2_ram_log_event(tag,
                TASK2_EVT_POINT,
                1U,
                elapsed_ms,
                distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                ir_ok,
                &sample,
                nav_ok,
                1U,
                motor_b_total,
                motor_a_total);
            race_log_printf("%s arc exit: t=%lu dist=%ld yaw=%ld yprog=%ld raw=0x%02X mask=0x%02X cnt=%u\r\n",
                tag,
                elapsed_ms,
                distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U);
            stop_reason = 1U;
            break;
        }

        target_speed_diff = (distance_count < TASK3_ARC_ENTRY_COUNT) ?
            TASK11_DA_ARC_ENTRY_TARGET_DIFF : TASK11_DA_ARC_CRUISE_TARGET_DIFF;
        line_control_valid = ((line_valid != 0U) && (exit_point_seen == 0U)) ? 1U : 0U;
        if ((ignore_wide_before_arc_arm != 0U) &&
            (exit_line_seen == 0U) &&
            (wide_line != 0U)) {
            line_control_valid = 0U;
        }

        if ((line_control_valid == 0U) && (exit_point_seen == 0U)) {
            base_pwm -= TASK11_LINE_LOST_BASE_DROP;
        }
        race_drive_config(&drive_config, base_pwm, target_speed_diff);
        straight_drive_update(&diff_pid,
            &drive_config,
            motor_b_delta,
            motor_a_delta,
            motor_b_total,
            motor_a_total,
            &drive);

        if (line_control_valid != 0U) {
            raw_error = sample.error;
            filtered_error += (raw_error - filtered_error) / TASK11_LINE_ERROR_FILTER_DIVISOR;
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -TASK11_LINE_DERIV_LIMIT,
                TASK11_LINE_DERIV_LIMIT);
            last_filtered_error = filtered_error;
            line_turn = (filtered_error / TASK11_LINE_TURN_DIVISOR) +
                (derivative / TASK11_LINE_KD_DIVISOR);
            line_turn = clamp_i32(line_turn,
                -TASK11_LINE_TURN_LIMIT,
                TASK11_LINE_TURN_LIMIT);
            last_turn = line_turn;
        } else if (last_turn != 0) {
            line_turn = clamp_i32(last_turn,
                -TASK11_LINE_LOST_TURN,
                TASK11_LINE_LOST_TURN);
        } else {
            line_turn = TASK3_ARC_TURN_RIGHT * TASK11_LINE_LOST_TURN;
        }

        expected_yaw_cdeg = race_arc_expected_yaw_cdeg(distance_count,
            TASK3_ARC_TURN_RIGHT);
        if (nav_ok != 0U) {
            heading_error_cdeg = normalize_cdeg(phase_yaw_cdeg - expected_yaw_cdeg);
            if (TASK11_ARC_YAW_NAV_ENABLE != 0) {
                nav_turn = race_heading_turn_from_error(heading_error_cdeg,
                    gyro_z_filtered_mdps,
                    TASK11_ARC_YAW_CORR_DIVISOR,
                    TASK11_ARC_GYRO_DAMP_DIVISOR,
                    TASK11_ARC_YAW_CORR_MAX);
            }
        }
        control_turn = clamp_i32(line_turn + nav_turn,
            -TASK11_LINE_TURN_LIMIT,
            TASK11_LINE_TURN_LIMIT);
        left_pwm = clamp_i32(drive.motor_b_pwm + control_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        right_pwm = clamp_i32(drive.motor_a_pwm - control_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (distance_count >= TASK2_ARC_FORCE_STOP_COUNT) {
            stop_reason = 2U;
            break;
        }

        if (report_elapsed_ms >= TASK2_ARC_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            task2_ram_log_sample(tag,
                elapsed_ms,
                distance_count,
                yaw_cdeg,
                phase_yaw_cdeg,
                yaw_progress_cdeg,
                expected_yaw_cdeg,
                heading_error_cdeg,
                gyro_z_filtered_mdps,
                ir_ok,
                &sample,
                nav_ok,
                exit_point_seen,
                motor_b_total,
                motor_a_total,
                &drive,
                drive.correction,
                line_turn,
                nav_turn,
                control_turn,
                target_speed_diff,
                left_pwm,
                right_pwm);
            race_log_printf("%s t=%lu dist=%ld cexit=%u seen=%u nav=%u yaw=%ld pyaw=%ld yprog=%ld exp=%ld herr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld filt=%ld der=%ld line_turn=%ld nav_turn=%ld turn=%ld tdiff=%ld ff=%ld fb=%ld corr=%ld pwm=%ld/%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                exit_point_seen,
                exit_line_seen,
                nav_ok,
                yaw_cdeg,
                phase_yaw_cdeg,
                yaw_progress_cdeg,
                expected_yaw_cdeg,
                heading_error_cdeg,
                gyro_z_filtered_mdps,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                filtered_error,
                derivative,
                line_turn,
                nav_turn,
                control_turn,
                target_speed_diff,
                drive.feedforward_correction,
                drive.feedback_correction,
                drive.correction,
                left_pwm,
                right_pwm);
        }
    }

    if ((post_exit_arc_enable != 0U) &&
        (stop_reason == 1U) &&
        (exit_point_seen != 0U)) {
        int32_t arc_turn_sign = (phase_yaw_cdeg <= 0) ? -1 : 1;
        uint8_t post_exit_ok;

        post_exit_ok = race_advance_after_point("TASK2_C_ADVANCE",
            TASK11_ARC_POINT_ADVANCE_COUNT);
        if (post_exit_ok != 0U) {
            post_exit_ok = task2_post_exit_arc_to_yaw("TASK2_C_ARC_170",
                yaw_start,
                arc_turn_sign);
        }
        if (post_exit_ok == 0U) {
            stop_reason = 4U;
        }
    }

    if (stop_reason != 1U) {
        TB6612_Brake();
    }
    if (stop_reason == 0U) {
        stop_reason = 4U;
    }
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    nav_ok = JY62_PeekNavigation(&nav);
    task2_ram_log_event(tag,
        TASK2_EVT_STOP,
        stop_reason,
        elapsed_ms,
        motion_distance_count(motor_b_total, motor_a_total),
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? abs_i32(normalize_cdeg(nav.yaw_relative_cdeg - yaw_start)) : 0,
        ir_ok,
        &sample,
        nav_ok,
        exit_point_seen,
        motor_b_total,
        motor_a_total);
    race_log_printf("%s stop: reason=%s t=%lu dist=%ld cexit=%u seen=%u yaw=%ld yprog=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u B_total=%ld A_total=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "post_exit_arc" : ((stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "uart_stop" : "timeout")),
        elapsed_ms,
        motion_distance_count(motor_b_total, motor_a_total),
        exit_point_seen,
        exit_line_seen,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? abs_i32(normalize_cdeg(nav.yaw_relative_cdeg - yaw_start)) : 0,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        motor_b_total,
        motor_a_total);

    return stop_reason;
}

static uint8_t run_task2_bc_race_arc_debug(const char *tag)
{
    const task2_race_arc_segment_config_t config = {
        .tag = tag,
        .post_exit_arc_enable = 1U,
        .point_alarm_enable = 1U,
        .ignore_wide_before_arc_arm = 0U
    };

    return run_task2_race_arc_segment(&config);
}

static uint8_t run_task2_da_race_arc(const char *tag)
{
    const task2_race_arc_segment_config_t config = {
        .tag = tag,
        .post_exit_arc_enable = 0U,
        .point_alarm_enable = 1U,
        .ignore_wide_before_arc_arm = 1U
    };

    return run_task2_race_arc_segment(&config);
}

#endif
