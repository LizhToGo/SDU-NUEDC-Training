#ifndef ARC_SEGMENT_H
#define ARC_SEGMENT_H

/**
 * @file arc_segment.h
 * @brief Task 2 race-style arc segment controller.
 *
 * Implements the current Task2 BC/DA arc behavior with line following,
 * differential drive, optional post-exit yaw alignment, and Task2 RAM logging.
 */

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

/**
 * @brief Configuration for one Task2 race arc segment.
 */
typedef struct {
    const char *tag;
    uint8_t post_exit_arc_enable;
    uint8_t point_alarm_enable;
    uint8_t ignore_wide_before_arc_arm;
} task2_race_arc_segment_config_t;

/**
 * @brief Snapshot captured when an arc exit point is detected.
 */
typedef struct {
    int32_t filtered_error;
    int32_t last_filtered_error;
    int32_t last_turn;
} task2_arc_control_memory_t;

/**
 * @brief Snapshot captured when an arc segment stops.
 */
typedef struct {
    const ir_tracking_sample_t *sample;
    uint8_t line_control_valid;
    uint8_t exit_point_seen;
    uint8_t nav_ok;
    int32_t distance_count;
    int32_t phase_yaw_cdeg;
    int32_t gyro_z_filtered_mdps;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
} task2_arc_control_input_t;

/**
 * @brief Calculated arc control values for one loop tick.
 */
typedef struct {
    int32_t raw_error;
    int32_t derivative;
    int32_t filtered_error;
    int32_t target_speed_diff;
    int32_t line_turn;
    int32_t nav_turn;
    int32_t control_turn;
    int32_t expected_yaw_cdeg;
    int32_t heading_error_cdeg;
    int32_t left_pwm;
    int32_t right_pwm;
} task2_arc_control_state_t;

/**
 * @brief Persistent IR line-filter memory for an arc segment.
 */
typedef struct {
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    uint8_t nav_ok;
    const ir_tracking_sample_t *sample;
    int32_t motor_b_total;
    int32_t motor_a_total;
} task2_arc_point_snapshot_t;

/**
 * @brief Report object for periodic arc debug output.
 */
typedef struct {
    task2_arc_point_snapshot_t point;
    uint8_t stop_reason;
    uint8_t exit_point_seen;
    uint8_t exit_line_seen;
} task2_arc_stop_snapshot_t;

/**
 * @brief Runtime counters and flags for one arc segment.
 */
typedef struct {
    const char *tag;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t phase_yaw_cdeg;
    int32_t yaw_progress_cdeg;
    int32_t gyro_z_filtered_mdps;
    uint8_t ir_ok;
    uint8_t nav_ok;
    uint8_t exit_point_seen;
    uint8_t exit_line_seen;
    const ir_tracking_sample_t *sample;
    int32_t motor_b_total;
    int32_t motor_a_total;
    const straight_drive_output_t *drive;
    const task2_arc_control_state_t *control;
} task2_arc_report_t;

/**
 * @brief Complete local state for run_task2_race_arc_segment().
 */
typedef struct {
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    uint32_t elapsed_ms;
    uint32_t report_elapsed_ms;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t distance_count;
    int32_t yaw_start;
    int32_t yaw_cdeg;
    int32_t phase_yaw_cdeg;
    int32_t yaw_progress_cdeg;
    int32_t gyro_z_filtered_mdps;
    uint8_t nav_ok;
    uint8_t ir_ok;
    uint8_t exit_line_seen;
    uint8_t exit_point_seen;
    uint8_t stop_reason;
} task2_arc_runtime_t;

/**
 * @brief Return whether an IR sample is usable for arc point/control logic.
 */
static uint8_t task2_arc_line_valid(uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    return ((ir_ok != 0U) && (sample->line_lost == 0U)) ? 1U : 0U;
}

/**
 * @brief Return whether the current line sample looks like a wide crossing.
 */
static uint8_t task2_arc_wide_line(uint8_t line_valid,
    const ir_tracking_sample_t *sample)
{
    return ((line_valid != 0U) &&
        (sample->active_count >= TASK3_ARC_WIDE_LINE_MIN_COUNT)) ? 1U : 0U;
}

/**
 * @brief Return whether line geometry is ready for arc exit detection.
 */
static uint8_t task2_arc_exit_line_ready(uint8_t line_valid,
    int32_t yaw_progress_cdeg)
{
    return ((line_valid != 0U) &&
        (yaw_progress_cdeg >= RACE_ARC_POINT_YAW_ARM_CDEG)) ? 1U : 0U;
}

/**
 * @brief Apply distance/yaw arming before accepting an arc exit point.
 */
static uint8_t task2_arc_exit_point_ready(uint8_t exit_point_seen,
    uint8_t exit_line_seen,
    uint8_t line_valid,
    uint8_t wide_line)
{
    return ((exit_point_seen == 0U) &&
        (exit_line_seen != 0U) &&
        ((line_valid == 0U) || (wide_line != 0U))) ? 1U : 0U;
}

/**
 * @brief Combine line and wide-line conditions into final arc stop readiness.
 */
static uint8_t task2_arc_exit_stop_ready(uint8_t *exit_line_seen,
    uint8_t exit_point_seen,
    uint8_t line_valid,
    uint8_t wide_line,
    int32_t yaw_progress_cdeg)
{
    if (task2_arc_exit_line_ready(line_valid, yaw_progress_cdeg) != 0U) {
        *exit_line_seen = 1U;
    }

    return task2_arc_exit_point_ready(exit_point_seen,
        *exit_line_seen,
        line_valid,
        wide_line);
}

/**
 * @brief Decide whether line error should drive arc steering this tick.
 */
static uint8_t task2_arc_line_control_valid(uint8_t line_valid,
    uint8_t exit_point_seen,
    uint8_t ignore_wide_before_arc_arm,
    uint8_t exit_line_seen,
    uint8_t wide_line)
{
    uint8_t control_valid = ((line_valid != 0U) &&
        (exit_point_seen == 0U)) ? 1U : 0U;

    if ((ignore_wide_before_arc_arm != 0U) &&
        (exit_line_seen == 0U) &&
        (wide_line != 0U)) {
        control_valid = 0U;
    }

    return control_valid;
}

/**
 * @brief Convert arc stop reason to log text.
 */
static const char *task2_arc_stop_reason_name(uint8_t stop_reason)
{
    if (stop_reason == 1U) {
        return "post_exit_arc";
    }
    if (stop_reason == 2U) {
        return "force";
    }
    if (stop_reason == 3U) {
        return "uart_stop";
    }
    return "timeout";
}

/**
 * @brief Log the moment an arc exit point is detected.
 */
static void task2_arc_log_exit_point(const char *tag,
    const task2_arc_point_snapshot_t *snapshot)
{
    task2_ram_log_event(tag,
        TASK2_EVT_POINT,
        1U,
        snapshot->elapsed_ms,
        snapshot->distance_count,
        snapshot->yaw_cdeg,
        snapshot->yaw_progress_cdeg,
        snapshot->ir_ok,
        snapshot->sample,
        snapshot->nav_ok,
        1U,
        snapshot->motor_b_total,
        snapshot->motor_a_total);
    race_log_printf("%s arc exit: t=%lu dist=%ld yaw=%ld yprog=%ld raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        snapshot->elapsed_ms,
        snapshot->distance_count,
        snapshot->yaw_cdeg,
        snapshot->yaw_progress_cdeg,
        (snapshot->ir_ok != 0U) ? snapshot->sample->raw : 0xFFU,
        (snapshot->ir_ok != 0U) ? snapshot->sample->line_mask : 0U,
        (snapshot->ir_ok != 0U) ? snapshot->sample->active_count : 0U);
}

/**
 * @brief Log final arc stop state.
 */
static void task2_arc_log_stop(const char *tag,
    const task2_arc_stop_snapshot_t *snapshot)
{
    race_log_printf("%s stop: reason=%s t=%lu dist=%ld cexit=%u seen=%u yaw=%ld yprog=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u B_total=%ld A_total=%ld\r\n",
        tag,
        task2_arc_stop_reason_name(snapshot->stop_reason),
        snapshot->point.elapsed_ms,
        snapshot->point.distance_count,
        snapshot->exit_point_seen,
        snapshot->exit_line_seen,
        snapshot->point.yaw_cdeg,
        snapshot->point.yaw_progress_cdeg,
        snapshot->point.ir_ok,
        (snapshot->point.ir_ok != 0U) ? snapshot->point.sample->raw : 0xFFU,
        (snapshot->point.ir_ok != 0U) ? snapshot->point.sample->line_mask : 0U,
        (snapshot->point.ir_ok != 0U) ? snapshot->point.sample->active_count : 0U,
        snapshot->point.motor_b_total,
        snapshot->point.motor_a_total);
}

/**
 * @brief Print one periodic arc diagnostic line.
 */
static void task2_arc_log_report(const task2_arc_report_t *report)
{
    task2_ram_log_sample(report->tag,
        report->elapsed_ms,
        report->distance_count,
        report->yaw_cdeg,
        report->phase_yaw_cdeg,
        report->yaw_progress_cdeg,
        report->control->expected_yaw_cdeg,
        report->control->heading_error_cdeg,
        report->gyro_z_filtered_mdps,
        report->ir_ok,
        report->sample,
        report->nav_ok,
        report->exit_point_seen,
        report->motor_b_total,
        report->motor_a_total,
        report->drive,
        report->drive->correction,
        report->control->line_turn,
        report->control->nav_turn,
        report->control->control_turn,
        report->control->target_speed_diff,
        report->control->left_pwm,
        report->control->right_pwm);
    race_log_printf("%s t=%lu dist=%ld cexit=%u seen=%u nav=%u yaw=%ld pyaw=%ld yprog=%ld exp=%ld herr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld filt=%ld der=%ld line_turn=%ld nav_turn=%ld turn=%ld tdiff=%ld ff=%ld fb=%ld corr=%ld pwm=%ld/%ld\r\n",
        report->tag,
        report->elapsed_ms,
        report->distance_count,
        report->exit_point_seen,
        report->exit_line_seen,
        report->nav_ok,
        report->yaw_cdeg,
        report->phase_yaw_cdeg,
        report->yaw_progress_cdeg,
        report->control->expected_yaw_cdeg,
        report->control->heading_error_cdeg,
        report->gyro_z_filtered_mdps,
        report->ir_ok,
        (report->ir_ok != 0U) ? report->sample->raw : 0xFFU,
        (report->ir_ok != 0U) ? report->sample->line_mask : 0U,
        (report->ir_ok != 0U) ? report->sample->active_count : 0U,
        (report->ir_ok != 0U) ? report->sample->line_lost : 1U,
        (report->ir_ok != 0U) ? report->sample->error : 0,
        report->control->filtered_error,
        report->control->derivative,
        report->control->line_turn,
        report->control->nav_turn,
        report->control->control_turn,
        report->control->target_speed_diff,
        report->drive->feedforward_correction,
        report->drive->feedback_correction,
        report->drive->correction,
        report->control->left_pwm,
        report->control->right_pwm);
}

/**
 * @brief Reset encoders, initialize yaw baseline, and start arc motor PWM.
 */
static void task2_arc_start(const char *tag,
    task2_arc_runtime_t *state,
    straight_pid_t *diff_pid)
{
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    race_diff_pid_reset(diff_pid);
    (void)JY62_GetNavigation(&state->nav);
    state->nav_ok = state->nav.valid;
    if (state->nav_ok != 0U) {
        state->yaw_start = state->nav.yaw_relative_cdeg;
        state->yaw_cdeg = state->yaw_start;
        state->gyro_z_filtered_mdps = state->nav.gyro_z_filtered_mdps;
    }

    task2_ram_log_event(tag,
        TASK2_EVT_START,
        0U,
        0U,
        0,
        state->yaw_start,
        0,
        0U,
        &state->sample,
        state->nav_ok,
        0U,
        0,
        0);
}

/**
 * @brief Advance arc elapsed/report timers by one control period.
 */
static void task2_arc_tick(task2_arc_runtime_t *state)
{
    delay_ms_with_st011(CONTROL_PERIOD_MS);
    state->elapsed_ms += CONTROL_PERIOD_MS;
    state->report_elapsed_ms += CONTROL_PERIOD_MS;
}

/**
 * @brief Read IR, encoder, and yaw state for the current arc tick.
 */
static void task2_arc_read_runtime(task2_arc_runtime_t *state)
{
    encoder_get_delta_counts(&state->motor_b_delta,
        &state->motor_a_delta);
    encoder_get_total_counts(&state->motor_b_total,
        &state->motor_a_total);
    state->distance_count = motion_distance_count(state->motor_b_total,
        state->motor_a_total);
    (void)JY62_GetNavigation(&state->nav);
    state->nav_ok = state->nav.valid;
    if (state->nav_ok != 0U) {
        state->yaw_cdeg = state->nav.yaw_relative_cdeg;
        state->phase_yaw_cdeg = normalize_cdeg(state->yaw_cdeg -
            state->yaw_start);
        state->yaw_progress_cdeg = abs_i32(state->phase_yaw_cdeg);
        state->gyro_z_filtered_mdps = state->nav.gyro_z_filtered_mdps;
    } else {
        state->phase_yaw_cdeg = 0;
        state->yaw_progress_cdeg = 0;
        state->gyro_z_filtered_mdps = 0;
    }

    state->ir_ok = IRTracking_ReadSample(&state->sample);
}

/**
 * @brief Copy current runtime state into an exit-point snapshot.
 */
static void task2_arc_fill_point_snapshot(
    task2_arc_point_snapshot_t *snapshot,
    const task2_arc_runtime_t *state)
{
    snapshot->elapsed_ms = state->elapsed_ms;
    snapshot->distance_count = state->distance_count;
    snapshot->yaw_cdeg = state->yaw_cdeg;
    snapshot->yaw_progress_cdeg = state->yaw_progress_cdeg;
    snapshot->ir_ok = state->ir_ok;
    snapshot->nav_ok = state->nav_ok;
    snapshot->sample = &state->sample;
    snapshot->motor_b_total = state->motor_b_total;
    snapshot->motor_a_total = state->motor_a_total;
}

/**
 * @brief Populate a periodic arc report from runtime/control state.
 */
static void task2_arc_fill_report(task2_arc_report_t *report,
    const char *tag,
    const task2_arc_runtime_t *state,
    const straight_drive_output_t *drive,
    const task2_arc_control_state_t *control)
{
    report->tag = tag;
    report->elapsed_ms = state->elapsed_ms;
    report->distance_count = state->distance_count;
    report->yaw_cdeg = state->yaw_cdeg;
    report->phase_yaw_cdeg = state->phase_yaw_cdeg;
    report->yaw_progress_cdeg = state->yaw_progress_cdeg;
    report->gyro_z_filtered_mdps = state->gyro_z_filtered_mdps;
    report->ir_ok = state->ir_ok;
    report->nav_ok = state->nav_ok;
    report->exit_point_seen = state->exit_point_seen;
    report->exit_line_seen = state->exit_line_seen;
    report->sample = &state->sample;
    report->motor_b_total = state->motor_b_total;
    report->motor_a_total = state->motor_a_total;
    report->drive = drive;
    report->control = control;
}

/**
 * @brief Capture final encoder, IR, and yaw values before stop logging.
 */
static void task2_arc_refresh_stop_state(task2_arc_runtime_t *state)
{
    encoder_get_total_counts(&state->motor_b_total,
        &state->motor_a_total);
    state->distance_count = motion_distance_count(state->motor_b_total,
        state->motor_a_total);
    state->nav_ok = JY62_PeekNavigation(&state->nav);
}

/**
 * @brief Copy final runtime state into an arc stop snapshot.
 */
static void task2_arc_fill_stop_snapshot(
    task2_arc_stop_snapshot_t *snapshot,
    const task2_arc_runtime_t *state)
{
    snapshot->stop_reason = state->stop_reason;
    snapshot->exit_point_seen = state->exit_point_seen;
    snapshot->exit_line_seen = state->exit_line_seen;
    snapshot->point.elapsed_ms = state->elapsed_ms;
    snapshot->point.distance_count = state->distance_count;
    snapshot->point.yaw_cdeg = (state->nav_ok != 0U) ?
        state->nav.yaw_relative_cdeg : 0;
    snapshot->point.yaw_progress_cdeg = (state->nav_ok != 0U) ?
        abs_i32(normalize_cdeg(state->nav.yaw_relative_cdeg -
            state->yaw_start)) : 0;
    snapshot->point.ir_ok = state->ir_ok;
    snapshot->point.nav_ok = state->nav_ok;
    snapshot->point.sample = &state->sample;
    snapshot->point.motor_b_total = state->motor_b_total;
    snapshot->point.motor_a_total = state->motor_a_total;
}

/**
 * @brief Calculate target speed difference, line turn, yaw turn, and PWM.
 */
static void task2_arc_update_control(
    task2_arc_control_state_t *control,
    task2_arc_control_memory_t *memory,
    straight_pid_t *diff_pid,
    straight_drive_config_t *drive_config,
    straight_drive_output_t *drive,
    const task2_arc_control_input_t *input)
{
    int32_t base_pwm = RACE_ARC_BASE_PWM;

    control->raw_error = 0;
    control->derivative = 0;
    control->line_turn = 0;
    control->nav_turn = 0;
    control->heading_error_cdeg = 0;
    control->target_speed_diff =
        (input->distance_count < TASK3_ARC_ENTRY_COUNT) ?
        RACE_DA_ARC_ENTRY_TARGET_DIFF : RACE_DA_ARC_CRUISE_TARGET_DIFF;

    if ((input->line_control_valid == 0U) &&
        (input->exit_point_seen == 0U)) {
        base_pwm -= RACE_LINE_LOST_BASE_DROP;
    }
    race_drive_config(drive_config, base_pwm, control->target_speed_diff);
    straight_drive_update(diff_pid,
        drive_config,
        input->motor_b_delta,
        input->motor_a_delta,
        input->motor_b_total,
        input->motor_a_total,
        drive);

    if (input->line_control_valid != 0U) {
        control->raw_error = input->sample->error;
        memory->filtered_error += (control->raw_error -
            memory->filtered_error) / RACE_LINE_ERROR_FILTER_DIVISOR;
        control->derivative = clamp_i32(memory->filtered_error -
                memory->last_filtered_error,
            -RACE_LINE_DERIV_LIMIT,
            RACE_LINE_DERIV_LIMIT);
        memory->last_filtered_error = memory->filtered_error;
        control->line_turn = (memory->filtered_error /
                RACE_LINE_TURN_DIVISOR) +
            (control->derivative / RACE_LINE_KD_DIVISOR);
        control->line_turn = clamp_i32(control->line_turn,
            -RACE_LINE_TURN_LIMIT,
            RACE_LINE_TURN_LIMIT);
        memory->last_turn = control->line_turn;
    } else if (memory->last_turn != 0) {
        control->line_turn = clamp_i32(memory->last_turn,
            -RACE_LINE_LOST_TURN,
            RACE_LINE_LOST_TURN);
    } else {
        control->line_turn = TASK3_ARC_TURN_RIGHT * RACE_LINE_LOST_TURN;
    }

    control->filtered_error = memory->filtered_error;
    control->expected_yaw_cdeg = race_arc_expected_yaw_cdeg(
        input->distance_count,
        TASK3_ARC_TURN_RIGHT);
    if (input->nav_ok != 0U) {
        control->heading_error_cdeg = normalize_cdeg(
            input->phase_yaw_cdeg - control->expected_yaw_cdeg);
        if (RACE_ARC_YAW_NAV_ENABLE != 0) {
            control->nav_turn = race_heading_turn_from_error(
                control->heading_error_cdeg,
                input->gyro_z_filtered_mdps,
                RACE_ARC_YAW_CORR_DIVISOR,
                RACE_ARC_GYRO_DAMP_DIVISOR,
                RACE_ARC_YAW_CORR_MAX);
        }
    }
    control->control_turn = clamp_i32(control->line_turn + control->nav_turn,
        -RACE_LINE_TURN_LIMIT,
        RACE_LINE_TURN_LIMIT);
    control->left_pwm = clamp_i32(drive->motor_b_pwm + control->control_turn,
        RACE_LINE_MIN_PWM,
        RACE_LINE_MAX_PWM);
    control->right_pwm = clamp_i32(drive->motor_a_pwm - control->control_turn,
        RACE_LINE_MIN_PWM,
        RACE_LINE_MAX_PWM);
}

/**
 * @brief Execute one Task2 arc segment until exit line, force limit, stop, or timeout.
 */
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
    task2_arc_runtime_t state = {0};
    task2_arc_control_memory_t control_memory = {0};

    task2_arc_start(tag, &state, &diff_pid);

    while (state.elapsed_ms < TASK2_ARC_MAX_RUN_MS) {
        uint8_t line_valid;
        uint8_t line_control_valid;
        uint8_t wide_line;
        task2_arc_control_input_t control_input;
        task2_arc_control_state_t control;

        task2_arc_tick(&state);

        if (task_uart_stop_requested() != 0U) {
            state.stop_reason = 3U;
            break;
        }

        task2_arc_read_runtime(&state);
        line_valid = task2_arc_line_valid(state.ir_ok, &state.sample);
        wide_line = task2_arc_wide_line(line_valid, &state.sample);
        if (task2_arc_exit_stop_ready(&state.exit_line_seen,
            state.exit_point_seen,
            line_valid,
            wide_line,
            state.yaw_progress_cdeg) != 0U) {
            task2_arc_point_snapshot_t point_snapshot;

            state.exit_point_seen = 1U;
            if (point_alarm_enable != 0U) {
                st011_start_pulse(TASK2_POINT_ALARM_MS);
            }
            task2_arc_fill_point_snapshot(&point_snapshot, &state);
            task2_arc_log_exit_point(tag, &point_snapshot);
            state.stop_reason = 1U;
            break;
        }

        line_control_valid = task2_arc_line_control_valid(line_valid,
            state.exit_point_seen,
            ignore_wide_before_arc_arm,
            state.exit_line_seen,
            wide_line);

        control_input.sample = &state.sample;
        control_input.line_control_valid = line_control_valid;
        control_input.exit_point_seen = state.exit_point_seen;
        control_input.nav_ok = state.nav_ok;
        control_input.distance_count = state.distance_count;
        control_input.phase_yaw_cdeg = state.phase_yaw_cdeg;
        control_input.gyro_z_filtered_mdps = state.gyro_z_filtered_mdps;
        control_input.motor_b_delta = state.motor_b_delta;
        control_input.motor_a_delta = state.motor_a_delta;
        control_input.motor_b_total = state.motor_b_total;
        control_input.motor_a_total = state.motor_a_total;
        task2_arc_update_control(&control,
            &control_memory,
            &diff_pid,
            &drive_config,
            &drive,
            &control_input);
        TB6612_SetDifferential((int16_t)control.left_pwm,
            (int16_t)control.right_pwm);

        if (state.distance_count >= TASK2_ARC_FORCE_STOP_COUNT) {
            state.stop_reason = 2U;
            break;
        }

        if (state.report_elapsed_ms >= TASK2_ARC_REPORT_PERIOD_MS) {
            task2_arc_report_t report;

            state.report_elapsed_ms = 0;
            task2_arc_fill_report(&report, tag, &state, &drive, &control);
            task2_arc_log_report(&report);
        }
    }

    if ((post_exit_arc_enable != 0U) &&
        (state.stop_reason == 1U) &&
        (state.exit_point_seen != 0U)) {
        int32_t arc_turn_sign = (state.phase_yaw_cdeg <= 0) ? -1 : 1;
        uint8_t post_exit_ok;

        post_exit_ok = race_advance_after_point("TASK2_C_ADVANCE",
            RACE_ARC_POINT_ADVANCE_COUNT);
        if (post_exit_ok != 0U) {
            post_exit_ok = task2_post_exit_arc_to_yaw("TASK2_C_ARC_170",
                state.yaw_start,
                arc_turn_sign);
        }
        if (post_exit_ok == 0U) {
            state.stop_reason = 4U;
        }
    }

    if (state.stop_reason != 1U) {
        TB6612_Brake();
    }
    if (state.stop_reason == 0U) {
        state.stop_reason = 4U;
    }
    task2_arc_refresh_stop_state(&state);
    {
        task2_arc_stop_snapshot_t stop_snapshot;

        task2_arc_fill_stop_snapshot(&stop_snapshot, &state);

        task2_ram_log_event(tag,
            TASK2_EVT_STOP,
            stop_snapshot.stop_reason,
            stop_snapshot.point.elapsed_ms,
            stop_snapshot.point.distance_count,
            stop_snapshot.point.yaw_cdeg,
            stop_snapshot.point.yaw_progress_cdeg,
            stop_snapshot.point.ir_ok,
            stop_snapshot.point.sample,
            stop_snapshot.point.nav_ok,
            stop_snapshot.exit_point_seen,
            stop_snapshot.point.motor_b_total,
            stop_snapshot.point.motor_a_total);
        task2_arc_log_stop(tag, &stop_snapshot);
    }

    return state.stop_reason;
}

/**
 * @brief Convenience wrapper for Task2 BC arc configuration.
 */
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

/**
 * @brief Convenience wrapper for Task2 DA arc configuration.
 */
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
