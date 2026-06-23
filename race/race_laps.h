#ifndef RACE_LAPS_H
#define RACE_LAPS_H

/**
 * @file race_laps.h
 * @brief Top-level task 3/4 race lap orchestration.
 *
 * This header is included from main.c after the local task config types and
 * helper hooks are declared. The lower-level race actions live in
 * race_primitives.h and per-segment state logic lives in race_phase.h.
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
 * @brief Snapshot captured when a race phase ends at a point or force limit.
 */
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

/**
 * @brief Static parameters for the currently active AC/CB/BD/DA phase.
 */
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

/**
 * @brief Full mutable race runtime state shared by phase helpers.
 */
typedef struct {
    line_result_t result;
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    straight_pid_t diff_pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms;
    uint32_t phase_start_ms;
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

/**
 * @brief Convert internal stop/point reason codes to log text.
 */
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

/**
 * @brief Convert phase index 0..3 to AC/CB/BD/DA label.
 */
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
#include "race_primitives.h"
#include "race_phase.h"

/**
 * @brief Task2-specific CD straight segment after exiting the BC arc.
 */
static uint8_t run_task2_cd_exit_angle_straight(const char *tag)
{
    const straight_line_segment_config_t config = {
        .tag = tag,
        .zero_heading = 0U,
        .start_alarm_ms = 0U,
        .stop_alarm_ms = 0U,
        .line_arm_count = 0,
        .force_stop_count = RACE_STRAIGHT_FORCE_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .entry_brake_enable = 0U,
        .fixed_yaw_target_enable = 1U,
        .fixed_yaw_target_cdeg = TASK2_CD_STRAIGHT_TARGET_CDEG
    };

    return run_straight_to_line_segment(&config);
}

/**
 * @brief Execute Task3/Task4 race laps using the shared AC/CB/BD/DA phase loop.
 */
static void run_race_laps(uint8_t target_laps)
{
    race_context_t ctx = {0};
    race_phase_config_t phase_config;
    uint32_t control_period_ms;

    race_init_lap_context(&ctx, target_laps);
    if (ctx.stop_reason != 0U) {
        race_finish_lap_context(&ctx);
        return;
    }
    control_period_ms = (ctx.target_laps == TASK4_LAP_COUNT) ?
        RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS;

    while ((ctx.elapsed_ms < RACE_TOTAL_MAX_RUN_MS) &&
        (ctx.lap_count < ctx.target_laps)) {
        race_configure_phase(&ctx, &phase_config);

        delay_ms_with_st011(control_period_ms);
        ctx.elapsed_ms += control_period_ms;
        ctx.report_elapsed_ms += control_period_ms;

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
                RACE_RAM_EVENT_POINT);

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

        if (race_check_straight_force_turn(&ctx, &phase_config) != 0U) {
            race_capture_result(&ctx, 2U);
            race_log_point_state(&ctx,
                &phase_config,
                ctx.result.reason,
                RACE_RAM_EVENT_FORCE);

            if (race_execute_straight_force_turn_action(&ctx) == 0U) {
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
                RACE_RAM_EVENT_FORCE);

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
