#ifndef RACE_PHASE_H
#define RACE_PHASE_H

/**
 * @file race_phase.h
 * @brief Race phase state update, control, point handling, and lap transitions.
 *
 * This header is included from race_laps.h after race_context_t and race_primitives.h are available.
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
 * @brief Read JY62 navigation and update yaw progress for the active phase.
 */
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

/**
 * @brief Fill the static AC/CB/BD/DA configuration for ctx->phase.
 */
static void race_configure_phase(const race_context_t *ctx,
    race_phase_config_t *config)
{
    uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;

    if (ctx->phase == 0U) {
        config->phase_name = "AC";
        config->point_name = "C_LINE";
        config->force_name = "C_FORCE";
        config->arc_mode = 0U;
        config->phase_turn_dir = 0;
        config->straight_target_cdeg = task4_mode ?
            RACE_TASK4_AC_HEADING_TARGET_CDEG :
            RACE_TASK3_AC_HEADING_TARGET_CDEG;
        config->point_arm_count = RACE_AC_POINT_ARM_COUNT;
        config->force_count = RACE_STRAIGHT_FORCE_COUNT;
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
        config->straight_target_cdeg = task4_mode ?
            RACE_TASK4_BD_HEADING_TARGET_CDEG :
            RACE_TASK3_BD_HEADING_TARGET_CDEG;
        config->point_arm_count = task4_mode ?
            RACE_TASK4_BD_POINT_ARM_COUNT : RACE_BD_POINT_ARM_COUNT;
        config->force_count = RACE_STRAIGHT_FORCE_COUNT;
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

/**
 * @brief Refresh sensors, encoders, phase distance, line flags, and yaw state.
 */
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
        RACE_LOG_FLAG_IR_OK : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->line_lost_seen != 0U) ?
        RACE_LOG_FLAG_LINE_LOST : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->edge_point_seen != 0U) ?
        RACE_LOG_FLAG_EDGE_SEEN : 0U);
    ctx->point_log_flags |= (uint8_t)((ctx->straight_line_seen_count != 0U) ?
        RACE_LOG_FLAG_GUIDE_SEEN : 0U);
    ctx->point_log_flags |= (uint8_t)((config->arc_mode != 0U) ?
        RACE_LOG_FLAG_ARC_MODE : 0U);
}

/**
 * @brief Quickly ramp a Task4 high-speed base down to the stable Task3 base.
 */
static int32_t race_task4_decel_base_pwm(int32_t fast_base_pwm,
    int32_t stable_base_pwm,
    int32_t phase_distance_count,
    int32_t decel_start_count,
    int32_t decel_ramp_count)
{
    int32_t decel_distance;
    int32_t pwm_drop;

    if ((fast_base_pwm <= stable_base_pwm) ||
        (phase_distance_count <= decel_start_count)) {
        return fast_base_pwm;
    }
    if (decel_ramp_count <= 0) {
        return stable_base_pwm;
    }

    decel_distance = phase_distance_count - decel_start_count;
    if (decel_distance >= decel_ramp_count) {
        return stable_base_pwm;
    }

    pwm_drop = ((fast_base_pwm - stable_base_pwm) * decel_distance) /
        decel_ramp_count;
    return fast_base_pwm - pwm_drop;
}

/**
 * @brief Ramp Task4's first AC launch from a gentler start to full speed.
 */
static int32_t race_task4_first_ac_ramp_base_pwm(int32_t full_base_pwm,
    int32_t phase_distance_count)
{
    int32_t ramp_count = RACE_TASK4_FIRST_AC_RAMP_COUNT;
    int32_t start_base_pwm = RACE_TASK4_FIRST_AC_RAMP_START_PWM;
    int32_t pwm_gain;

    if ((ramp_count <= 0) || (full_base_pwm <= start_base_pwm)) {
        return full_base_pwm;
    }
    if (phase_distance_count <= 0) {
        return start_base_pwm;
    }
    if (phase_distance_count >= ramp_count) {
        return full_base_pwm;
    }

    pwm_gain = ((full_base_pwm - start_base_pwm) * phase_distance_count) /
        ramp_count;
    return start_base_pwm + pwm_gain;
}

/**
 * @brief Calculate line turn, yaw turn, wheel-speed PID, and final PWM values.
 */
static void race_compute_loop_control(race_context_t *ctx,
    const race_phase_config_t *config)
{
    uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;

    ctx->raw_error = 0;
    ctx->derivative = 0;
    ctx->line_turn = 0;
    ctx->nav_turn = 0;
    ctx->control_turn = 0;
    ctx->heading_error_cdeg = 0;
    ctx->expected_yaw_cdeg = 0;
    ctx->arc_actual_yaw_cdeg = 0;

    if (config->arc_mode != 0U) {
        ctx->base_pwm = task4_mode ? RACE_TASK4_ARC_BASE_PWM :
            RACE_ARC_BASE_PWM;
        if (task4_mode != 0U) {
            ctx->base_pwm = race_task4_decel_base_pwm(ctx->base_pwm,
                RACE_ARC_BASE_PWM,
                ctx->phase_distance_count,
                RACE_TASK4_EXIT_DECEL_START_COUNT,
                RACE_TASK4_EXIT_DECEL_RAMP_COUNT);
        }
        if (ctx->phase == 1U) {
            ctx->target_speed_diff =
                (ctx->phase_distance_count < TASK3_ARC_ENTRY_COUNT) ?
                    RACE_CB_ARC_ENTRY_TARGET_DIFF :
                    RACE_CB_ARC_CRUISE_TARGET_DIFF;
        } else {
            ctx->target_speed_diff =
                (ctx->phase_distance_count < TASK3_ARC_ENTRY_COUNT) ?
                    RACE_DA_ARC_ENTRY_TARGET_DIFF :
                    RACE_DA_ARC_CRUISE_TARGET_DIFF;
        }
    } else {
        if (task4_mode != 0U) {
            ctx->base_pwm = RACE_TASK4_STRAIGHT_BASE_PWM;
            if ((ctx->lap_count == 0U) && (ctx->phase == 0U)) {
                ctx->base_pwm = race_task4_first_ac_ramp_base_pwm(
                    ctx->base_pwm,
                    ctx->phase_distance_count);
            }
            ctx->base_pwm = race_task4_decel_base_pwm(ctx->base_pwm,
                RACE_STRAIGHT_BASE_PWM,
                ctx->phase_distance_count,
                RACE_TASK4_ENTRY_DECEL_START_COUNT,
                RACE_TASK4_ENTRY_DECEL_RAMP_COUNT);
            ctx->target_speed_diff = RACE_TASK4_STRAIGHT_TARGET_DIFF;
        } else {
            ctx->base_pwm = RACE_STRAIGHT_BASE_PWM;
            ctx->target_speed_diff = RACE_STRAIGHT_TARGET_DIFF;
        }
    }

    if ((ctx->line_valid == 0U) &&
        ((config->arc_mode != 0U) || (RACE_STRAIGHT_GYRO_NAV_ENABLE == 0))) {
        ctx->base_pwm -= RACE_LINE_LOST_BASE_DROP;
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
            RACE_LINE_ERROR_FILTER_DIVISOR;
        ctx->derivative = clamp_i32(ctx->filtered_error - ctx->last_filtered_error,
            -RACE_LINE_DERIV_LIMIT,
            RACE_LINE_DERIV_LIMIT);
        ctx->last_filtered_error = ctx->filtered_error;
        ctx->line_turn = (ctx->filtered_error / RACE_LINE_TURN_DIVISOR) +
            (ctx->derivative / RACE_LINE_KD_DIVISOR);
        ctx->line_turn = clamp_i32(ctx->line_turn,
            -RACE_LINE_TURN_LIMIT,
            RACE_LINE_TURN_LIMIT);
        ctx->last_turn = ctx->line_turn;
    } else if (ctx->last_turn != 0) {
        ctx->line_turn = clamp_i32(ctx->last_turn,
            -RACE_LINE_LOST_TURN,
            RACE_LINE_LOST_TURN);
    } else {
        ctx->line_turn = config->phase_turn_dir * RACE_LINE_LOST_TURN;
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
            if (RACE_STRAIGHT_GYRO_NAV_ENABLE != 0) {
                ctx->nav_turn = race_heading_turn_from_error(
                    ctx->heading_error_cdeg,
                    ctx->gyro_z_filtered_mdps,
                    RACE_STRAIGHT_HEADING_CORR_DIVISOR,
                    RACE_STRAIGHT_GYRO_DAMP_DIVISOR,
                    RACE_STRAIGHT_HEADING_CORR_MAX);
            }
        } else {
            ctx->arc_actual_yaw_cdeg = ctx->phase_yaw_cdeg;
            ctx->heading_error_cdeg = normalize_cdeg(ctx->arc_actual_yaw_cdeg -
                ctx->expected_yaw_cdeg);
            if (RACE_ARC_YAW_NAV_ENABLE != 0) {
                ctx->nav_turn = race_heading_turn_from_error(
                    ctx->heading_error_cdeg,
                    ctx->gyro_z_filtered_mdps,
                    RACE_ARC_YAW_CORR_DIVISOR,
                    RACE_ARC_GYRO_DAMP_DIVISOR,
                    RACE_ARC_YAW_CORR_MAX);
            }
        }
    }

    if (config->arc_mode != 0U) {
        ctx->control_turn = clamp_i32(ctx->line_turn + ctx->nav_turn,
            -RACE_LINE_TURN_LIMIT,
            RACE_LINE_TURN_LIMIT);
    } else if ((ctx->nav_ok != 0U) && (RACE_STRAIGHT_GYRO_NAV_ENABLE != 0)) {
        ctx->control_turn = ctx->nav_turn;
#if RACE_STRAIGHT_IR_ASSIST_ENABLE
        if (ctx->line_valid != 0U) {
            ctx->control_turn = clamp_i32(ctx->control_turn +
                (ctx->line_turn / RACE_STRAIGHT_IR_ASSIST_DIVISOR),
                -RACE_LINE_TURN_LIMIT,
                RACE_LINE_TURN_LIMIT);
        }
#endif
    } else {
        ctx->control_turn = ctx->line_turn;
    }

    {
        int32_t max_pwm = (ctx->target_laps == TASK4_LAP_COUNT) ?
            RACE_TASK4_LINE_MAX_PWM : RACE_LINE_MAX_PWM;

        ctx->left_pwm = clamp_i32(ctx->drive.motor_b_pwm + ctx->control_turn,
        RACE_LINE_MIN_PWM,
            max_pwm);
        ctx->right_pwm = clamp_i32(ctx->drive.motor_a_pwm - ctx->control_turn,
        RACE_LINE_MIN_PWM,
            max_pwm);
    }
}

/**
 * @brief Decide whether the current phase reached its normal task point.
 */
static uint8_t race_check_phase_point(race_context_t *ctx,
    const race_phase_config_t *config)
{
    if (config->arc_mode == 0U) {
        ctx->straight_point_candidate =
            ((ctx->phase_distance_count >= config->point_arm_count) &&
             (ctx->line_valid != 0U)) ? 1U : 0U;
        if (ctx->straight_point_candidate != 0U) {
            if (ctx->straight_point_count < RACE_STRAIGHT_POINT_CONFIRM_COUNT) {
                ctx->straight_point_count++;
            }
        } else {
            ctx->straight_point_count = 0U;
        }
        ctx->point_ready = (ctx->straight_point_count >=
            RACE_STRAIGHT_POINT_CONFIRM_COUNT) ? 1U : 0U;
    } else if ((ctx->phase == 1U) || (ctx->phase == 3U)) {
        uint32_t phase_elapsed_ms = ctx->elapsed_ms - ctx->phase_start_ms;

        ctx->point_ready = ((phase_elapsed_ms >= RACE_ARC_EXIT_IGNORE_MS) &&
            (ctx->line_lost_seen != 0U)) ? 1U : 0U;
    } else {
        ctx->point_ready = ((ctx->phase_distance_count >= config->point_arm_count) &&
            (ctx->yaw_progress_cdeg >= RACE_ARC_POINT_YAW_ARM_CDEG) &&
            (ctx->edge_point_seen != 0U)) ? 1U : 0U;
    }

    return ctx->point_ready;
}

/**
 * @brief Decide whether an AC/BD straight segment should force its point turn.
 */
static uint8_t race_check_straight_force_turn(const race_context_t *ctx,
    const race_phase_config_t *config)
{
    if (config->arc_mode != 0U) {
        return 0U;
    }
    if (ctx->phase == 0U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        int32_t force_count = task4_mode ? RACE_TASK4_AC_FORCE_TURN_COUNT :
            RACE_TASK3_AC_FORCE_TURN_COUNT;

        if ((task4_mode != 0U) && (ctx->lap_count == 0U)) {
            force_count = RACE_TASK4_FIRST_AC_FORCE_TURN_COUNT;
        }

        return (ctx->phase_distance_count >= force_count) ? 1U : 0U;
    }
    if (ctx->phase == 2U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        int32_t force_count = task4_mode ? RACE_TASK4_BD_FORCE_TURN_COUNT :
            RACE_BD_FORCE_TURN_COUNT;

        return (ctx->phase_distance_count >= force_count) ? 1U : 0U;
    }
    return 0U;
}

/**
 * @brief Save high-rate race RAM samples around point windows.
 */
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

        if (ctx->phase_distance_count <= RACE_RAM_WINDOW_AFTER_START_COUNT) {
            window_log_flags |= RACE_LOG_FLAG_START_WINDOW;
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
            ctx->left_pwm,
            ctx->right_pwm,
            ctx->gyro_z_mdps,
            ctx->gyro_z_filtered_mdps,
            ctx->roll_cdeg,
            ctx->pitch_cdeg,
            ctx->nav_frame_delta,
            ctx->nav_update_flags);
    }
}

/**
 * @brief Record and optionally print the state captured at a phase point.
 */
static void race_log_point_state(const race_context_t *ctx,
    const race_phase_config_t *config,
    uint8_t reason,
    uint8_t event_type)
{
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
    st011_start_pulse(RACE_POINT_ALARM_MS);

    if (event_type == RACE_RAM_EVENT_POINT) {
        race_post_point_context_begin(ctx->elapsed_ms, ctx->phase_distance_count);
    }

    (void)config;
}

/**
 * @brief Copy current loop state into ctx->result with the supplied reason.
 */
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

/**
 * @brief Run the post-point action for AC, CB, BD, or DA.
 */
static uint8_t race_execute_point_action(const race_context_t *ctx)
{
    uint8_t turn_success = 1U;

    if (ctx->phase == 0U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        const sensor_fast_turn_config_t turn_config = {
            .tag = "RACE_C_LEFT_TURN",
            .motor_b_pwm = task4_mode ?
                RACE_TASK4_ENTRY_LEFT_TURN_B_PWM : RACE_LEFT_TURN_B_PWM,
            .motor_a_pwm = task4_mode ?
                RACE_TASK4_ENTRY_LEFT_TURN_A_PWM : RACE_LEFT_TURN_A_PWM,
            .slow_motor_b_pwm = task4_mode ?
                RACE_TASK4_ENTRY_LEFT_TURN_SLOW_B_PWM : RACE_LEFT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = task4_mode ?
                RACE_TASK4_ENTRY_LEFT_TURN_SLOW_A_PWM : RACE_LEFT_TURN_SLOW_A_PWM,
            .stop_mask = RACE_IR_CENTER_6_MASK,
            .forbid_mask = RACE_IR_CENTER_6_FORBID_MASK,
            .stop_error_max = RACE_TURN_CENTER6_ERROR_MAX,
            .yaw_stop_enable = 0U,
            .yaw_stop_target_cdeg = 0
        };
        turn_success = race_advance_after_point("RACE_C_ADVANCE",
            RACE_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_sensor_fast_turn(&turn_config);
        }
    } else if (ctx->phase == 1U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        int32_t target_cdeg = task4_mode ?
            RACE_TASK4_BD_HEADING_TARGET_CDEG :
            RACE_TASK3_BD_HEADING_TARGET_CDEG;
        const gyro_turn_config_t turn_config = {
            .tag = "RACE_B_GYRO_TO_BD",
            .motor_b_pwm = RACE_EXIT_LEFT_TURN_B_PWM,
            .motor_a_pwm = RACE_EXIT_LEFT_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_EXIT_LEFT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_EXIT_LEFT_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = target_cdeg,
            .predictive_stop_enable = task4_mode ?
                RACE_TASK4_EXIT_TURN_PREDICT_ENABLE : 0U,
            .predictive_stop_ms = task4_mode ?
                RACE_TASK4_EXIT_TURN_PREDICT_MS : 0,
            .predictive_stop_min_gz_mdps = task4_mode ?
                RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS : 0,
            .control_period_ms = task4_mode ?
                RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS
        };
        turn_success = task4_mode ?
            race_advance_after_point_with_heading("RACE_B_ADVANCE",
                RACE_ARC_POINT_ADVANCE_COUNT,
                RACE_TASK4_B_ADVANCE_HEADING_TARGET_CDEG) :
            race_advance_after_point("RACE_B_ADVANCE",
                RACE_ARC_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_gyro_turn_to_yaw(&turn_config);
        }
    } else if (ctx->phase == 2U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        const sensor_fast_turn_config_t turn_config = {
            .tag = "RACE_D_RIGHT_TURN",
            .motor_b_pwm = task4_mode ?
                RACE_TASK4_ENTRY_RIGHT_TURN_B_PWM : RACE_RIGHT_TURN_B_PWM,
            .motor_a_pwm = task4_mode ?
                RACE_TASK4_ENTRY_RIGHT_TURN_A_PWM : RACE_RIGHT_TURN_A_PWM,
            .slow_motor_b_pwm = task4_mode ?
                RACE_TASK4_ENTRY_RIGHT_TURN_SLOW_B_PWM : RACE_RIGHT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = task4_mode ?
                RACE_TASK4_ENTRY_RIGHT_TURN_SLOW_A_PWM : RACE_RIGHT_TURN_SLOW_A_PWM,
            .stop_mask = RACE_IR_CENTER_6_MASK,
            .forbid_mask = RACE_IR_CENTER_6_FORBID_MASK,
            .stop_error_max = RACE_TURN_CENTER6_ERROR_MAX,
            .yaw_stop_enable = 0U,
            .yaw_stop_target_cdeg = 0
        };
        turn_success = race_advance_after_point("RACE_D_ADVANCE",
            RACE_POINT_ADVANCE_COUNT);
        if (turn_success != 0U) {
            turn_success = race_sensor_fast_turn(&turn_config);
        }
    } else if ((uint8_t)(ctx->lap_count + 1U) < ctx->target_laps) {
        int32_t target_cdeg = RACE_TASK4_AC_HEADING_TARGET_CDEG;
        const gyro_turn_config_t turn_config = {
            .tag = "RACE_A_GYRO_TO_AC",
            .motor_b_pwm = RACE_EXIT_RIGHT_TURN_B_PWM,
            .motor_a_pwm = RACE_EXIT_RIGHT_TURN_A_PWM,
            .slow_motor_b_pwm = RACE_EXIT_RIGHT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = RACE_EXIT_RIGHT_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = target_cdeg,
            .predictive_stop_enable = RACE_TASK4_EXIT_TURN_PREDICT_ENABLE,
            .predictive_stop_ms = RACE_TASK4_EXIT_TURN_PREDICT_MS,
            .predictive_stop_min_gz_mdps =
                RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS,
            .control_period_ms = RACE_TASK4_CONTROL_PERIOD_MS
        };
        turn_success = race_advance_after_point_with_heading("RACE_A_ADVANCE",
            RACE_ARC_POINT_ADVANCE_COUNT,
            RACE_TASK4_A_ADVANCE_HEADING_TARGET_CDEG);
        if (turn_success != 0U) {
            turn_success = race_gyro_turn_to_yaw(&turn_config);
        }
    }

    return turn_success;
}

/**
 * @brief Force an AC/BD straight entry: turn toward the arc, then drive until line appears.
 */
static uint8_t race_execute_straight_force_turn_action(const race_context_t *ctx)
{
    int32_t yaw_cdeg = 0;
    int32_t gyro_z_filtered_mdps = 0;
    int32_t target_cdeg;
    uint8_t nav_ok;
    uint8_t turn_success;

    nav_ok = race_peek_yaw(&yaw_cdeg, &gyro_z_filtered_mdps);
    (void)gyro_z_filtered_mdps;
    if (nav_ok == 0U) {
        return 0U;
    }

    if (ctx->phase == 0U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        target_cdeg = normalize_cdeg(yaw_cdeg +
            RACE_FORCE_ENTRY_TURN_CDEG);
        const gyro_turn_config_t turn_config = {
            .tag = "RACE_C_FORCE_TURN",
            .motor_b_pwm = task4_mode ?
                RACE_TASK4_FORCE_LEFT_TURN_B_PWM : RACE_LEFT_TURN_B_PWM,
            .motor_a_pwm = task4_mode ?
                RACE_TASK4_FORCE_LEFT_TURN_A_PWM : RACE_LEFT_TURN_A_PWM,
            .slow_motor_b_pwm = task4_mode ?
                RACE_TASK4_FORCE_LEFT_TURN_SLOW_B_PWM : RACE_LEFT_TURN_SLOW_B_PWM,
            .slow_motor_a_pwm = task4_mode ?
                RACE_TASK4_FORCE_LEFT_TURN_SLOW_A_PWM : RACE_LEFT_TURN_SLOW_A_PWM,
            .yaw_stop_target_cdeg = target_cdeg,
            .control_period_ms = task4_mode ?
                RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS
        };

        turn_success = race_gyro_turn_to_yaw(&turn_config);
        if (turn_success != 0U) {
            turn_success = race_drive_forward_until_line("RACE_C_FORCE_FIND",
                RACE_FORCE_FIND_LINE_COUNT);
        }
        return turn_success;
    }

    if (ctx->phase == 2U) {
        uint8_t task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;
        target_cdeg = normalize_cdeg(yaw_cdeg -
            RACE_FORCE_ENTRY_TURN_CDEG);
        {
            const gyro_turn_config_t turn_config = {
                .tag = "RACE_D_FORCE_TURN",
                .motor_b_pwm = task4_mode ?
                    RACE_TASK4_FORCE_RIGHT_TURN_B_PWM : RACE_RIGHT_TURN_B_PWM,
                .motor_a_pwm = task4_mode ?
                    RACE_TASK4_FORCE_RIGHT_TURN_A_PWM : RACE_RIGHT_TURN_A_PWM,
                .slow_motor_b_pwm = task4_mode ?
                    RACE_TASK4_FORCE_RIGHT_TURN_SLOW_B_PWM : RACE_RIGHT_TURN_SLOW_B_PWM,
                .slow_motor_a_pwm = task4_mode ?
                    RACE_TASK4_FORCE_RIGHT_TURN_SLOW_A_PWM : RACE_RIGHT_TURN_SLOW_A_PWM,
                .yaw_stop_target_cdeg = target_cdeg,
                .control_period_ms = task4_mode ?
                    RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS
            };

            turn_success = race_gyro_turn_to_yaw(&turn_config);
            if (turn_success != 0U) {
                turn_success = race_drive_forward_until_line("RACE_D_FORCE_FIND",
                    RACE_FORCE_FIND_LINE_COUNT);
            }
            return turn_success;
        }
    }

    return 0U;
}

/**
 * @brief Reset encoder/PID/filter state for the next race phase.
 */
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

/**
 * @brief Move the race state machine to the next phase or lap.
 */
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

    ctx->phase_start_ms = ctx->elapsed_ms;

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
    race_log_segment_start_snapshot(ctx->target_laps,
        ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->nav_ok,
        ctx->yaw_cdeg,
        ctx->gyro_z_filtered_mdps);
}

/**
 * @brief Keep the loop call site stable after removing live UART reports.
 */
static void race_log_periodic_data(race_context_t *ctx,
    const race_phase_config_t *config)
{
    (void)ctx;
    (void)config;
}

/**
 * @brief Start action: AB is zero, turn right to the configured AC heading.
 */
static uint8_t race_align_start_to_ac(const char *tag,
    int16_t motor_b_pwm,
    int16_t motor_a_pwm,
    int16_t slow_motor_b_pwm,
    int16_t slow_motor_a_pwm,
    int32_t ac_target_cdeg,
    int32_t bd_target_cdeg,
    uint8_t predictive_stop_enable)
{
    const gyro_turn_config_t turn_config = {
        .tag = tag,
        .motor_b_pwm = motor_b_pwm,
        .motor_a_pwm = motor_a_pwm,
        .slow_motor_b_pwm = slow_motor_b_pwm,
        .slow_motor_a_pwm = slow_motor_a_pwm,
        .yaw_stop_target_cdeg = ac_target_cdeg,
        .predictive_stop_enable = predictive_stop_enable,
        .predictive_stop_ms = (predictive_stop_enable != 0U) ?
            RACE_TASK4_EXIT_TURN_PREDICT_MS : 0,
        .predictive_stop_min_gz_mdps = (predictive_stop_enable != 0U) ?
            RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS : 0,
        .control_period_ms = (predictive_stop_enable != 0U) ?
            RACE_TASK4_CONTROL_PERIOD_MS : CONTROL_PERIOD_MS
    };

    race_log_printf("%s_ALIGN target=%ld bd_target=%ld\r\n",
        tag,
        (long)ac_target_cdeg,
        (long)bd_target_cdeg);
    return race_gyro_turn_to_yaw(&turn_config);
}

/**
 * @brief Task3-only start action using its own tunable parameters.
 */
static uint8_t race_task3_align_start_to_ac(void)
{
#if RACE_TASK3_START_ALIGN_ENABLE
    return race_align_start_to_ac("RACE_TASK3_START_TO_AC",
        RACE_TASK3_START_RIGHT_TURN_B_PWM,
        RACE_TASK3_START_RIGHT_TURN_A_PWM,
        RACE_TASK3_START_RIGHT_TURN_SLOW_B_PWM,
        RACE_TASK3_START_RIGHT_TURN_SLOW_A_PWM,
        RACE_TASK3_AC_HEADING_TARGET_CDEG,
        RACE_TASK3_BD_HEADING_TARGET_CDEG,
        0U);
#else
    return 1U;
#endif
}

/**
 * @brief Task4-only start action using its own tunable parameters.
 */
static uint8_t race_task4_align_start_to_ac(void)
{
#if RACE_TASK4_START_ALIGN_ENABLE
    return race_align_start_to_ac("RACE_TASK4_START_TO_AC",
        RACE_TASK4_START_RIGHT_TURN_B_PWM,
        RACE_TASK4_START_RIGHT_TURN_A_PWM,
        RACE_TASK4_START_RIGHT_TURN_SLOW_B_PWM,
        RACE_TASK4_START_RIGHT_TURN_SLOW_A_PWM,
        RACE_TASK4_AC_HEADING_TARGET_CDEG,
        RACE_TASK4_BD_HEADING_TARGET_CDEG,
        RACE_TASK4_EXIT_TURN_PREDICT_ENABLE);
#else
    return 1U;
#endif
}

/**
 * @brief Initialize the race context before Task3/Task4 starts.
 */
static void race_init_lap_context(race_context_t *ctx, uint8_t target_laps)
{
    uint8_t task3_mode;
    uint8_t task4_mode;

    ctx->target_laps = (target_laps == 0U) ? 1U : target_laps;
    task3_mode = (ctx->target_laps == 1U) ? 1U : 0U;
    task4_mode = (ctx->target_laps == TASK4_LAP_COUNT) ? 1U : 0U;

    TB6612_Brake();
    delay_ms_with_st011(RACE_POINT_SETTLE_MS);
    {
        jy62_navigation_t zero_nav = {0};

        (void)JY62_GetNavigation(&zero_nav);
        if (zero_nav.valid != 0U) {
            JY62_SetYawZeroToCurrent();
            g_jy62_zero_ready = 1U;
        }
    }
    if ((task3_mode != 0U) || (task4_mode != 0U)) {
        delay_ms_with_st011(RACE_POINT_SETTLE_MS);
        uint8_t align_ok = (task4_mode != 0U) ?
            race_task4_align_start_to_ac() : race_task3_align_start_to_ac();

        if (align_ok == 0U) {
            ctx->stop_reason = 2U;
            return;
        }
    }
    delay_ms_with_st011(RACE_POINT_SETTLE_MS);
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
    race_ram_log_event(RACE_RAM_EVENT_START,
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
    race_log_segment_start_snapshot(ctx->target_laps,
        ctx->lap_count,
        ctx->phase,
        ctx->elapsed_ms,
        ctx->nav_ok,
        ctx->yaw_cdeg,
        ctx->gyro_z_filtered_mdps);

    {
        int32_t ac_target_cdeg = task4_mode ?
            RACE_TASK4_AC_HEADING_TARGET_CDEG :
            RACE_TASK3_AC_HEADING_TARGET_CDEG;
        int32_t bd_target_cdeg = task4_mode ?
            RACE_TASK4_BD_HEADING_TARGET_CDEG :
            RACE_TASK3_BD_HEADING_TARGET_CDEG;

        race_log_printf("RACE start: ac_zero_collect laps=%u yaw=%ld nav=%u nav_fd=%lu upd=0x%02X base=%d arc_base=%d ac_tgt=%ld bd_tgt=%ld gyro_to=%d cb_diff=%d/%d da_diff=%d/%d ff_gain=%d gyro_st=%u arc_yaw=%u yaw_stop=%u b_exit=%ld a_exit=%ld report=%d\r\n",
            ctx->target_laps,
            ctx->yaw_cdeg,
            ctx->nav_ok,
            ctx->nav_frame_delta,
            ctx->nav_update_flags,
            RACE_LINE_BASE_PWM,
            RACE_ARC_BASE_PWM,
            (long)ac_target_cdeg,
            (long)bd_target_cdeg,
            RACE_GYRO_TURN_TIMEOUT_MS,
            RACE_CB_ARC_ENTRY_TARGET_DIFF,
            RACE_CB_ARC_CRUISE_TARGET_DIFF,
            RACE_DA_ARC_ENTRY_TARGET_DIFF,
            RACE_DA_ARC_CRUISE_TARGET_DIFF,
            RACE_DIFF_FF_GAIN,
            RACE_STRAIGHT_GYRO_NAV_ENABLE,
            RACE_ARC_YAW_NAV_ENABLE,
            RACE_EXIT_TURN_YAW_STOP_ENABLE,
            (long)bd_target_cdeg,
            (long)ac_target_cdeg,
            RACE_LINE_REPORT_PERIOD_MS);
    }
}

/**
 * @brief Brake, determine final reason, log completion, and dump RAM logs.
 */
static void race_finish_lap_context(race_context_t *ctx)
{
    TB6612_Brake();
    st011_finish_pending_pulse();
    encoder_reset_distance_counts();

    if (ctx->stop_reason == 0U) {
        ctx->stop_reason = (ctx->lap_count >= ctx->target_laps) ? 1U :
            ((ctx->elapsed_ms >= RACE_TOTAL_MAX_RUN_MS) ? 4U : 2U);
    }
    race_ram_log_event(RACE_RAM_EVENT_COMPLETE,
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
    race_ram_log_dump(ctx->target_laps);
}

#endif /* RACE_PHASE_H */
