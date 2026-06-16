#ifndef HEADING_STRAIGHT_H
#define HEADING_STRAIGHT_H

/**
 * @file heading_straight.h
 * @brief Heading-assisted straight segment controller.
 *
 * Provides the shared straight-to-line routine used by task 1/2 and the
 * straight portions of task 3/4. It combines encoder speed balance, distance
 * balance, JY62 heading correction, and optional end-of-line search behavior.
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
#include "turn/line_fast_turn.h"

/**
 * @brief Static configuration for one heading-assisted straight segment.
 */
typedef struct {
    const char *tag;
    uint8_t zero_heading;
    int32_t heading_target_cdeg;
    uint8_t heading_only;
    uint8_t fast_correction;
    uint8_t line_search_protect;
    uint32_t start_alarm_ms;
    uint32_t stop_alarm_ms;
} heading_straight_segment_config_t;

/**
 * @brief Per-mode correction limits for heading and distance control.
 */
typedef struct {
    int32_t heading_corr_divisor;
    int32_t heading_corr_max;
    int32_t distance_corr_divisor;
    int32_t distance_corr_max;
} heading_straight_control_limits_t;

/**
 * @brief Planned IR line arming/search settings for a straight segment.
 */
typedef struct {
    int32_t line_arm_count;
    int32_t force_stop_count;
    int32_t search_start_count;
    int32_t search_sweep_start_count;
    int32_t search_sweep_period_ms;
} heading_straight_line_plan_t;

/**
 * @brief Current heading correction terms from JY62 and filtering.
 */
typedef struct {
    int32_t raw;
    int32_t raw_error;
    int32_t error;
    int32_t filtered;
    int32_t gain;
    int32_t gyro_z;
    int32_t correction;
    uint8_t wobble;
    uint8_t task4_ac_start_boost;
} heading_straight_nav_correction_t;

/**
 * @brief Current IR stop/search state for the straight segment.
 */
typedef struct {
    uint8_t ir_armed;
    uint8_t stop_mask;
    uint8_t line_centered;
    uint8_t line_stop_ready;
    uint8_t search_mode;
    int32_t stop_error_max;
    int32_t search_start_count;
    int32_t search_sweep_start_count;
    int32_t search_corr_divisor;
    int32_t search_corr_max;
    int32_t search_soft_corr;
    int32_t search_sweep_corr;
    int32_t search_base_drop;
    int32_t correction_limit;
} heading_straight_line_state_t;

/**
 * @brief Calculated drive and balance terms for one loop tick.
 */
typedef struct {
    int32_t motor_b_speed;
    int32_t motor_a_speed;
    int32_t speed_error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t speed_correction;
    int32_t distance_error;
    int32_t distance_correction;
    int32_t balance_correction;
    int32_t search_correction;
    int32_t correction;
    int32_t base_b_pwm;
    int32_t base_a_pwm;
    int32_t motor_b_pwm;
    int32_t motor_a_pwm;
    uint8_t heading_priority;
} heading_straight_drive_state_t;

/**
 * @brief Inputs for heading_straight_update_balance_control().
 */
typedef struct {
    const heading_straight_segment_config_t *config;
    const heading_straight_line_state_t *line_state;
    const heading_straight_nav_correction_t *heading;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
} heading_straight_balance_input_t;

/**
 * @brief Inputs for endpoint search PWM override.
 */
typedef struct {
    const heading_straight_segment_config_t *config;
    const heading_straight_line_state_t *line_state;
    const ir_tracking_sample_t *sample;
    uint8_t ir_ok;
    uint32_t elapsed_ms;
    int32_t distance_count;
} heading_straight_search_input_t;

/**
 * @brief Periodic report data for heading straight segments.
 */
typedef struct {
    const heading_straight_segment_config_t *config;
    const heading_straight_line_plan_t *line_plan;
    const heading_straight_line_state_t *line_state;
    const heading_straight_nav_correction_t *heading;
    const heading_straight_drive_state_t *drive;
    const ir_tracking_sample_t *sample;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t motor_b_total;
    int32_t motor_a_total;
    uint8_t ir_ok;
    uint8_t nav_ok;
} heading_straight_report_t;

/**
 * @brief Select correction limits based on heading-only and fast-correction modes.
 */
static heading_straight_control_limits_t heading_straight_control_limits(
    uint8_t fast_correction,
    uint8_t heading_only,
    int32_t heading_target_cdeg,
    uint8_t task4_ac_start_boost)
{
    heading_straight_control_limits_t limits;

    if (task4_ac_start_boost != 0U) {
        limits.heading_corr_divisor = TASK4_AC_START_HEADING_CORR_DIVISOR;
        limits.heading_corr_max = TASK4_AC_START_HEADING_CORR_MAX;
    } else if (fast_correction != 0U) {
        limits.heading_corr_divisor = TASK2_AB_HEADING_CORR_DIVISOR;
        limits.heading_corr_max = TASK2_AB_HEADING_CORR_MAX;
    } else if (heading_only != 0U) {
        limits.heading_corr_divisor = TASK3_BD_HEADING_CORR_DIVISOR;
        limits.heading_corr_max = TASK3_BD_HEADING_CORR_MAX;
    } else if (heading_target_cdeg != 0) {
        limits.heading_corr_divisor = TASK2_CD_HEADING_CORR_DIVISOR;
        limits.heading_corr_max = TASK2_CD_HEADING_CORR_MAX;
    } else {
        limits.heading_corr_divisor = TASK1_HEADING_CORR_DIVISOR;
        limits.heading_corr_max = TASK1_HEADING_CORR_MAX;
    }

    limits.distance_corr_divisor = (fast_correction != 0U) ?
        TASK2_AB_DISTANCE_CORR_DIVISOR : TASK1_DISTANCE_CORR_DIVISOR;
    limits.distance_corr_max = (fast_correction != 0U) ?
        TASK2_AB_DISTANCE_CORR_MAX : TASK1_DISTANCE_CORR_MAX;

    return limits;
}

/**
 * @brief Build the line arming/search plan for a straight segment.
 */
static heading_straight_line_plan_t heading_straight_line_plan(
    uint8_t line_search_protect)
{
    heading_straight_line_plan_t plan;

    if (line_search_protect >= 2U) {
        plan.line_arm_count = TASK3_STRAIGHT_LINE_ARM_COUNT;
        plan.force_stop_count = TASK3_STRAIGHT_FORCE_STOP_COUNT;
        plan.search_start_count = TASK3_STRAIGHT_SEARCH_START_COUNT;
        plan.search_sweep_start_count = TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        plan.search_sweep_period_ms = TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS;
    } else if (line_search_protect != 0U) {
        plan.line_arm_count = TASK1_B_LINE_ARM_COUNT;
        plan.force_stop_count = TASK1_FORCE_STOP_COUNT;
        plan.search_start_count = TASK2_STRAIGHT_SEARCH_START_COUNT;
        plan.search_sweep_start_count = TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        plan.search_sweep_period_ms = 0;
    } else {
        plan.line_arm_count = TASK1_B_LINE_ARM_COUNT;
        plan.force_stop_count = TASK1_FORCE_STOP_COUNT;
        plan.search_start_count = 0;
        plan.search_sweep_start_count = 0;
        plan.search_sweep_period_ms = 0;
    }

    return plan;
}

/**
 * @brief Update the JY62 heading correction used by straight segments.
 */
/**
 * @brief Read JY62 and calculate heading correction for the current tick.
 */
static void heading_straight_update_nav_correction(
    const heading_straight_segment_config_t *config,
    int32_t distance_count,
    uint8_t nav_ok,
    const jy62_navigation_t *nav,
    heading_filter_t *filter,
    heading_straight_nav_correction_t *out)
{
    heading_straight_control_limits_t limits;

    out->raw = (nav_ok != 0U) ? nav->yaw_relative_cdeg : 0;
    out->gyro_z = (nav_ok != 0U) ? nav->gyro_z_filtered_mdps : 0;
    out->raw_error = (nav_ok != 0U) ?
        normalize_cdeg(out->raw - config->heading_target_cdeg) : 0;
    out->task4_ac_start_boost = ((config->heading_only != 0U) &&
        (nav_ok != 0U) &&
        (task4_ac_debug_enabled(config->tag) != 0U) &&
        (config->tag[7] != '1') &&
        (distance_count < TASK4_AC_START_TURN_COUNT) &&
        (abs_i32(out->raw_error) >= TASK4_AC_START_BOOST_MIN_ERR_CDEG)) ? 1U : 0U;
    out->filtered = 0;
    out->gain = 0;
    out->wobble = 0U;
    out->error = (nav_ok != 0U) ?
        heading_filter_update(filter,
            out->raw_error,
            out->gyro_z,
            &out->filtered,
            &out->gain,
            &out->wobble) : 0;

    if ((config->heading_target_cdeg != 0) || (config->heading_only != 0U)) {
        out->error = out->raw_error;
    } else if ((config->fast_correction != 0U) && (nav_ok != 0U)) {
        out->error = (abs_i32(out->raw_error) < TASK2_AB_HEADING_DEADBAND_CDEG) ?
            0 : out->raw_error;
    }

    limits = heading_straight_control_limits(config->fast_correction,
        config->heading_only,
        config->heading_target_cdeg,
        out->task4_ac_start_boost);
    out->correction = (out->error * TASK1_HEADING_CORR_SIGN) /
        limits.heading_corr_divisor;
    if ((config->heading_target_cdeg != 0) || (config->heading_only != 0U)) {
        out->correction -= out->gyro_z / TASK2_CD_HEADING_GYRO_DAMP_DIVISOR;
    }
    out->correction = clamp_i32(out->correction,
        -limits.heading_corr_max,
        limits.heading_corr_max);
}

/**
 * @brief Derive line-arm, stop, and search state from the latest IR sample.
 */
/**
 * @brief Read IR tracking state and update line/search flags.
 */
static void heading_straight_update_line_state(
    const heading_straight_segment_config_t *config,
    int32_t distance_count,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    const heading_straight_nav_correction_t *heading,
    heading_straight_line_state_t *out)
{
    heading_straight_line_plan_t plan = heading_straight_line_plan(
        config->line_search_protect);

    out->ir_armed = (distance_count >= plan.line_arm_count) ? 1U : 0U;
    out->stop_mask = (config->fast_correction != 0U) ?
        TASK2_AB_STOP_MASK : TASK2_CD_STOP_CENTER_MASK;
    out->stop_error_max = (config->fast_correction != 0U) ?
        TASK2_AB_STOP_ERROR_MAX : TASK2_CD_STOP_ERROR_MAX;
    out->line_centered = ((ir_ok != 0U) &&
        ((sample->line_mask & out->stop_mask) != 0U) &&
        (abs_i32(sample->error) <= out->stop_error_max)) ? 1U : 0U;
    if (config->line_search_protect >= 2U) {
        out->line_stop_ready = ((ir_ok != 0U) &&
            ((sample->line_mask & TASK3_STRAIGHT_STOP_MASK) != 0U) &&
            (sample->active_count >= TASK3_STRAIGHT_STOP_MIN_IR_COUNT)) ? 1U : 0U;
    } else {
        out->line_stop_ready = out->line_centered;
    }
    out->search_start_count = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_START_COUNT : TASK2_STRAIGHT_SEARCH_START_COUNT;
    out->search_sweep_start_count = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT : TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT;
    out->search_corr_divisor = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_CORR_DIVISOR : TASK2_STRAIGHT_SEARCH_CORR_DIVISOR;
    out->search_corr_max = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_CORR_MAX : TASK2_STRAIGHT_SEARCH_CORR_MAX;
    out->search_soft_corr = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_SOFT_CORR : TASK2_STRAIGHT_SEARCH_SOFT_CORR;
    out->search_sweep_corr = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_SWEEP_CORR : TASK2_STRAIGHT_SEARCH_SWEEP_CORR;
    out->search_base_drop = (config->line_search_protect >= 2U) ?
        TASK3_STRAIGHT_SEARCH_BASE_DROP : TASK2_STRAIGHT_SEARCH_BASE_DROP;
    out->correction_limit = (heading->task4_ac_start_boost != 0U) ?
        TASK4_AC_START_CORR_MAX :
        (((config->line_search_protect >= 2U) || (config->heading_only != 0U)) ?
            TASK3_STRAIGHT_CORR_MAX : STRAIGHT_CORR_MAX);
    out->search_mode = ((config->line_search_protect != 0U) &&
        ((distance_count >= out->search_start_count) ||
         ((out->ir_armed != 0U) && (ir_ok != 0U) &&
          (sample->line_lost == 0U) && (out->line_centered == 0U)))) ? 1U : 0U;
}

/**
 * @brief Return nonzero when the straight segment should stop on a line hit.
 */
/**
 * @brief Check whether the configured line-hit stop condition is satisfied.
 */
static uint8_t heading_straight_line_hit_ready(
    const heading_straight_segment_config_t *config,
    const heading_straight_line_state_t *line_state,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    return ((line_state->ir_armed != 0U) &&
        (ir_ok != 0U) &&
        (sample->line_lost == 0U) &&
        (sample->active_count >= TASK1_STOP_MIN_IR_COUNT) &&
        ((config->line_search_protect == 0U) ||
         (line_state->line_stop_ready != 0U))) ? 1U : 0U;
}

/**
 * @brief Run optional turn handoff hooks when a straight segment reaches a line.
 */
/**
 * @brief Handle line-hit side effects, including the Task6 fast-turn hook.
 */
static uint8_t heading_straight_handle_line_hit(
    const heading_straight_segment_config_t *config,
    uint32_t elapsed_ms,
    int32_t distance_count,
    const ir_tracking_sample_t *sample,
    uint8_t nav_ok,
    const jy62_navigation_t *nav,
    const heading_straight_nav_correction_t *heading)
{
    if (g_task6_c_turn_requested != 0U) {
        const line_fast_turn_config_t turn_config = {
            .tag = "TASK6_C_TURN",
            .ac_elapsed_ms = elapsed_ms,
            .ac_distance_count = distance_count,
            .line_sample = sample,
            .line_yaw_cdeg = heading->raw,
            .target_cdeg = TASK6_C_TURN_TARGET_CDEG,
            .motor_b_pwm = TASK6_C_TURN_B_PWM,
            .motor_a_pwm = TASK6_C_TURN_A_PWM,
            .handoff_turn_dir = TASK3_ARC_TURN_LEFT,
            .target_is_absolute = 0U,
            .enable_line_stop = 1U,
            .brake_after_turn = 1U,
            .report_samples = 1U
        };

        g_task6_c_turn_requested = 0U;
        st011_start_pulse(TASK3_POINT_ALARM_MS);
        (void)run_line_fast_turn(&turn_config);
        return 1U;
    }

    if (task4_ac_debug_enabled(config->tag) != 0U) {
        const line_fast_turn_config_t turn_config = {
            .tag = task4_ac_turn_tag(config->tag),
            .ac_elapsed_ms = elapsed_ms,
            .ac_distance_count = distance_count,
            .line_sample = sample,
            .line_yaw_cdeg = heading->raw,
            .target_cdeg = TASK6_C_TURN_TARGET_CDEG,
            .motor_b_pwm = TASK6_C_TURN_B_PWM,
            .motor_a_pwm = TASK6_C_TURN_A_PWM,
            .handoff_turn_dir = TASK3_ARC_TURN_LEFT,
            .target_is_absolute = 0U,
            .enable_line_stop = 1U,
            .brake_after_turn = 0U,
            .report_samples = 0U
        };

        task4_print_ac_line_debug(config->tag,
            elapsed_ms,
            distance_count,
            sample,
            nav_ok,
            nav);
        st011_start_pulse(TASK3_POINT_ALARM_MS);
        (void)run_line_fast_turn(&turn_config);
        return 1U;
    }

    if (task4_bd_debug_enabled(config->tag) != 0U) {
        const line_fast_turn_config_t turn_config = {
            .tag = task4_bd_turn_tag(config->tag),
            .ac_elapsed_ms = elapsed_ms,
            .ac_distance_count = distance_count,
            .line_sample = sample,
            .line_yaw_cdeg = heading->raw,
            .target_cdeg = TASK4_D_TURN_TARGET_CDEG,
            .motor_b_pwm = TASK4_D_TURN_B_PWM,
            .motor_a_pwm = TASK4_D_TURN_A_PWM,
            .handoff_turn_dir = TASK3_ARC_TURN_RIGHT,
            .target_is_absolute = 0U,
            .enable_line_stop = 0U,
            .brake_after_turn = 0U,
            .report_samples = 0U
        };

        st011_start_pulse(TASK3_POINT_ALARM_MS);
        (void)run_line_fast_turn(&turn_config);
        return 1U;
    }

    return 0U;
}

/**
 * @brief Convert heading-straight stop reason to log text.
 */
static const char *heading_straight_stop_reason_name(uint8_t stop_reason)
{
    if (stop_reason == 1U) {
        return "line";
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
 * @brief Run start alarm/zeroing, initialize sensors, encoders, and first PWM.
 */
static void heading_straight_start_segment(
    const heading_straight_segment_config_t *config,
    straight_pid_t *pid,
    heading_filter_t *heading_filter,
    heading_straight_line_plan_t *line_plan)
{
    heading_straight_control_limits_t start_limits;

    if (config->start_alarm_ms != 0U) {
        TB6612_Brake();
        st011_pulse(config->start_alarm_ms);
        delay_ms_with_st011(TASK1_START_SETTLE_MS);
    }
    if (config->zero_heading != 0U) {
        (void)jy62_zero_to_current(config->tag,
            (config->start_alarm_ms != 0U) ? TASK1_START_SETTLE_MS : 0U);
    }
    if (config->start_alarm_ms != 0U) {
        delay_ms_with_st011(TASK1_AFTER_ZERO_DELAY_MS);
    }

    IRTracking_Init();
    straight_pid_reset(pid);
    heading_filter_reset(heading_filter);
    encoder_reset_distance_counts();
    encoder_enable_interrupts();

    start_limits = heading_straight_control_limits(config->fast_correction,
        config->heading_only,
        config->heading_target_cdeg,
        0U);
    *line_plan = heading_straight_line_plan(config->line_search_protect);
    lc_printf("%s start: zero=%u heading_only=%u fast=%u line_protect=%u B_base=%d A_base=%d ramp=%d/%d h_div=%ld h_max=%ld d_div=%ld d_max=%ld arm=%ld force=%ld search=%ld sweep=%ld period=%ld\r\n",
        config->tag,
        config->zero_heading,
        config->heading_only,
        config->fast_correction,
        config->line_search_protect,
        STRAIGHT_B_BASE_PWM,
        STRAIGHT_A_BASE_PWM,
        TASK1_RAMP_B_START_PWM,
        TASK1_RAMP_A_START_PWM,
        start_limits.heading_corr_divisor,
        start_limits.heading_corr_max,
        start_limits.distance_corr_divisor,
        start_limits.distance_corr_max,
        line_plan->line_arm_count,
        line_plan->force_stop_count,
        line_plan->search_start_count,
        line_plan->search_sweep_start_count,
        line_plan->search_sweep_period_ms);

    TB6612_SetDifferential(TASK1_RAMP_B_START_PWM, TASK1_RAMP_A_START_PWM);
}

/**
 * @brief Print final state for a heading-straight segment.
 */
static void heading_straight_log_stop(
    const heading_straight_segment_config_t *config,
    const heading_straight_line_plan_t *line_plan,
    uint32_t elapsed_ms,
    uint8_t stop_reason,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    jy62_navigation_t *nav)
{
    int32_t motor_b_total;
    int32_t motor_a_total;
    uint8_t stop_nav_ok;

    encoder_get_delta_counts(&motor_b_total, &motor_a_total);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    stop_nav_ok = JY62_PeekNavigation(nav);
    lc_printf("%s stop: reason=%s t=%lu dist=%ld arm=%d force=%d raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u rel_cdeg=%ld B_total=%ld A_total=%ld\r\n",
        config->tag,
        heading_straight_stop_reason_name(stop_reason),
        elapsed_ms,
        motion_distance_count(motor_b_total, motor_a_total),
        line_plan->line_arm_count,
        line_plan->force_stop_count,
        (ir_ok != 0U) ? sample->raw : 0xFFU,
        (ir_ok != 0U) ? sample->line_mask : 0U,
        (ir_ok != 0U) ? sample->active_count : 0U,
        (ir_ok != 0U) ? sample->line_lost : 1U,
        ir_ok,
        stop_nav_ok,
        nav->yaw_relative_cdeg,
        motor_b_total,
        motor_a_total);
}

/**
 * @brief Calculate encoder balance, heading correction, and final drive PWM.
 */
static void heading_straight_update_balance_control(
    const heading_straight_balance_input_t *input,
    straight_pid_t *pid,
    heading_straight_drive_state_t *drive)
{
    const heading_straight_segment_config_t *config = input->config;
    const heading_straight_line_state_t *line_state = input->line_state;
    const heading_straight_nav_correction_t *heading = input->heading;
    heading_straight_control_limits_t limits;

    drive->motor_b_speed = abs_i32(input->motor_b_delta);
    drive->motor_a_speed = abs_i32(input->motor_a_delta);
    drive->speed_correction = straight_pid_update(pid,
        drive->motor_b_speed,
        drive->motor_a_speed,
        STRAIGHT_TARGET_SPEED_DIFF,
        &drive->speed_error,
        &drive->p_term,
        &drive->i_term,
        &drive->d_term);
    drive->distance_error = input->motor_b_total - input->motor_a_total;
    limits = heading_straight_control_limits(config->fast_correction,
        config->heading_only,
        config->heading_target_cdeg,
        heading->task4_ac_start_boost);
    drive->distance_correction = clamp_i32(drive->distance_error /
            limits.distance_corr_divisor,
        -limits.distance_corr_max,
        limits.distance_corr_max);
    drive->balance_correction = drive->speed_correction +
        drive->distance_correction;
    drive->heading_priority = 0U;

    if ((config->fast_correction == 0U) &&
        (abs_i32(heading->error) >= TASK1_HEADING_PRIORITY_CDEG) &&
        (abs_i32(drive->speed_error) <= TASK1_HEADING_PRIORITY_MAX_VERR) &&
        (abs_i32(drive->distance_error) <= TASK1_HEADING_PRIORITY_MAX_DERR) &&
        (heading->correction != 0) &&
        (((drive->balance_correction > 0) && (heading->correction < 0)) ||
         ((drive->balance_correction < 0) && (heading->correction > 0)))) {
        drive->balance_correction = 0;
        pid->integral = 0;
        drive->heading_priority = 1U;
    }
    if (config->heading_only != 0U) {
        drive->balance_correction = 0;
        pid->integral = 0;
        drive->heading_priority = 2U;
    }

    drive->correction = clamp_i32(drive->balance_correction +
            heading->correction,
        -line_state->correction_limit,
        line_state->correction_limit);
    if (config->fast_correction != 0U) {
        drive->correction = clamp_i32(drive->correction +
                TASK2_AB_BIAS_CORRECTION,
            -line_state->correction_limit,
            line_state->correction_limit);
    }
}

/**
 * @brief Override drive PWM with endpoint search oscillation when enabled.
 */
static void heading_straight_update_search_pwm(
    const heading_straight_search_input_t *input,
    heading_straight_drive_state_t *drive)
{
    const heading_straight_segment_config_t *config = input->config;
    const heading_straight_line_state_t *line_state = input->line_state;
    const ir_tracking_sample_t *sample = input->sample;
    int32_t search_direction;

    drive->base_b_pwm = ramp_i32(TASK1_RAMP_B_START_PWM, STRAIGHT_B_BASE_PWM,
        input->elapsed_ms, TASK1_START_RAMP_MS);
    drive->base_a_pwm = ramp_i32(TASK1_RAMP_A_START_PWM, STRAIGHT_A_BASE_PWM,
        input->elapsed_ms, TASK1_START_RAMP_MS);
    if ((config->fast_correction == 0U) &&
        (config->heading_only == 0U) &&
        (config->line_search_protect == 0U) &&
        (input->distance_count >= TASK1_APPROACH_SLOW_COUNT)) {
        drive->base_b_pwm = TASK1_APPROACH_B_BASE_PWM;
        drive->base_a_pwm = TASK1_APPROACH_A_BASE_PWM;
    }

    drive->search_correction = 0;
    if (line_state->search_mode != 0U) {
        drive->base_b_pwm -= line_state->search_base_drop;
        drive->base_a_pwm -= line_state->search_base_drop;
        if (config->line_search_protect >= 2U) {
            search_direction = 0;
        } else {
            search_direction = task2_straight_search_direction(
                config->fast_correction,
                config->heading_target_cdeg);
        }
        if ((input->ir_ok != 0U) && (sample->line_lost == 0U)) {
            drive->search_correction = clamp_i32(-(sample->error /
                    line_state->search_corr_divisor),
                -line_state->search_corr_max,
                line_state->search_corr_max);
            if ((drive->search_correction == 0) && (search_direction != 0)) {
                drive->search_correction = search_direction *
                    line_state->search_soft_corr;
            }
        } else if ((config->line_search_protect < 2U) &&
            (input->distance_count >= line_state->search_sweep_start_count)) {
            drive->search_correction = search_direction *
                line_state->search_sweep_corr;
        } else if (config->line_search_protect < 2U) {
            drive->search_correction = search_direction *
                line_state->search_soft_corr;
        }
        drive->correction = clamp_i32(drive->correction +
                drive->search_correction,
            -line_state->correction_limit,
            line_state->correction_limit);
    }

    drive->motor_b_pwm = clamp_i32(drive->base_b_pwm - drive->correction,
        STRAIGHT_MIN_PWM,
        STRAIGHT_MAX_PWM);
    drive->motor_a_pwm = clamp_i32(drive->base_a_pwm + drive->correction,
        STRAIGHT_MIN_PWM,
        STRAIGHT_MAX_PWM);
}

/**
 * @brief Print one periodic heading-straight diagnostic report.
 */
static void heading_straight_log_report(const heading_straight_report_t *report)
{
    lc_printf("%s t=%lu dist=%ld arm=%u arm_cnt=%ld fast=%u find=%u centered=%u stopok=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u ir_err=%ld nav=%u h_raw=%ld h_tgt=%ld h_flt=%ld h_use=%ld h_gain=%ld h_wob=%u gzlp=%ld h_corr=%ld hp=%u boost=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld search=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
        report->config->tag,
        report->elapsed_ms,
        report->distance_count,
        report->line_state->ir_armed,
        report->line_plan->line_arm_count,
        report->config->fast_correction,
        report->line_state->search_mode,
        report->line_state->line_centered,
        report->line_state->line_stop_ready,
        (report->ir_ok != 0U) ? report->sample->raw : 0xFFU,
        (report->ir_ok != 0U) ? report->sample->line_mask : 0U,
        (report->ir_ok != 0U) ? report->sample->active_count : 0U,
        (report->ir_ok != 0U) ? report->sample->line_lost : 1U,
        report->ir_ok,
        (report->ir_ok != 0U) ? report->sample->error : 0,
        report->nav_ok,
        report->heading->raw,
        report->config->heading_target_cdeg,
        report->heading->filtered,
        report->heading->error,
        report->heading->gain,
        report->heading->wobble,
        report->heading->gyro_z,
        report->heading->correction,
        report->drive->heading_priority,
        report->heading->task4_ac_start_boost,
        report->motor_b_total,
        report->motor_a_total,
        report->drive->distance_error,
        report->drive->distance_correction,
        report->drive->motor_b_speed,
        report->drive->motor_a_speed,
        (int32_t)STRAIGHT_TARGET_SPEED_DIFF,
        report->drive->speed_error,
        report->drive->p_term,
        report->drive->i_term,
        report->drive->d_term,
        report->drive->speed_correction,
        report->drive->balance_correction,
        report->drive->search_correction,
        report->drive->correction,
        report->drive->base_b_pwm,
        report->drive->base_a_pwm,
        report->drive->motor_b_pwm,
        report->drive->motor_a_pwm);
}

/**
 * @brief Drive a heading-stabilized straight segment until a target line/limit.
 */
/**
 * @brief Drive by heading until line hit, forced distance, UART stop, or timeout.
 */
static uint8_t run_heading_straight_to_line_segment(
    const heading_straight_segment_config_t *config)
{
    const char *tag = config->tag;
    straight_pid_t pid;
    heading_filter_t heading_filter;
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint32_t jy62_report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    uint8_t ir_ok = 0U;
    uint8_t stop_reason = 0U;
    heading_straight_line_plan_t start_line_plan;
    uint8_t task6_turn_hook_ran = 0U;

    heading_straight_start_segment(config,
        &pid,
        &heading_filter,
        &start_line_plan);

    while (elapsed_ms < TASK1_MAX_RUN_MS) {
        int32_t motor_b_total;
        int32_t motor_a_total;
        int32_t distance_count;
        uint8_t nav_ok;
        heading_straight_line_state_t line_state;
        heading_straight_nav_correction_t heading;
        heading_straight_drive_state_t drive;
        heading_straight_balance_input_t balance_input;
        heading_straight_search_input_t search_input;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;
        jy62_report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);

        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        nav_ok = JY62_PeekNavigation(&nav);
        heading_straight_update_nav_correction(config,
            distance_count,
            nav_ok,
            &nav,
            &heading_filter,
            &heading);

        ir_ok = IRTracking_ReadSample(&sample);
        heading_straight_update_line_state(config,
            distance_count,
            ir_ok,
            &sample,
            &heading,
            &line_state);

        if (heading_straight_line_hit_ready(config,
            &line_state,
            ir_ok,
            &sample) != 0U) {
            stop_reason = 1U;
            task6_turn_hook_ran = heading_straight_handle_line_hit(config,
                elapsed_ms,
                distance_count,
                &sample,
                nav_ok,
                &nav,
                &heading);
            break;
        }

        if (distance_count >= start_line_plan.force_stop_count) {
            stop_reason = 2U;
            break;
        }

        balance_input.config = config;
        balance_input.line_state = &line_state;
        balance_input.heading = &heading;
        balance_input.motor_b_delta = motor_b_delta;
        balance_input.motor_a_delta = motor_a_delta;
        balance_input.motor_b_total = motor_b_total;
        balance_input.motor_a_total = motor_a_total;
        heading_straight_update_balance_control(&balance_input, &pid, &drive);

        search_input.config = config;
        search_input.line_state = &line_state;
        search_input.sample = &sample;
        search_input.ir_ok = ir_ok;
        search_input.elapsed_ms = elapsed_ms;
        search_input.distance_count = distance_count;
        heading_straight_update_search_pwm(&search_input, &drive);

        TB6612_SetDifferential((int16_t)drive.motor_b_pwm,
            (int16_t)drive.motor_a_pwm);

        if (report_elapsed_ms >= TASK1_REPORT_PERIOD_MS) {
            heading_straight_report_t report;

            report_elapsed_ms = 0;
            report.config = config;
            report.line_plan = &start_line_plan;
            report.line_state = &line_state;
            report.heading = &heading;
            report.drive = &drive;
            report.sample = &sample;
            report.elapsed_ms = elapsed_ms;
            report.distance_count = distance_count;
            report.motor_b_total = motor_b_total;
            report.motor_a_total = motor_a_total;
            report.ir_ok = ir_ok;
            report.nav_ok = nav_ok;
            heading_straight_log_report(&report);
        }

        if (jy62_report_elapsed_ms >= JY62_TASK_REPORT_PERIOD_MS) {
            jy62_report_elapsed_ms = 0;
            jy62_print_navigation_line(tag, elapsed_ms);
        }
    }

    if (task6_turn_hook_ran == 0U) {
        if ((config->stop_alarm_ms != 0U) || (stop_reason != 1U)) {
            TB6612_Brake();
        }
        heading_straight_log_stop(config,
            &start_line_plan,
            elapsed_ms,
            stop_reason,
            ir_ok,
            &sample,
            &nav);
        if (config->stop_alarm_ms != 0U) {
            st011_pulse(config->stop_alarm_ms);
        }
    }

    return stop_reason;
}

#endif
