#ifndef APP_STRAIGHT_H
#define APP_STRAIGHT_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"

/**
 * @brief Tunable parameters for encoder differential straight driving.
 *
 * Positive correction lowers B PWM and raises A PWM in the current wiring.
 */
typedef struct {
    int32_t base_b_pwm;
    int32_t base_a_pwm;
    int32_t target_speed_diff;
    int32_t diff_ff_gain;
    int32_t distance_corr_divisor;
    int32_t distance_corr_max;
    int32_t correction_max;
    int32_t min_pwm;
    int32_t max_pwm;
} straight_drive_config_t;

/**
 * @brief Encoder input snapshot for one control period.
 */
typedef struct {
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
} straight_drive_input_t;

/**
 * @brief Full calculated output of one straight-drive PID update.
 */
typedef struct {
    int32_t motor_b_speed;
    int32_t motor_a_speed;
    int32_t speed_diff;
    int32_t distance_count;
    int32_t distance_error;
    int32_t distance_correction;
    int32_t pid_error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t feedforward_correction;
    int32_t feedback_correction;
    int32_t correction;
    int32_t motor_b_pwm;
    int32_t motor_a_pwm;
} straight_drive_output_t;

/**
 * @brief Fill a straight-drive config from shared base PWM and supplied gains.
 */
static inline void straight_drive_apply_test_config(straight_drive_config_t *config,
    int32_t target_speed_diff,
    int32_t diff_ff_gain,
    int32_t distance_corr_divisor,
    int32_t distance_corr_max,
    int32_t correction_max)
{
    config->base_b_pwm = STRAIGHT_B_BASE_PWM;
    config->base_a_pwm = STRAIGHT_A_BASE_PWM;
    config->target_speed_diff = target_speed_diff;
    config->diff_ff_gain = diff_ff_gain;
    config->distance_corr_divisor = distance_corr_divisor;
    config->distance_corr_max = distance_corr_max;
    config->correction_max = correction_max;
    config->min_pwm = STRAIGHT_MIN_PWM;
    config->max_pwm = STRAIGHT_MAX_PWM;
}

/**
 * @brief Load the PID-test straight-drive parameter set.
 */
static inline void straight_drive_config_pid_test(straight_drive_config_t *config)
{
    straight_drive_apply_test_config(config,
        PID_TEST_TARGET_SPEED_DIFF,
        PID_TEST_DIFF_FF_GAIN,
        PID_TEST_DISTANCE_CORR_DIVISOR,
        PID_TEST_DISTANCE_CORR_MAX,
        PID_TEST_CORR_MAX);
}

/**
 * @brief Load the PD-test straight-drive parameter set.
 */
static inline void straight_drive_config_pd_test(straight_drive_config_t *config)
{
    straight_drive_apply_test_config(config,
        PD_TEST_TARGET_SPEED_DIFF,
        PD_TEST_DIFF_FF_GAIN,
        PD_TEST_DISTANCE_CORR_DIVISOR,
        PD_TEST_DISTANCE_CORR_MAX,
        PD_TEST_CORR_MAX);
}

/*
 * Feedforward correction for the expected B-A wheel-speed bias.
 * Unit: PWM correction count. Positive correction lowers B PWM and raises A PWM.
 */
static inline int32_t straight_drive_feedforward(const straight_drive_config_t *config)
{
    int32_t correction = -(config->target_speed_diff * config->diff_ff_gain);

    return clamp_i32(correction,
        -config->correction_max,
        config->correction_max);
}

/**
 * @brief Update straight-drive correction from a grouped encoder snapshot.
 */
static inline void straight_drive_update_from_input(straight_pid_t *pid,
    const straight_drive_config_t *config,
    const straight_drive_input_t *input,
    straight_drive_output_t *out)
{
    int32_t distance_corr_divisor = (config->distance_corr_divisor != 0) ?
        config->distance_corr_divisor : 1;

    out->motor_b_speed = abs_i32(input->motor_b_delta);
    out->motor_a_speed = abs_i32(input->motor_a_delta);
    out->speed_diff = out->motor_b_speed - out->motor_a_speed;
    out->distance_count = motion_distance_count(input->motor_b_total,
        input->motor_a_total);
    out->distance_error = input->motor_b_total - input->motor_a_total;
    out->feedforward_correction = straight_drive_feedforward(config);
    out->feedback_correction = straight_pid_update(pid,
        out->motor_b_speed,
        out->motor_a_speed,
        config->target_speed_diff,
        &out->pid_error,
        &out->p_term,
        &out->i_term,
        &out->d_term);
    out->distance_correction = clamp_i32(out->distance_error / distance_corr_divisor,
        -config->distance_corr_max,
        config->distance_corr_max);
    out->correction = clamp_i32(out->feedforward_correction +
            out->feedback_correction +
            out->distance_correction,
        -config->correction_max,
        config->correction_max);
    out->motor_b_pwm = clamp_i32(config->base_b_pwm - out->correction,
        config->min_pwm,
        config->max_pwm);
    out->motor_a_pwm = clamp_i32(config->base_a_pwm + out->correction,
        config->min_pwm,
        config->max_pwm);
}

/**
 * @brief Convenience wrapper for straight_drive_update_from_input().
 */
static inline void straight_drive_update(straight_pid_t *pid,
    const straight_drive_config_t *config,
    int32_t motor_b_delta,
    int32_t motor_a_delta,
    int32_t motor_b_total,
    int32_t motor_a_total,
    straight_drive_output_t *out)
{
    const straight_drive_input_t input = {
        motor_b_delta,
        motor_a_delta,
        motor_b_total,
        motor_a_total,
    };

    straight_drive_update_from_input(pid, config, &input, out);
}

#endif
