#ifndef APP_STRAIGHT_H
#define APP_STRAIGHT_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"

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

static inline void straight_drive_config_pid_test(straight_drive_config_t *config)
{
    config->base_b_pwm = STRAIGHT_B_BASE_PWM;
    config->base_a_pwm = STRAIGHT_A_BASE_PWM;
    config->target_speed_diff = PID_TEST_TARGET_SPEED_DIFF;
    config->diff_ff_gain = PID_TEST_DIFF_FF_GAIN;
    config->distance_corr_divisor = PID_TEST_DISTANCE_CORR_DIVISOR;
    config->distance_corr_max = PID_TEST_DISTANCE_CORR_MAX;
    config->correction_max = PID_TEST_CORR_MAX;
    config->min_pwm = STRAIGHT_MIN_PWM;
    config->max_pwm = STRAIGHT_MAX_PWM;
}

static inline void straight_drive_config_pd_test(straight_drive_config_t *config)
{
    config->base_b_pwm = STRAIGHT_B_BASE_PWM;
    config->base_a_pwm = STRAIGHT_A_BASE_PWM;
    config->target_speed_diff = PD_TEST_TARGET_SPEED_DIFF;
    config->diff_ff_gain = PD_TEST_DIFF_FF_GAIN;
    config->distance_corr_divisor = PD_TEST_DISTANCE_CORR_DIVISOR;
    config->distance_corr_max = PD_TEST_DISTANCE_CORR_MAX;
    config->correction_max = PD_TEST_CORR_MAX;
    config->min_pwm = STRAIGHT_MIN_PWM;
    config->max_pwm = STRAIGHT_MAX_PWM;
}

static inline int32_t straight_drive_feedforward(const straight_drive_config_t *config)
{
    int32_t correction = -(config->target_speed_diff * config->diff_ff_gain);

    return clamp_i32(correction,
        -config->correction_max,
        config->correction_max);
}

static inline void straight_drive_update(straight_pid_t *pid,
    const straight_drive_config_t *config,
    int32_t motor_b_delta,
    int32_t motor_a_delta,
    int32_t motor_b_total,
    int32_t motor_a_total,
    straight_drive_output_t *out)
{
    int32_t distance_corr_divisor = (config->distance_corr_divisor != 0) ?
        config->distance_corr_divisor : 1;

    out->motor_b_speed = abs_i32(motor_b_delta);
    out->motor_a_speed = abs_i32(motor_a_delta);
    out->speed_diff = out->motor_b_speed - out->motor_a_speed;
    out->distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
    out->distance_error = motor_b_total - motor_a_total;
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

#endif
