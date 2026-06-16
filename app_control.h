#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

#include "app_config.h"

/* 直行 PID 状态。系数使用整数定点，最终统一除以 STRAIGHT_PID_SCALE。 */
/**
 * @brief Integer PID state for wheel-speed difference control.
 */
typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t i_limit;
    int32_t corr_max;
    int32_t integral;
    int32_t last_error;
} straight_pid_t;

/* 航向经验滤波状态，用来抑制车身短时晃动带来的 JY62 误修正。 */
/**
 * @brief Low-pass filter state for heading correction.
 */
typedef struct {
    int32_t filtered_cdeg;
    uint8_t initialized;
} heading_filter_t;

/* 限幅函数，用于 PWM、积分和修正量。 */
static inline int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/* 从起始值平滑过渡到目标值，用于起步斜坡。 */
static inline int32_t ramp_i32(int32_t start_value, int32_t target_value, uint32_t elapsed_ms, uint32_t ramp_ms)
{
    if ((ramp_ms == 0U) || (elapsed_ms >= ramp_ms)) {
        return target_value;
    }

    return start_value + (((target_value - start_value) * (int32_t)elapsed_ms) / (int32_t)ramp_ms);
}

/**
 * @brief Absolute value helper for signed 32-bit integers.
 */
static inline int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 进入直行闭环控制前，重置 PID 运行状态。 */
static inline void straight_pid_reset(straight_pid_t *pid)
{
    pid->kp = STRAIGHT_PID_KP;
    pid->ki = STRAIGHT_PID_KI;
    pid->kd = STRAIGHT_PID_KD;
    pid->i_limit = STRAIGHT_I_LIMIT;
    pid->corr_max = STRAIGHT_CORR_MAX;
    pid->integral = 0;
    pid->last_error = 0;
}

/**
 * @brief Override PID integral and output limits after reset.
 */
static inline void straight_pid_set_limits(straight_pid_t *pid,
    int32_t i_limit,
    int32_t corr_max)
{
    pid->i_limit = (i_limit > 0) ? i_limit : STRAIGHT_I_LIMIT;
    pid->corr_max = (corr_max > 0) ? corr_max : STRAIGHT_CORR_MAX;
    pid->integral = clamp_i32(pid->integral, -pid->i_limit, pid->i_limit);
}

/**
 * @brief Reset heading low-pass filter state.
 */
static inline void heading_filter_reset(heading_filter_t *filter)
{
    filter->filtered_cdeg = 0;
    filter->initialized = 0U;
}

/* 根据 Z 轴角速度给航向修正降权：慢速偏航保留，高速晃动逐步压掉。 */
/**
 * @brief Reduce heading correction gain when gyro motion is too large.
 */
static inline int32_t heading_filter_gain_from_gyro(int32_t gyro_z_filtered_mdps)
{
    int32_t gyro_abs = abs_i32(gyro_z_filtered_mdps);

    if (gyro_abs <= TASK1_HEADING_GYRO_GATE_START_MDPS) {
        return 100;
    }

    if (gyro_abs >= TASK1_HEADING_GYRO_GATE_END_MDPS) {
        return 0;
    }

    return 100 - (((gyro_abs - TASK1_HEADING_GYRO_GATE_START_MDPS) * 100) /
        (TASK1_HEADING_GYRO_GATE_END_MDPS - TASK1_HEADING_GYRO_GATE_START_MDPS));
}

/*
 * 经验公式：
 * 1. rel_cdeg 先做一阶低通，得到稳定航向误差；
 * 2. |gzlp_mdps| 较大时改用更慢的低通，并降低修正增益；
 * 3. 小于死区的误差直接归零，避免小抖动来回打方向。
 */
/**
 * @brief Low-pass and gate raw heading error before yaw correction.
 */
static inline int32_t heading_filter_update(heading_filter_t *filter,
    int32_t raw_heading_cdeg,
    int32_t gyro_z_filtered_mdps,
    int32_t *filtered_heading_out,
    int32_t *gain_percent_out,
    uint8_t *wobble_out)
{
    int32_t gain_percent = heading_filter_gain_from_gyro(gyro_z_filtered_mdps);
    int32_t filter_divisor = (gain_percent >= 100) ?
        TASK1_HEADING_FILTER_DIVISOR : TASK1_HEADING_WOBBLE_FILTER_DIVISOR;
    int32_t used_heading;

    if (filter->initialized == 0U) {
        filter->filtered_cdeg = raw_heading_cdeg;
        filter->initialized = 1U;
    } else {
        filter->filtered_cdeg += (raw_heading_cdeg - filter->filtered_cdeg) / filter_divisor;
    }

    used_heading = filter->filtered_cdeg;
    if (abs_i32(used_heading) < TASK1_HEADING_DEADBAND_CDEG) {
        used_heading = 0;
    } else {
        used_heading = (used_heading * gain_percent) / 100;
    }

    *filtered_heading_out = filter->filtered_cdeg;
    *gain_percent_out = gain_percent;
    *wobble_out = (gain_percent < 100) ? 1U : 0U;

    return used_heading;
}

/*
 * PID 目标：让 B_spd - A_spd 接近 target_speed_diff。
 * correction 为正表示 B 轮相对目标偏快，因此会降低 B 轮 PWM、提高 A 轮 PWM。
 */
/**
 * @brief Update wheel-speed-difference PID and return PWM correction.
 */
static inline int32_t straight_pid_update(straight_pid_t *pid,
    int32_t motor_b_speed,
    int32_t motor_a_speed,
    int32_t target_speed_diff,
    int32_t *error_out,
    int32_t *p_out,
    int32_t *i_out,
    int32_t *d_out)
{
    int32_t speed_diff = motor_b_speed - motor_a_speed;
    int32_t error = speed_diff - target_speed_diff;
    int32_t derivative = error - pid->last_error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t output;

    /*
     * err = (B_spd - A_spd) - target：
     * err > 0 表示左轮 B 相对目标偏快，需要降低 B、提高 A；
     * err < 0 表示右轮 A 相对目标偏快，需要提高 B、降低 A。
     */
    pid->integral = clamp_i32(pid->integral + error, -pid->i_limit, pid->i_limit);
    pid->last_error = error;

    p_term = pid->kp * error;
    i_term = pid->ki * pid->integral;
    d_term = pid->kd * derivative;
    output = p_term + i_term + d_term;
    output /= STRAIGHT_PID_SCALE;

    *error_out = error;
    *p_out = p_term;
    *i_out = i_term;
    *d_out = d_term;

    return clamp_i32(output, -pid->corr_max, pid->corr_max);
}

#endif
