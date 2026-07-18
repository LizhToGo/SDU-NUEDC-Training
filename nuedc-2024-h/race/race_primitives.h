#ifndef RACE_PRIMITIVES_H
#define RACE_PRIMITIVES_H

/**
 * @file race_primitives.h
 * @brief 竞速控制原语和数学辅助函数。
 *
 * 本头文件在 race_laps.h 声明竞速上下文相关辅助类型后引入。
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
 * @brief 从 JY62 读取当前相对航向和滤波后的 Z 轴角速度。
 */
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

/**
 * @brief 复位竞速差速轮速 PID 参数。
 */
static void race_diff_pid_reset(straight_pid_t *pid)
{
    straight_pid_reset(pid);
    pid->kp = RACE_DIFF_KP;
    pid->ki = 0;
    pid->kd = RACE_DIFF_KD;
    pid->i_limit = 0;
    pid->corr_max = RACE_DIFF_CORR_MAX;
    pid->integral = 0;
    pid->last_error = 0;
}

/**
 * @brief 配置竞速直线/弧线的基础 PWM 和目标轮速差。
 */
static void race_drive_config(straight_drive_config_t *config,
    int32_t base_pwm,
    int32_t target_speed_diff)
{
    config->base_b_pwm = base_pwm;
    config->base_a_pwm = base_pwm;
    config->target_speed_diff = target_speed_diff;
    config->diff_ff_gain = RACE_DIFF_FF_GAIN;
    config->distance_corr_divisor = 1;
    config->distance_corr_max = 0;
    config->correction_max = RACE_DIFF_CORR_MAX;
    config->min_pwm = RACE_LINE_MIN_PWM;
    config->max_pwm = (base_pwm > RACE_LINE_MAX_PWM) ?
        RACE_TASK4_LINE_MAX_PWM : RACE_LINE_MAX_PWM;
}

/**
 * @brief 将航向误差和陀螺仪阻尼转换为限幅后的转向量。
 */
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

/**
 * @brief 根据距离和转向方向估算弧线期望航向进度。
 */
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

/**
 * @brief 判断红外采样是否包含必需位且不包含禁止位。
 */
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

/**
 * @brief 判断左侧边缘传感器是否看到线。
 */
static uint8_t race_left_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_right_clear)
{
    return race_ir_mask_seen(sample,
        RACE_IR_LEFT_EDGE_MASK,
        (require_right_clear != 0U) ? RACE_IR_RIGHT_EDGE_MASK : 0U);
}

/**
 * @brief 判断右侧边缘传感器是否看到线。
 */
static uint8_t race_right_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_left_clear)
{
    return race_ir_mask_seen(sample,
        RACE_IR_RIGHT_EDGE_MASK,
        (require_left_clear != 0U) ? RACE_IR_LEFT_EDGE_MASK : 0U);
}

/**
 * @brief 判断转向误差是否已经跨过目标航向。
 */
static uint8_t race_turn_crossed_target(uint8_t error_valid,
    int32_t last_error_cdeg,
    int32_t current_error_cdeg)
{
    return ((error_valid != 0U) &&
        ((abs_i32(last_error_cdeg) <= RACE_TURN_CROSS_ARM_CDEG) ||
         (abs_i32(current_error_cdeg) <= RACE_TURN_CROSS_ARM_CDEG)) &&
        (((last_error_cdeg < 0) && (current_error_cdeg >= 0)) ||
         ((last_error_cdeg > 0) && (current_error_cdeg <= 0)))) ? 1U : 0U;
}

/**
 * @brief 预测当前 Z 轴角速度在短时间窗内造成的航向漂移。
 */
static int32_t race_predict_yaw_delta_cdeg(int32_t gyro_z_filtered_mdps,
    int32_t predict_ms)
{
    if (predict_ms <= 0) {
        return 0;
    }
    return (int32_t)((gyro_z_filtered_mdps * predict_ms) / 10000);
}

/**
 * @brief 判断快速转向停车逻辑是否拿到有效红外线采样。
 */
static uint8_t race_sensor_fast_turn_line_seen(uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    return ((ir_ok != 0U) &&
        (sample->line_lost == 0U) &&
        (sample->active_count >= RACE_IR_TURN_STOP_MIN_COUNT)) ? 1U : 0U;
}

/**
 * @brief 评估快速转向的中心线、宽线和误差停车条件。
 */
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

/**
 * @brief 评估快速转向的航向停车条件。
 */
static uint8_t race_sensor_fast_turn_yaw_ready(
    const sensor_fast_turn_config_t *config,
    uint8_t nav_ok,
    int32_t yaw_stop_error_cdeg,
    uint8_t yaw_cross_ready)
{
    return ((config->yaw_stop_enable != 0U) &&
        (nav_ok != 0U) &&
        ((abs_i32(yaw_stop_error_cdeg) <= RACE_TURN_YAW_STOP_TOL_CDEG) ||
         (yaw_cross_ready != 0U))) ? 1U : 0U;
}

/**
 * @brief 判断快速转向是否应该切换到慢速 PWM 组合。
 */
static uint8_t race_sensor_fast_turn_should_slow(
    const sensor_fast_turn_config_t *config,
    uint8_t line_seen,
    int32_t turn_yaw_progress,
    int32_t yaw_stop_error_cdeg,
    uint8_t slow_mode)
{
    return (((line_seen != 0U) ||
        ((RACE_FAST_TURN_GYRO_SLOW_ENABLE != 0) &&
         ((turn_yaw_progress >= RACE_FAST_TURN_GYRO_SLOW_CDEG) ||
          ((config->yaw_stop_enable != 0U) &&
           (abs_i32(yaw_stop_error_cdeg) <= RACE_TURN_YAW_SLOW_ZONE_CDEG))))) &&
        (slow_mode == 0U)) ? 1U : 0U;
}

/**
 * @brief 快速转向函数使用的可变运行状态。
 */
typedef struct {
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    uint32_t elapsed_ms;
    uint32_t report_elapsed_ms;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t turn_yaw_start;
    int32_t turn_yaw_delta;
    int32_t turn_yaw_progress;
    int32_t yaw_stop_error_cdeg;
    int32_t last_yaw_stop_error_cdeg;
    int32_t turn_gyro_z_filtered_mdps;
    uint8_t stop_reason;
    uint8_t ir_ok;
    uint8_t nav_ok;
    uint8_t turn_nav_ok;
    uint8_t line_stop_ready;
    uint8_t line_seen;
    uint8_t slow_mode;
    uint8_t center_ready;
    uint8_t wide_ready;
    uint8_t err_ready;
    uint8_t yaw_stop_ready;
    uint8_t yaw_error_valid;
    uint8_t yaw_cross_ready;
} race_sensor_fast_turn_state_t;

/**
 * @brief 将快速转向停止原因转换为日志文本。
 */
static const char *race_sensor_fast_turn_stop_reason_name(uint8_t stop_reason)
{
    if (stop_reason == 1U) {
        return "line";
    }
    if (stop_reason == 4U) {
        return "wide";
    }
    if (stop_reason == 6U) {
        return "yaw";
    }
    if (stop_reason == 2U) {
        return "timeout";
    }
    return "uart_stop";
}

/**
 * @brief 初始化传感器、编码器、航向基准和转向电机 PWM。
 */
static void race_sensor_fast_turn_start(
    const sensor_fast_turn_config_t *config,
    race_sensor_fast_turn_state_t *state)
{
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    state->turn_nav_ok = race_peek_yaw(&state->turn_yaw_start,
        &state->turn_gyro_z_filtered_mdps);
    if ((config->yaw_stop_enable != 0U) && (state->turn_nav_ok != 0U)) {
        state->last_yaw_stop_error_cdeg = normalize_cdeg(
            state->turn_yaw_start - config->yaw_stop_target_cdeg);
        state->yaw_error_valid = 1U;
        if (RACE_FAST_TURN_GYRO_SLOW_ENABLE != 0) {
            state->slow_mode = 1U;
        }
    }
    TB6612_SetDifferential((state->slow_mode != 0U) ?
            config->slow_motor_b_pwm : config->motor_b_pwm,
        (state->slow_mode != 0U) ?
            config->slow_motor_a_pwm : config->motor_a_pwm);
    race_log_printf("%s start: sensor_fast_turn pwm=%d/%d slow=%d/%d stop_mask=0x%02X forbid=0x%02X err_max=%ld yaw_stop=%u target=%ld\r\n",
        config->tag,
        config->motor_b_pwm,
        config->motor_a_pwm,
        config->slow_motor_b_pwm,
        config->slow_motor_a_pwm,
        config->stop_mask,
        config->forbid_mask,
        config->stop_error_max,
        config->yaw_stop_enable,
        config->yaw_stop_target_cdeg);
}

/**
 * @brief 刷新快速转向的红外、航向、编码器状态和慢速模式判断。
 */
static void race_sensor_fast_turn_update(
    const sensor_fast_turn_config_t *config,
    race_sensor_fast_turn_state_t *state)
{
    state->ir_ok = IRTracking_ReadSample(&state->sample);
    state->nav_ok = JY62_PeekNavigation(&state->nav);
    state->turn_yaw_delta =
        ((state->turn_nav_ok != 0U) && (state->nav_ok != 0U)) ?
        normalize_cdeg(state->nav.yaw_relative_cdeg -
            state->turn_yaw_start) : 0;
    state->turn_yaw_progress = abs_i32(state->turn_yaw_delta);
    state->yaw_stop_error_cdeg =
        ((config->yaw_stop_enable != 0U) && (state->nav_ok != 0U)) ?
        normalize_cdeg(state->nav.yaw_relative_cdeg -
            config->yaw_stop_target_cdeg) : 0;
    state->yaw_cross_ready = 0U;
    if ((config->yaw_stop_enable != 0U) && (state->nav_ok != 0U)) {
        state->yaw_cross_ready = race_turn_crossed_target(
            state->yaw_error_valid,
            state->last_yaw_stop_error_cdeg,
            state->yaw_stop_error_cdeg);
        state->last_yaw_stop_error_cdeg = state->yaw_stop_error_cdeg;
        state->yaw_error_valid = 1U;
    }
    encoder_get_total_counts(&state->motor_b_total, &state->motor_a_total);
    state->line_seen = race_sensor_fast_turn_line_seen(state->ir_ok,
        &state->sample);
    state->yaw_stop_ready = race_sensor_fast_turn_yaw_ready(config,
        state->nav_ok,
        state->yaw_stop_error_cdeg,
        state->yaw_cross_ready);
    if (race_sensor_fast_turn_should_slow(config,
        state->line_seen,
        state->turn_yaw_progress,
        state->yaw_stop_error_cdeg,
        state->slow_mode) != 0U) {
        state->slow_mode = 1U;
        TB6612_SetDifferential(config->slow_motor_b_pwm,
            config->slow_motor_a_pwm);
    }
    state->line_stop_ready = race_sensor_fast_turn_line_ready(config,
        state->line_seen,
        &state->sample,
        &state->center_ready,
        &state->wide_ready,
        &state->err_ready);
}

/**
 * @brief 打印一行快速转向周期性诊断信息。
 */
static void race_sensor_fast_turn_log_sample(
    const sensor_fast_turn_config_t *config,
    const race_sensor_fast_turn_state_t *state)
{
    race_log_printf("%s t=%lu nav=%u yaw=%ld yprog=%ld target=%ld yerr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u seen=%u center=%u wide=%u err_ok=%u line_ready=%u yaw_ready=%u\r\n",
        config->tag,
        state->elapsed_ms,
        state->nav_ok,
        (state->nav_ok != 0U) ? state->nav.yaw_relative_cdeg : 0,
        state->turn_yaw_progress,
        config->yaw_stop_target_cdeg,
        state->yaw_stop_error_cdeg,
        (state->nav_ok != 0U) ? state->nav.gyro_z_filtered_mdps : 0,
        state->ir_ok,
        (state->ir_ok != 0U) ? state->sample.raw : 0xFFU,
        (state->ir_ok != 0U) ? state->sample.line_mask : 0U,
        (state->ir_ok != 0U) ? state->sample.active_count : 0U,
        (state->ir_ok != 0U) ? state->sample.line_lost : 1U,
        (state->ir_ok != 0U) ? state->sample.error : 0,
        state->motor_b_total,
        state->motor_a_total,
        state->slow_mode,
        state->line_seen,
        state->center_ready,
        state->wide_ready,
        state->err_ready,
        state->line_stop_ready,
        state->yaw_stop_ready);
}

/**
 * @brief 刹车、读取最终传感器状态并结束快速转向。
 */
static void race_sensor_fast_turn_finish(
    const sensor_fast_turn_config_t *config,
    race_sensor_fast_turn_state_t *state)
{
    TB6612_Brake();
    state->ir_ok = IRTracking_ReadSample(&state->sample);
    state->nav_ok = JY62_PeekNavigation(&state->nav);
    encoder_get_total_counts(&state->motor_b_total, &state->motor_a_total);
    race_log_printf("%s stop: reason=%s t=%lu nav=%u yaw=%ld target=%ld yerr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u yaw_ready=%u\r\n",
        config->tag,
        race_sensor_fast_turn_stop_reason_name(state->stop_reason),
        state->elapsed_ms,
        state->nav_ok,
        (state->nav_ok != 0U) ? state->nav.yaw_relative_cdeg : 0,
        config->yaw_stop_target_cdeg,
        ((config->yaw_stop_enable != 0U) && (state->nav_ok != 0U)) ?
            normalize_cdeg(state->nav.yaw_relative_cdeg -
                config->yaw_stop_target_cdeg) : 0,
        (state->nav_ok != 0U) ? state->nav.gyro_z_filtered_mdps : 0,
        state->ir_ok,
        (state->ir_ok != 0U) ? state->sample.raw : 0xFFU,
        (state->ir_ok != 0U) ? state->sample.line_mask : 0U,
        (state->ir_ok != 0U) ? state->sample.active_count : 0U,
        (state->ir_ok != 0U) ? state->sample.line_lost : 1U,
        (state->ir_ok != 0U) ? state->sample.error : 0,
        state->motor_b_total,
        state->motor_a_total,
        state->slow_mode,
        state->yaw_stop_ready);
}

/**
 * @brief 在竞速点位后执行红外/航向辅助快速转向。
 */
static uint8_t race_sensor_fast_turn(
    const sensor_fast_turn_config_t *config)
{
    race_sensor_fast_turn_state_t state = {0};
    uint32_t control_period_ms = (config->control_period_ms != 0U) ?
        config->control_period_ms : CONTROL_PERIOD_MS;

    race_sensor_fast_turn_start(config, &state);

    while (state.elapsed_ms < RACE_FAST_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(control_period_ms);
        state.elapsed_ms += control_period_ms;
        state.report_elapsed_ms += control_period_ms;

        if (task_uart_stop_requested() != 0U) {
            state.stop_reason = 3U;
            break;
        }

        race_sensor_fast_turn_update(config, &state);

        if (state.line_stop_ready != 0U) {
            state.stop_reason = 1U;
            break;
        }
        if (state.yaw_stop_ready != 0U) {
            state.stop_reason = 6U;
            break;
        }

        if (state.report_elapsed_ms >= RACE_FAST_TURN_REPORT_PERIOD_MS) {
            state.report_elapsed_ms = 0;
            race_sensor_fast_turn_log_sample(config, &state);
        }
    }

    if (state.stop_reason == 0U) {
        state.stop_reason = 2U;
    }

    race_sensor_fast_turn_finish(config, &state);

    encoder_reset_distance_counts();
    return ((state.stop_reason == 1U) || (state.stop_reason == 4U) ||
        (state.stop_reason == 6U)) ? 1U : 0U;
}

/**
 * @brief 原地或近原地转向，直到到达配置的目标航向。
 */
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
    int32_t predicted_yaw_stop_error_cdeg = 0;
    int32_t predicted_yaw_delta_cdeg = 0;
    int32_t turn_gyro_z_filtered_mdps = 0;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t slow_mode = 0U;
    uint8_t yaw_error_valid = 0U;
    uint8_t yaw_cross_ready = 0U;
    uint8_t predictive_stop_ready = 0U;
    uint32_t control_period_ms = (config->control_period_ms != 0U) ?
        config->control_period_ms : CONTROL_PERIOD_MS;

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
    if (abs_i32(last_yaw_stop_error_cdeg) <= RACE_TURN_YAW_SLOW_ZONE_CDEG) {
        slow_mode = 1U;
    }
    TB6612_SetDifferential((slow_mode != 0U) ? slow_motor_b_pwm : motor_b_pwm,
        (slow_mode != 0U) ? slow_motor_a_pwm : motor_a_pwm);
    race_log_printf("%s start: gyro_turn pwm=%d/%d slow=%d/%d yaw0=%ld target=%ld yerr=%ld timeout=%d\r\n",
        tag,
        motor_b_pwm,
        motor_a_pwm,
        slow_motor_b_pwm,
        slow_motor_a_pwm,
        turn_yaw_start,
        yaw_stop_target_cdeg,
        last_yaw_stop_error_cdeg,
        RACE_GYRO_TURN_TIMEOUT_MS);

    while (elapsed_ms < RACE_GYRO_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(control_period_ms);
        elapsed_ms += control_period_ms;
        report_elapsed_ms += control_period_ms;

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
        predictive_stop_ready = 0U;
        predicted_yaw_delta_cdeg = 0;
        predicted_yaw_stop_error_cdeg = yaw_stop_error_cdeg;
        if ((config->predictive_stop_enable != 0U) &&
            (abs_i32(yaw_stop_error_cdeg) <=
                RACE_TASK4_EXIT_TURN_PREDICT_ARM_CDEG) &&
            (abs_i32(nav.gyro_z_filtered_mdps) >=
                abs_i32(config->predictive_stop_min_gz_mdps))) {
            predicted_yaw_delta_cdeg = race_predict_yaw_delta_cdeg(
                nav.gyro_z_filtered_mdps,
                config->predictive_stop_ms);
            predicted_yaw_stop_error_cdeg = normalize_cdeg(
                nav.yaw_relative_cdeg + predicted_yaw_delta_cdeg -
                yaw_stop_target_cdeg);
            if ((abs_i32(predicted_yaw_stop_error_cdeg) <
                    abs_i32(yaw_stop_error_cdeg)) &&
                ((abs_i32(predicted_yaw_stop_error_cdeg) <=
                    RACE_TURN_YAW_STOP_TOL_CDEG) ||
                 (race_turn_crossed_target(1U,
                    yaw_stop_error_cdeg,
                    predicted_yaw_stop_error_cdeg) != 0U))) {
                predictive_stop_ready = 1U;
            }
        }
        yaw_cross_ready = race_turn_crossed_target(yaw_error_valid,
            last_yaw_stop_error_cdeg,
            yaw_stop_error_cdeg);
        last_yaw_stop_error_cdeg = yaw_stop_error_cdeg;
        yaw_error_valid = 1U;

        if ((slow_mode == 0U) &&
            (abs_i32(yaw_stop_error_cdeg) <= RACE_TURN_YAW_SLOW_ZONE_CDEG)) {
            slow_mode = 1U;
            TB6612_SetDifferential(slow_motor_b_pwm, slow_motor_a_pwm);
        }
        if ((abs_i32(yaw_stop_error_cdeg) <= RACE_TURN_YAW_STOP_TOL_CDEG) ||
            (yaw_cross_ready != 0U) ||
            (predictive_stop_ready != 0U)) {
            stop_reason = 6U;
            break;
        }

        if (report_elapsed_ms >= RACE_FAST_TURN_REPORT_PERIOD_MS) {
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
    return (stop_reason == 6U) ? 1U : 0U;
}

/**
 * @brief 点位后按固定编码器距离向前过渡。
 */
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
        race_log_printf("%s skip: advance_count=%ld\r\n", tag, advance_count);
        return 1U;
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)RACE_POINT_ADVANCE_PWM,
        (int16_t)RACE_POINT_ADVANCE_PWM);
    race_log_printf("%s start: advance_count=%ld pwm=%d\r\n",
        tag,
        advance_count,
        RACE_POINT_ADVANCE_PWM);

    while (elapsed_ms < RACE_POINT_ADVANCE_TIMEOUT_MS) {
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

        if (report_elapsed_ms >= RACE_FAST_TURN_REPORT_PERIOD_MS) {
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

/**
 * @brief 任务四弧线出口后，带小幅航向保持修正的前进过渡。
 */
static uint8_t race_advance_after_point_with_heading(const char *tag,
    int32_t advance_count,
    int32_t target_cdeg)
{
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;
    int32_t heading_error_cdeg = 0;
    int32_t correction = 0;
    int32_t motor_b_pwm = RACE_POINT_ADVANCE_PWM;
    int32_t motor_a_pwm = RACE_POINT_ADVANCE_PWM;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t target_valid = 0U;

    if ((RACE_TASK4_ADVANCE_GYRO_ENABLE == 0) || (advance_count <= 0)) {
        return race_advance_after_point(tag, advance_count);
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    target_cdeg = normalize_cdeg(target_cdeg);
    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok != 0U) {
        target_valid = 1U;
    }
    TB6612_SetDifferential((int16_t)motor_b_pwm, (int16_t)motor_a_pwm);
    race_log_printf("%s start: advance_count=%ld pwm=%d hold=%u target=%ld gyro=1\r\n",
        tag,
        advance_count,
        RACE_POINT_ADVANCE_PWM,
        target_valid,
        (long)target_cdeg);

    while (elapsed_ms < RACE_POINT_ADVANCE_TIMEOUT_MS) {
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

        if ((nav_ok != 0U) && (target_valid != 0U)) {
            heading_error_cdeg = normalize_cdeg(nav.yaw_relative_cdeg -
                target_cdeg);
            correction = race_heading_turn_from_error(heading_error_cdeg,
                nav.gyro_z_filtered_mdps,
                RACE_TASK4_ADVANCE_HEADING_CORR_DIVISOR,
                RACE_TASK4_ADVANCE_GYRO_DAMP_DIVISOR,
                RACE_TASK4_ADVANCE_HEADING_CORR_MAX);
        } else {
            heading_error_cdeg = 0;
            correction = 0;
        }

        motor_b_pwm = clamp_i32(RACE_POINT_ADVANCE_PWM + correction,
            RACE_LINE_MIN_PWM,
            RACE_LINE_MAX_PWM);
        motor_a_pwm = clamp_i32(RACE_POINT_ADVANCE_PWM - correction,
            RACE_LINE_MIN_PWM,
            RACE_LINE_MAX_PWM);
        TB6612_SetDifferential((int16_t)motor_b_pwm, (int16_t)motor_a_pwm);

        if (distance_count >= advance_count) {
            stop_reason = 1U;
            break;
        }

        if (report_elapsed_ms >= RACE_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            race_log_printf("%s t=%lu dist=%ld nav=%u yaw=%ld target=%ld herr=%ld corr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld pwm=%ld/%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                nav_ok,
                (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
                (long)target_cdeg,
                heading_error_cdeg,
                correction,
                (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                motor_b_total,
                motor_a_total,
                motor_b_pwm,
                motor_a_pwm);
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    distance_count = motion_distance_count(motor_b_total, motor_a_total);
    nav_ok = JY62_PeekNavigation(&nav);
    ir_ok = IRTracking_ReadSample(&sample);
    race_log_printf("%s stop: reason=%s t=%lu dist=%ld nav=%u yaw=%ld target=%ld herr=%ld corr=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld\r\n",
        tag,
        race_reason_name(stop_reason),
        elapsed_ms,
        distance_count,
        nav_ok,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (long)target_cdeg,
        (nav_ok != 0U) ? normalize_cdeg(nav.yaw_relative_cdeg -
            target_cdeg) : 0,
        correction,
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

/**
 * @brief 强制入弯后向前找线，直到任意有效线出现。
 */
static uint8_t race_drive_forward_until_line(const char *tag, int32_t max_count)
{
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;
    uint8_t ir_ok = 0U;
    uint8_t stop_reason = 0U;

    if (max_count <= 0) {
        return 0U;
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)RACE_FORCE_FIND_LINE_PWM,
        (int16_t)RACE_FORCE_FIND_LINE_PWM);
    race_log_printf("%s start: find_line max=%ld pwm=%d\r\n",
        tag,
        max_count,
        RACE_FORCE_FIND_LINE_PWM);

    while (elapsed_ms < RACE_FORCE_FIND_LINE_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = motion_distance_count(motor_b_total, motor_a_total);
        ir_ok = IRTracking_ReadSample(&sample);
        if ((ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (sample.active_count >= RACE_IR_TURN_STOP_MIN_COUNT)) {
            stop_reason = 1U;
            break;
        }
        if (distance_count >= max_count) {
            stop_reason = 2U;
            break;
        }
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    distance_count = motion_distance_count(motor_b_total, motor_a_total);
    ir_ok = IRTracking_ReadSample(&sample);
    race_log_printf("%s stop: reason=%s t=%lu dist=%ld ir=%u mask=0x%02X cnt=%u lost=%u err=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "line" : ((stop_reason == 2U) ? "distance" :
            ((stop_reason == 3U) ? "uart_stop" : "timeout")),
        elapsed_ms,
        distance_count,
        ir_ok,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0);

    return (stop_reason == 1U) ? 1U : 0U;
}

#endif /* RACE_PRIMITIVES_H */
