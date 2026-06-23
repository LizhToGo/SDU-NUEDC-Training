#ifndef STRAIGHT_LINE_H
#define STRAIGHT_LINE_H

/**
 * @file straight_line.h
 * @brief 任务一和任务二共用的直线到线控制器。
 *
 * 本模块保留编码器差速闭环、可选 JY62 航向修正、红外到线停车
 * 和串口文本诊断，把原先堆在 main.c 中的直线段逻辑集中到一处。
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
 * @brief 单次直线到线运行的静态配置。
 *
 * 任务一和任务二直线段共用这套控制器；距离单位为编码器计数，
 * 航向单位为百分之一度。
 */
typedef struct {
    const char *tag;
    uint8_t zero_heading;
    uint32_t start_alarm_ms;
    uint32_t stop_alarm_ms;
    int32_t line_arm_count;
    int32_t force_stop_count;
    uint8_t stop_min_ir_count;
    uint8_t yaw_corr_enable;
    uint8_t entry_brake_enable;
    uint8_t fixed_yaw_target_enable;
    int32_t fixed_yaw_target_cdeg;
} straight_line_segment_config_t;

/**
 * @brief 周期性直线诊断输出需要的状态快照。
 *
 * 使用参数对象可以让主控制循环保持清爽，不需要在打印函数参数中
 * 塞入一长串零散状态。
 */
typedef struct {
    const straight_line_segment_config_t *config;
    const straight_drive_config_t *drive_config;
    const straight_drive_output_t *drive;
    const ir_tracking_sample_t *sample;
    const jy62_navigation_t *nav;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_start_cdeg;
    int32_t yaw_target_cdeg;
    int32_t yaw_correction;
    int32_t correction;
    int32_t motor_b_total;
    int32_t motor_a_total;
    uint8_t ir_armed;
    uint8_t ir_ok;
    uint8_t nav_ok;
} straight_line_report_t;

/**
 * @brief 一次差速驱动更新所需的输入。
 *
 * 将编码器增量/总量和当前航向目标打包，便于驱动控制函数只关注
 * “如何算 PWM”。
 */
typedef struct {
    uint32_t elapsed_ms;
    int32_t target_b_base_pwm;
    int32_t target_a_base_pwm;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t yaw_target_cdeg;
    uint8_t nav_ok;
    const jy62_navigation_t *nav;
} straight_line_control_input_t;

/**
 * @brief 直线段启动时打印的诊断信息。
 */
typedef struct {
    const straight_line_segment_config_t *config;
    const straight_drive_config_t *drive_config;
    const ir_tracking_sample_t *sample;
    uint8_t nav_start_ok;
    uint32_t report_period_ms;
    int32_t yaw_start_cdeg;
    int32_t yaw_target_cdeg;
    int32_t feedforward_correction;
} straight_line_start_log_t;

/**
 * @brief 直线段结束时打印的诊断信息。
 */
typedef struct {
    const straight_line_segment_config_t *config;
    const ir_tracking_sample_t *sample;
    const jy62_navigation_t *nav;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_target_cdeg;
    int32_t motor_b_total;
    int32_t motor_a_total;
    uint8_t stop_reason;
    uint8_t ir_ok;
    uint8_t stop_nav_ok;
} straight_line_stop_log_t;

/**
 * @brief 直线到线控制函数的运行期状态。
 *
 * 这些字段对应控制循环里的局部变量。集中成结构体后，辅助函数可以
 * 在不改变时序、PID、航向修正和到线停车行为的前提下更新状态。
 */
typedef struct {
    uint32_t elapsed_ms;
    uint32_t report_elapsed_ms;
    uint32_t jy62_report_elapsed_ms;
    uint32_t report_period_ms;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t distance_count;
    int32_t yaw_correction;
    int32_t correction;
    int32_t yaw_start_cdeg;
    int32_t yaw_target_cdeg;
    int32_t feedforward_correction;
    int32_t target_b_base_pwm;
    int32_t target_a_base_pwm;
    uint8_t ir_ok;
    uint8_t nav_ok;
    uint8_t nav_start_ok;
    uint8_t stop_nav_ok;
    uint8_t stop_reason;
    uint8_t ir_armed;
} straight_line_runtime_t;

static int32_t task2_fixed_yaw_correction(uint8_t nav_ok,
    const jy62_navigation_t *nav,
    int32_t yaw_target_cdeg)
{
#if ENABLE_JY62_NAV
    int32_t yaw_error_cdeg;
    int32_t correction;

    if ((nav_ok == 0U) || (nav == 0)) {
        return 0;
    }

    yaw_error_cdeg = normalize_cdeg(nav->yaw_relative_cdeg - yaw_target_cdeg);
    if (abs_i32(yaw_error_cdeg) <= STRAIGHT_YAW_DEADBAND_CDEG) {
        yaw_error_cdeg = 0;
    }

    correction = -(yaw_error_cdeg / TASK2_CD_HEADING_CORR_DIVISOR);
#if TASK2_CD_HEADING_GYRO_DAMP_DIVISOR > 0
    correction -= nav->gyro_z_filtered_mdps / TASK2_CD_HEADING_GYRO_DAMP_DIVISOR;
#endif

    return clamp_i32(correction,
        -TASK2_CD_HEADING_CORR_MAX,
        TASK2_CD_HEADING_CORR_MAX);
#else
    (void)nav_ok;
    (void)nav;
    (void)yaw_target_cdeg;
    return 0;
#endif
}

static int32_t straight_line_yaw_correction(uint8_t nav_ok,
    const jy62_navigation_t *nav,
    int32_t yaw_start_cdeg)
{
#if ENABLE_JY62_NAV
    int32_t yaw_error_cdeg;
    int32_t correction;

    if ((nav_ok == 0U) || (nav == 0)) {
        return 0;
    }

    yaw_error_cdeg = normalize_cdeg(nav->yaw_relative_cdeg - yaw_start_cdeg);
    if (abs_i32(yaw_error_cdeg) <= STRAIGHT_YAW_DEADBAND_CDEG) {
        yaw_error_cdeg = 0;
    }

    correction = -(yaw_error_cdeg / STRAIGHT_YAW_CORR_DIVISOR);
#if STRAIGHT_YAW_GYRO_DAMP_DIVISOR > 0
    correction -= nav->gyro_z_filtered_mdps / STRAIGHT_YAW_GYRO_DAMP_DIVISOR;
#endif

    return clamp_i32(correction,
        -STRAIGHT_YAW_CORR_MAX,
        STRAIGHT_YAW_CORR_MAX);
#else
    (void)nav_ok;
    (void)nav;
    (void)yaw_start_cdeg;
    return 0;
#endif
}

static const char *straight_line_stop_reason_name(uint8_t stop_reason)
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
 * @brief 执行起步前的刹车、声光、稳定等待和航向置零。
 */
static void straight_line_prepare_start(
    const straight_line_segment_config_t *config)
{
    if (config->entry_brake_enable != 0U) {
        TB6612_Brake();
    }
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
}

/**
 * @brief 运行直线差速 PID，并叠加可选的航向反馈。
 *
 * 最终修正量为正时降低 B 轮 PWM、提高 A 轮 PWM，沿用
 * straight_drive_update() 的历史约定。
 */
static void straight_line_apply_drive_control(
    const straight_line_segment_config_t *config,
    const straight_line_control_input_t *input,
    straight_pid_t *pid,
    straight_drive_config_t *drive_config,
    straight_drive_output_t *drive,
    int32_t *yaw_correction,
    int32_t *correction)
{
    drive_config->base_b_pwm = ramp_i32(TASK1_RAMP_B_START_PWM,
        input->target_b_base_pwm,
        input->elapsed_ms,
        TASK1_START_RAMP_MS);
    drive_config->base_a_pwm = ramp_i32(TASK1_RAMP_A_START_PWM,
        input->target_a_base_pwm,
        input->elapsed_ms,
        TASK1_START_RAMP_MS);
    straight_drive_update(pid,
        drive_config,
        input->motor_b_delta,
        input->motor_a_delta,
        input->motor_b_total,
        input->motor_a_total,
        drive);
    if (config->yaw_corr_enable != 0U) {
        *yaw_correction = (config->fixed_yaw_target_enable != 0U) ?
            task2_fixed_yaw_correction(input->nav_ok,
                input->nav,
                input->yaw_target_cdeg) :
            straight_line_yaw_correction(input->nav_ok,
                input->nav,
                input->yaw_target_cdeg);
    } else {
        *yaw_correction = 0;
    }

    *correction = clamp_i32(drive->correction + *yaw_correction,
        -drive_config->correction_max,
        drive_config->correction_max);
    drive->motor_b_pwm = clamp_i32(drive_config->base_b_pwm - *correction,
        drive_config->min_pwm,
        drive_config->max_pwm);
    drive->motor_a_pwm = clamp_i32(drive_config->base_a_pwm + *correction,
        drive_config->min_pwm,
        drive_config->max_pwm);
}

/**
 * @brief 判断红外到线停车是否已经使能且当前满足触发条件。
 */
static uint8_t straight_line_detect_line_crossing(
    const straight_line_segment_config_t *config,
    int32_t distance_count,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t *ir_armed)
{
    *ir_armed = (distance_count >= config->line_arm_count) ? 1U : 0U;

    return ((*ir_armed != 0U) &&
        (ir_ok != 0U) &&
        (sample->line_lost == 0U) &&
        (sample->active_count >= config->stop_min_ir_count)) ? 1U : 0U;
}

/**
 * @brief 执行一个控制周期内的传感器读取和电机 PWM 计算。
 */
static void straight_line_update_drive_step(
    const straight_line_segment_config_t *config,
    straight_line_runtime_t *runtime,
    straight_pid_t *pid,
    straight_drive_config_t *drive_config,
    straight_drive_output_t *drive,
    ir_tracking_sample_t *sample,
    jy62_navigation_t *nav)
{
#if ENABLE_JY62_NAV
    (void)JY62_GetNavigation(nav);
    runtime->nav_ok = nav->valid;
#else
    runtime->nav_ok = 0U;
#endif
    encoder_get_delta_counts(&runtime->motor_b_delta, &runtime->motor_a_delta);
    encoder_get_total_counts(&runtime->motor_b_total, &runtime->motor_a_total);
    {
        const straight_line_control_input_t control_input = {
            .elapsed_ms = runtime->elapsed_ms,
            .target_b_base_pwm = runtime->target_b_base_pwm,
            .target_a_base_pwm = runtime->target_a_base_pwm,
            .motor_b_delta = runtime->motor_b_delta,
            .motor_a_delta = runtime->motor_a_delta,
            .motor_b_total = runtime->motor_b_total,
            .motor_a_total = runtime->motor_a_total,
            .yaw_target_cdeg = runtime->yaw_target_cdeg,
            .nav_ok = runtime->nav_ok,
            .nav = nav
        };

        straight_line_apply_drive_control(config,
            &control_input,
            pid,
            drive_config,
            drive,
            &runtime->yaw_correction,
            &runtime->correction);
    }

    runtime->distance_count = drive->distance_count;
    runtime->ir_ok = IRTracking_ReadSample(sample);
}

/**
 * @brief 打印直线段启动诊断信息。
 */
static void straight_line_log_start(const straight_line_start_log_t *log)
{
    lc_printf("%s start: ctrl=task5 zero=%u ycorr=%u brake=%u rpt=%lu yaw0=%ld target=%ld fixed=%u B_base=%ld A_base=%ld ramp=%d/%d/%dms target_diff=%ld ff_gain=%ld ff_corr=%ld d_div=%ld d_max=%ld i_limit=%d corr_max=%ld arm=%ld force=%ld stop_min=%u nav0=%u\r\n",
        log->config->tag,
        log->config->zero_heading,
        log->config->yaw_corr_enable,
        log->config->entry_brake_enable,
        log->report_period_ms,
        log->yaw_start_cdeg,
        log->yaw_target_cdeg,
        log->config->fixed_yaw_target_enable,
        log->drive_config->base_b_pwm,
        log->drive_config->base_a_pwm,
        TASK1_RAMP_B_START_PWM,
        TASK1_RAMP_A_START_PWM,
        TASK1_START_RAMP_MS,
        log->drive_config->target_speed_diff,
        log->drive_config->diff_ff_gain,
        log->feedforward_correction,
        log->drive_config->distance_corr_divisor,
        log->drive_config->distance_corr_max,
        PID_TEST_I_LIMIT,
        log->drive_config->correction_max,
        log->config->line_arm_count,
        log->config->force_stop_count,
        log->config->stop_min_ir_count,
        log->nav_start_ok);
}

/**
 * @brief 打印一行周期性直线段诊断信息。
 */
static void straight_line_log_report(const straight_line_report_t *report)
{
    lc_printf("%s t=%lu dist=%ld arm=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u yaw=%ld gzlp=%ld ycorr=%ld B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld corr=%ld B_pwm=%ld A_pwm=%ld\r\n",
        report->config->tag,
        report->elapsed_ms,
        report->distance_count,
        report->ir_armed,
        (report->ir_ok != 0U) ? report->sample->raw : 0xFFU,
        (report->ir_ok != 0U) ? report->sample->line_mask : 0U,
        (report->ir_ok != 0U) ? report->sample->active_count : 0U,
        (report->ir_ok != 0U) ? report->sample->line_lost : 1U,
        report->ir_ok,
        report->nav_ok,
        (report->nav_ok != 0U) ? report->nav->yaw_relative_cdeg : 0,
        (report->nav_ok != 0U) ? report->nav->gyro_z_filtered_mdps : 0,
        report->yaw_correction,
        report->motor_b_total,
        report->motor_a_total,
        report->drive->distance_error,
        report->drive->distance_correction,
        report->drive->motor_b_speed,
        report->drive->motor_a_speed,
        report->drive_config->target_speed_diff,
        report->drive->pid_error,
        report->drive->p_term,
        report->drive->i_term,
        report->drive->d_term,
        report->drive->feedforward_correction,
        report->drive->feedback_correction,
        report->correction,
        report->drive->motor_b_pwm,
        report->drive->motor_a_pwm);
}

/**
 * @brief 处理直线段周期性诊断计数器。
 */
static void straight_line_log_runtime_report(
    const straight_line_segment_config_t *config,
    straight_line_runtime_t *runtime,
    const straight_drive_config_t *drive_config,
    const straight_drive_output_t *drive,
    const ir_tracking_sample_t *sample,
    const jy62_navigation_t *nav)
{
    if (runtime->report_elapsed_ms >= runtime->report_period_ms) {
        const straight_line_report_t report = {
            .config = config,
            .drive_config = drive_config,
            .drive = drive,
            .sample = sample,
            .nav = nav,
            .elapsed_ms = runtime->elapsed_ms,
            .distance_count = runtime->distance_count,
            .yaw_start_cdeg = runtime->yaw_start_cdeg,
            .yaw_target_cdeg = runtime->yaw_target_cdeg,
            .yaw_correction = runtime->yaw_correction,
            .correction = runtime->correction,
            .motor_b_total = runtime->motor_b_total,
            .motor_a_total = runtime->motor_a_total,
            .ir_armed = runtime->ir_armed,
            .ir_ok = runtime->ir_ok,
            .nav_ok = runtime->nav_ok
        };

        runtime->report_elapsed_ms = 0;
        straight_line_log_report(&report);
    }

    if (runtime->jy62_report_elapsed_ms >= JY62_TASK_REPORT_PERIOD_MS) {
        runtime->jy62_report_elapsed_ms = 0;
        jy62_print_navigation_line(config->tag, runtime->elapsed_ms);
    }
}

/**
 * @brief 打印直线段结束诊断信息。
 */
static void straight_line_log_stop(const straight_line_stop_log_t *log)
{
    lc_printf("%s stop: reason=%s t=%lu dist=%ld arm=%ld force=%ld raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u rel_cdeg=%ld B_total=%ld A_total=%ld\r\n",
        log->config->tag,
        straight_line_stop_reason_name(log->stop_reason),
        log->elapsed_ms,
        log->distance_count,
        log->config->line_arm_count,
        log->config->force_stop_count,
        (log->ir_ok != 0U) ? log->sample->raw : 0xFFU,
        (log->ir_ok != 0U) ? log->sample->line_mask : 0U,
        (log->ir_ok != 0U) ? log->sample->active_count : 0U,
        (log->ir_ok != 0U) ? log->sample->line_lost : 1U,
        log->ir_ok,
        log->stop_nav_ok,
        log->nav->yaw_relative_cdeg,
        log->motor_b_total,
        log->motor_a_total);
}

/**
 * @brief 直行直到识别到线、到达保护距离、收到停车命令或超时。
 *
 * @return 1 表示到线停车，2 表示距离保护，3 表示串口停车，0 表示超时。
 */
static uint8_t run_straight_to_line_segment(
    const straight_line_segment_config_t *config)
{
    straight_pid_t pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive = {0};
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    jy62_navigation_t nav_start = {0};
    straight_line_runtime_t runtime = {0};

    straight_line_prepare_start(config);

    IRTracking_Init();
    straight_drive_config_pid_test(&drive_config);
    runtime.target_b_base_pwm = drive_config.base_b_pwm;
    runtime.target_a_base_pwm = drive_config.base_a_pwm;
    straight_pid_reset(&pid);
    straight_pid_set_limits(&pid, PID_TEST_I_LIMIT, PID_TEST_CORR_MAX);
    runtime.feedforward_correction = straight_drive_feedforward(&drive_config);
#if ENABLE_JY62_NAV
    runtime.nav_start_ok = JY62_PeekNavigation(&nav_start);
    if (runtime.nav_start_ok != 0U) {
        runtime.yaw_start_cdeg = nav_start.yaw_relative_cdeg;
    }
#endif
    runtime.yaw_target_cdeg = (config->fixed_yaw_target_enable != 0U) ?
        normalize_cdeg(config->fixed_yaw_target_cdeg) : runtime.yaw_start_cdeg;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    runtime.report_period_ms = TASK1_REPORT_PERIOD_MS;
    {
        const straight_line_start_log_t start_log = {
            .config = config,
            .drive_config = &drive_config,
            .sample = &sample,
            .nav_start_ok = runtime.nav_start_ok,
            .report_period_ms = runtime.report_period_ms,
            .yaw_start_cdeg = runtime.yaw_start_cdeg,
            .yaw_target_cdeg = runtime.yaw_target_cdeg,
            .feedforward_correction = runtime.feedforward_correction
        };

        straight_line_log_start(&start_log);
    }
    TB6612_SetDifferential((int16_t)TASK1_RAMP_B_START_PWM,
        (int16_t)TASK1_RAMP_A_START_PWM);

    while (runtime.elapsed_ms < TASK1_MAX_RUN_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        runtime.elapsed_ms += CONTROL_PERIOD_MS;
        runtime.report_elapsed_ms += CONTROL_PERIOD_MS;
        runtime.jy62_report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            runtime.stop_reason = 3U;
            break;
        }

        straight_line_update_drive_step(config,
            &runtime,
            &pid,
            &drive_config,
            &drive,
            &sample,
            &nav);
        if (straight_line_detect_line_crossing(config,
            runtime.distance_count,
            runtime.ir_ok,
            &sample,
            &runtime.ir_armed) != 0U) {
            runtime.stop_reason = 1U;
            break;
        }
        if (runtime.distance_count >= config->force_stop_count) {
            runtime.stop_reason = 2U;
            break;
        }

        TB6612_SetDifferential((int16_t)drive.motor_b_pwm,
            (int16_t)drive.motor_a_pwm);

        straight_line_log_runtime_report(config,
            &runtime,
            &drive_config,
            &drive,
            &sample,
            &nav);
    }

    if ((config->stop_alarm_ms != 0U) || (runtime.stop_reason != 1U)) {
        TB6612_Brake();
    }
    encoder_get_total_counts(&runtime.motor_b_total, &runtime.motor_a_total);
    runtime.distance_count = motion_distance_count(runtime.motor_b_total,
        runtime.motor_a_total);
    runtime.stop_nav_ok = JY62_PeekNavigation(&nav);
    {
        const straight_line_stop_log_t stop_log = {
            .config = config,
            .sample = &sample,
            .nav = &nav,
            .elapsed_ms = runtime.elapsed_ms,
            .distance_count = runtime.distance_count,
            .yaw_target_cdeg = runtime.yaw_target_cdeg,
            .motor_b_total = runtime.motor_b_total,
            .motor_a_total = runtime.motor_a_total,
            .stop_reason = runtime.stop_reason,
            .ir_ok = runtime.ir_ok,
            .stop_nav_ok = runtime.stop_nav_ok
        };

        straight_line_log_stop(&stop_log);
    }
    if (config->stop_alarm_ms != 0U) {
        st011_pulse(config->stop_alarm_ms);
    }

    return runtime.stop_reason;
}

#endif
