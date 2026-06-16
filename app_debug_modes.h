#ifndef APP_DEBUG_MODES_H
#define APP_DEBUG_MODES_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "bsp_tb6612.h"

/**
 * @brief Lightweight stop detector used by standalone debug streams.
 */
static inline uint8_t debug_uart_stop_requested(void)
{
    static uint8_t seen_zero = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);

        if (ch == 0x00U) {
            seen_zero = 0U;
            return 1U;
        }
        if ((seen_zero != 0U) && (ch == '0')) {
            seen_zero = 0U;
            return 1U;
        }
        seen_zero = (ch == '0') ? 1U : 0U;
    }

    return 0U;
}

/* 主电机闭环调试。该函数会一直运行，并按设定周期打印 PID 数据。 */
/**
 * @brief Normalize centi-degree heading into the signed +/-180 degree range.
 */
static inline int32_t task5_normalize_cdeg(int32_t angle_cdeg)
{
    while (angle_cdeg > 18000L) {
        angle_cdeg -= 36000L;
    }
    while (angle_cdeg < -18000L) {
        angle_cdeg += 36000L;
    }
    return angle_cdeg;
}

/**
 * @brief Calculate optional yaw correction for Task5 PID speed testing.
 */
static inline int32_t task5_yaw_correction(uint8_t nav_ok,
    const jy62_navigation_t *nav,
    int32_t yaw_start_cdeg)
{
#if TASK5_YAW_CORR_ENABLE && ENABLE_JY62_NAV
    int32_t yaw_error_cdeg;
    int32_t correction;

    if ((nav_ok == 0U) || (nav == 0)) {
        return 0;
    }

    yaw_error_cdeg = task5_normalize_cdeg(nav->yaw_relative_cdeg - yaw_start_cdeg);
    if (abs_i32(yaw_error_cdeg) <= TASK5_YAW_DEADBAND_CDEG) {
        yaw_error_cdeg = 0;
    }

    correction = -(yaw_error_cdeg / TASK5_YAW_CORR_DIVISOR);
#if TASK5_YAW_GYRO_DAMP_DIVISOR > 0
    correction -= nav->gyro_z_filtered_mdps / TASK5_YAW_GYRO_DAMP_DIVISOR;
#endif

    return clamp_i32(correction, -TASK5_YAW_CORR_MAX, TASK5_YAW_CORR_MAX);
#else
    (void)nav_ok;
    (void)nav;
    (void)yaw_start_cdeg;
    return 0;
#endif
}

/**
 * @brief Encoder and averaged wheel-speed data for one Task5 RAM sample.
 */
typedef struct {
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t b_speed_avg;
    int32_t a_speed_avg;
    int32_t diff_avg;
} task5_motor_info_t;

/**
 * @brief JY62 state passed into Task5 logging and yaw correction.
 */
typedef struct {
    uint8_t nav_ok;
    const jy62_navigation_t *nav;
    uint32_t nav_frame_delta;
    int32_t yaw_start_cdeg;
    int32_t yaw_correction;
} task5_nav_info_t;

#if TASK5_RAM_LOG_ENABLE
/**
 * @brief Compact RAM record for one Task5 debug sample.
 */
typedef struct {
    uint32_t t_ms;
    int16_t motor_b_delta;
    int16_t motor_a_delta;
    int16_t motor_b_total;
    int16_t motor_a_total;
    int16_t distance_count;
    int16_t distance_error;
    int16_t distance_correction;
    int16_t motor_b_speed;
    int16_t motor_a_speed;
    int16_t speed_diff;
    int16_t motor_b_avg;
    int16_t motor_a_avg;
    int16_t diff_avg;
    int16_t pid_error;
    int16_t p_term;
    int16_t i_term;
    int16_t d_term;
    int16_t feedforward_correction;
    int16_t feedback_correction;
    int16_t yaw_correction;
    int16_t correction;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int16_t yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t gyro_z_x100_mdps;
    int16_t gzlp_x100_mdps;
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    uint16_t nav_frame_delta;
    uint8_t nav_ok;
    uint8_t nav_update_flags;
} task5_ram_log_t;

extern task5_ram_log_t * const g_task5_ram_log;
extern uint16_t g_task5_ram_log_count;
extern uint16_t g_task5_ram_log_overflow;

/**
 * @brief Saturate a 32-bit value for compact int16 RAM logging.
 */
static inline int16_t task5_sat_i16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

/**
 * @brief Saturate a 32-bit unsigned value for compact uint16 RAM logging.
 */
static inline uint16_t task5_sat_u16(uint32_t value)
{
    return (value > 65535UL) ? 65535U : (uint16_t)value;
}

/**
 * @brief Clear Task5 RAM log counters.
 */
static inline void task5_ram_log_reset(void)
{
    g_task5_ram_log_count = 0U;
    g_task5_ram_log_overflow = 0U;
}

/**
 * @brief Optional pacing delay between UART dump lines.
 */
static inline void task5_ram_dump_line_pause(void)
{
#if TASK5_DUMP_LINE_DELAY_MS > 0
    delay_ms(TASK5_DUMP_LINE_DELAY_MS);
#endif
}

/**
 * @brief Append one Task5 compact diagnostic record.
 */
static inline void task5_ram_log_sample(uint32_t elapsed_ms,
    const task5_motor_info_t *motor,
    const straight_drive_output_t *drive,
    const task5_nav_info_t *nav_info)
{
    task5_ram_log_t *log;
    int32_t yaw_cdeg = 0;
    int32_t yaw_progress_cdeg = 0;
    int32_t gyro_z_mdps = 0;
    int32_t gyro_z_filtered_mdps = 0;
    int32_t roll_cdeg = 0;
    int32_t pitch_cdeg = 0;
    uint8_t update_flags = 0U;

    if (g_task5_ram_log_count >= TASK5_RAM_LOG_CAPACITY) {
        g_task5_ram_log_overflow++;
        return;
    }

    if ((nav_info->nav_ok != 0U) && (nav_info->nav != 0)) {
        yaw_cdeg = nav_info->nav->yaw_relative_cdeg;
        yaw_progress_cdeg = task5_normalize_cdeg(yaw_cdeg -
            nav_info->yaw_start_cdeg);
        gyro_z_mdps = nav_info->nav->gyro_z_mdps;
        gyro_z_filtered_mdps = nav_info->nav->gyro_z_filtered_mdps;
        roll_cdeg = nav_info->nav->roll_cdeg;
        pitch_cdeg = nav_info->nav->pitch_cdeg;
        update_flags = nav_info->nav->update_flags;
    }

    log = &g_task5_ram_log[g_task5_ram_log_count++];
    log->t_ms = elapsed_ms;
    log->motor_b_delta = task5_sat_i16(motor->motor_b_delta);
    log->motor_a_delta = task5_sat_i16(motor->motor_a_delta);
    log->motor_b_total = task5_sat_i16(motor->motor_b_total);
    log->motor_a_total = task5_sat_i16(motor->motor_a_total);
    log->distance_count = task5_sat_i16(drive->distance_count);
    log->distance_error = task5_sat_i16(drive->distance_error);
    log->distance_correction = task5_sat_i16(drive->distance_correction);
    log->motor_b_speed = task5_sat_i16(drive->motor_b_speed);
    log->motor_a_speed = task5_sat_i16(drive->motor_a_speed);
    log->speed_diff = task5_sat_i16(drive->speed_diff);
    log->motor_b_avg = task5_sat_i16(motor->b_speed_avg);
    log->motor_a_avg = task5_sat_i16(motor->a_speed_avg);
    log->diff_avg = task5_sat_i16(motor->diff_avg);
    log->pid_error = task5_sat_i16(drive->pid_error);
    log->p_term = task5_sat_i16(drive->p_term);
    log->i_term = task5_sat_i16(drive->i_term);
    log->d_term = task5_sat_i16(drive->d_term);
    log->feedforward_correction = task5_sat_i16(drive->feedforward_correction);
    log->feedback_correction = task5_sat_i16(drive->feedback_correction);
    log->yaw_correction = task5_sat_i16(nav_info->yaw_correction);
    log->correction = task5_sat_i16(drive->correction);
    log->motor_b_pwm = task5_sat_i16(drive->motor_b_pwm);
    log->motor_a_pwm = task5_sat_i16(drive->motor_a_pwm);
    log->yaw_cdeg = task5_sat_i16(yaw_cdeg);
    log->yaw_progress_cdeg = task5_sat_i16(yaw_progress_cdeg);
    log->gyro_z_x100_mdps = task5_sat_i16(gyro_z_mdps / 100);
    log->gzlp_x100_mdps = task5_sat_i16(gyro_z_filtered_mdps / 100);
    log->roll_cdeg = task5_sat_i16(roll_cdeg);
    log->pitch_cdeg = task5_sat_i16(pitch_cdeg);
    log->nav_frame_delta = task5_sat_u16(nav_info->nav_frame_delta);
    log->nav_ok = nav_info->nav_ok;
    log->nav_update_flags = update_flags;
}

/**
 * @brief Dump the buffered Task5 RAM records over UART.
 */
static inline void task5_ram_log_dump(const straight_drive_config_t *config,
    int32_t feedforward_correction,
    uint8_t nav_start_ok,
    int32_t yaw_start_cdeg,
    int32_t motor_b_total,
    int32_t motor_a_total,
    int32_t distance_count)
{
    uint16_t i;
    uint32_t seq = 0U;
    uint16_t count = g_task5_ram_log_count;
    uint16_t overflow = g_task5_ram_log_overflow;

    if (count > TASK5_RAM_LOG_CAPACITY) {
        overflow++;
        count = TASK5_RAM_LOG_CAPACITY;
    }

    lc_printf("TASK5_RAM_BEGIN seq=%lu n=%u/%u ov=%u period=%ums final_B=%ld final_A=%ld final_dist=%ld final_d_err=%ld\r\n",
        (unsigned long)seq++,
        count,
        TASK5_RAM_LOG_CAPACITY,
        overflow,
        TASK5_RAM_LOG_PERIOD_MS,
        motor_b_total,
        motor_a_total,
        distance_count,
        motor_b_total - motor_a_total);
    task5_ram_dump_line_pause();

    lc_printf("TASK5_CFG seq=%lu B_base=%ld A_base=%ld target_diff=%ld ff_gain=%ld ff_corr=%ld d_div=%ld d_max=%ld i_limit=%d corr_max=%ld period=%dms yaw0=%ld nav0=%u yaw_en=%u yaw_db=%d yaw_div=%d yaw_max=%d yaw_gd=%d\r\n",
        (unsigned long)seq++,
        config->base_b_pwm,
        config->base_a_pwm,
        config->target_speed_diff,
        config->diff_ff_gain,
        feedforward_correction,
        config->distance_corr_divisor,
        config->distance_corr_max,
        PID_TEST_I_LIMIT,
        config->correction_max,
        CONTROL_PERIOD_MS,
        yaw_start_cdeg,
        nav_start_ok,
        TASK5_YAW_CORR_ENABLE,
        TASK5_YAW_DEADBAND_CDEG,
        TASK5_YAW_CORR_DIVISOR,
        TASK5_YAW_CORR_MAX,
        TASK5_YAW_GYRO_DAMP_DIVISOR);
    task5_ram_dump_line_pause();

    lc_printf("TASK5_DUMP_SECTION seq=%lu name=PID count=%u\r\n",
        (unsigned long)seq++,
        count);
    task5_ram_dump_line_pause();

    for (i = 0U; i < count; i++) {
        const task5_ram_log_t *log = &g_task5_ram_log[i];
        lc_printf("TASK5_PID seq=%lu idx=%u t=%lu B_cnt=%d A_cnt=%d B_total=%d A_total=%d dist_count=%d d_err=%d d_corr=%d B_spd=%d A_spd=%d diff=%d B_avg=%d A_avg=%d diff_avg=%d target_diff=%ld err=%d P=%d I=%d D=%d ff=%d fb=%d ycorr=%d corr=%d B_pwm=%d A_pwm=%d nav=%u yaw=%d yprog=%d gz=%d gzlp=%d roll=%d pitch=%d nav_fd=%u upd=0x%02X\r\n",
            (unsigned long)seq++,
            i,
            log->t_ms,
            log->motor_b_delta,
            log->motor_a_delta,
            log->motor_b_total,
            log->motor_a_total,
            log->distance_count,
            log->distance_error,
            log->distance_correction,
            log->motor_b_speed,
            log->motor_a_speed,
            log->speed_diff,
            log->motor_b_avg,
            log->motor_a_avg,
            log->diff_avg,
            config->target_speed_diff,
            log->pid_error,
            log->p_term,
            log->i_term,
            log->d_term,
            log->feedforward_correction,
            log->feedback_correction,
            log->yaw_correction,
            log->correction,
            log->motor_b_pwm,
            log->motor_a_pwm,
            log->nav_ok,
            log->yaw_cdeg,
            log->yaw_progress_cdeg,
            log->gyro_z_x100_mdps,
            log->gzlp_x100_mdps,
            log->roll_cdeg,
            log->pitch_cdeg,
            log->nav_frame_delta,
            log->nav_update_flags);
        task5_ram_dump_line_pause();
    }

    lc_printf("TASK5_DUMP_SECTION_END seq=%lu name=PID\r\n",
        (unsigned long)seq++);
    task5_ram_dump_line_pause();
    lc_printf("TASK5_RAM_END seq=%lu\r\n", (unsigned long)seq++);
}
#else
#define task5_ram_log_reset() ((void)0)
#define task5_ram_log_sample(elapsed_ms, motor, drive, nav_info) ((void)0)
#define task5_ram_log_dump(config, feedforward_correction, nav_start_ok, yaw_start_cdeg, motor_b_total, motor_a_total, distance_count) ((void)0)
#endif

/**
 * @brief UART 05 debug mode: run full PID wheel-speed stream until stop.
 */
static inline void run_motor_pid_stream(void)
{
    straight_pid_t pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms = 0;
    uint32_t log_elapsed_ms = 0;
    int32_t log_b_speed_sum = 0;
    int32_t log_a_speed_sum = 0;
    uint32_t log_sample_count = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t distance_count;
    int32_t feedforward_correction;
    jy62_navigation_t nav = {0};
    jy62_navigation_t nav_start = {0};
    uint32_t nav_frame_delta = 0U;
    uint8_t nav_ok = 0U;
    uint8_t nav_start_ok = 0U;
    int32_t yaw_start_cdeg = 0;
    int32_t yaw_correction = 0;

    straight_drive_config_pid_test(&drive_config);
    straight_pid_reset(&pid);
    straight_pid_set_limits(&pid, PID_TEST_I_LIMIT, PID_TEST_CORR_MAX);
    feedforward_correction = straight_drive_feedforward(&drive_config);
    task5_ram_log_reset();
#if ENABLE_JY62_NAV
    nav_start_ok = JY62_PeekNavigation(&nav_start);
    if (nav_start_ok != 0U) {
        yaw_start_cdeg = nav_start.yaw_relative_cdeg;
    }
#endif

    /* 每次进入 05 模式都从 0 开始累计，方便标定 COUNTS_PER_CM。 */
    encoder_reset_distance_counts();
    lc_printf("PID motor stream start: B_base=%d A_base=%d target_diff=%d ff_gain=%d ff_corr=%ld d_div=%d d_max=%d i_limit=%d corr_max=%d period=%dms ram_period=%dms ram_cap=%u yaw0=%ld nav0=%u yaw_en=%u yaw_div=%d yaw_max=%d\r\n",
        drive_config.base_b_pwm,
        drive_config.base_a_pwm,
        drive_config.target_speed_diff,
        drive_config.diff_ff_gain,
        feedforward_correction,
        drive_config.distance_corr_divisor,
        drive_config.distance_corr_max,
        PID_TEST_I_LIMIT,
        drive_config.correction_max,
        CONTROL_PERIOD_MS,
        TASK5_RAM_LOG_PERIOD_MS,
        TASK5_RAM_LOG_CAPACITY,
        yaw_start_cdeg,
        nav_start_ok,
        TASK5_YAW_CORR_ENABLE,
        TASK5_YAW_CORR_DIVISOR,
        TASK5_YAW_CORR_MAX);
    encoder_enable_interrupts();
    lc_printf("PID encoder IRQ enabled, B=PA14/PA15 A=PA16/PA17\r\n");
    TB6612_SetDifferential((int16_t)drive_config.base_b_pwm,
        (int16_t)drive_config.base_a_pwm);

    while (1) {
        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        log_elapsed_ms += CONTROL_PERIOD_MS;

        if (debug_uart_stop_requested() != 0U) {
            TB6612_Brake();
            encoder_get_total_counts(&motor_b_total, &motor_a_total);
            distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
            lc_printf("PID motor stream stop: UART0 00 B_total=%ld A_total=%ld dist_count=%ld d_err=%ld\r\n",
                motor_b_total,
                motor_a_total,
                distance_count,
                motor_b_total - motor_a_total);
            task5_ram_log_dump(&drive_config,
                feedforward_correction,
                nav_start_ok,
                yaw_start_cdeg,
                motor_b_total,
                motor_a_total,
                distance_count);
            return;
        }

        /* 用 20 ms 时间窗口内的编码器增量作为简化速度估计。 */
#if ENABLE_JY62_NAV
        nav_frame_delta = JY62_GetNavigation(&nav);
        nav_ok = nav.valid;
#else
        nav_frame_delta = 0U;
        nav_ok = 0U;
#endif

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        straight_drive_update(&pid,
            &drive_config,
            motor_b_delta,
            motor_a_delta,
            motor_b_total,
            motor_a_total,
            &drive);
        yaw_correction = task5_yaw_correction(nav_ok, &nav, yaw_start_cdeg);
        drive.correction = clamp_i32(drive.correction + yaw_correction,
            -drive_config.correction_max,
            drive_config.correction_max);
        drive.motor_b_pwm = clamp_i32(drive_config.base_b_pwm - drive.correction,
            drive_config.min_pwm,
            drive_config.max_pwm);
        drive.motor_a_pwm = clamp_i32(drive_config.base_a_pwm + drive.correction,
            drive_config.min_pwm,
            drive_config.max_pwm);
        log_b_speed_sum += drive.motor_b_speed;
        log_a_speed_sum += drive.motor_a_speed;
        log_sample_count++;

        /* TB6612_SetDifferential(left, right)：左轮对应 B 电机，右轮对应 A 电机。 */
        TB6612_SetDifferential((int16_t)drive.motor_b_pwm,
            (int16_t)drive.motor_a_pwm);

        if (log_elapsed_ms >= TASK5_RAM_LOG_PERIOD_MS) {
            int32_t b_speed_avg = (log_sample_count != 0U) ?
                (log_b_speed_sum / (int32_t)log_sample_count) : drive.motor_b_speed;
            int32_t a_speed_avg = (log_sample_count != 0U) ?
                (log_a_speed_sum / (int32_t)log_sample_count) : drive.motor_a_speed;
            int32_t diff_avg = b_speed_avg - a_speed_avg;
            const task5_motor_info_t motor_info = {
                .motor_b_delta = motor_b_delta,
                .motor_a_delta = motor_a_delta,
                .motor_b_total = motor_b_total,
                .motor_a_total = motor_a_total,
                .b_speed_avg = b_speed_avg,
                .a_speed_avg = a_speed_avg,
                .diff_avg = diff_avg
            };
            const task5_nav_info_t nav_info = {
                .nav_ok = nav_ok,
                .nav = &nav,
                .nav_frame_delta = nav_frame_delta,
                .yaw_start_cdeg = yaw_start_cdeg,
                .yaw_correction = yaw_correction
            };

            task5_ram_log_sample(elapsed_ms,
                &motor_info,
                &drive,
                &nav_info);
            log_elapsed_ms = 0;
            log_b_speed_sum = 0;
            log_a_speed_sum = 0;
            log_sample_count = 0;
        }
    }
}

/* 07 差速 PD 调参：关闭积分，保留目标差速数学前馈，累计距离差只作观测。 */
/**
 * @brief UART 07 debug mode: run PD-only wheel-speed stream until stop.
 */
static inline void run_motor_pd_stream(void)
{
    straight_pid_t pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive;
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t report_b_speed_sum = 0;
    int32_t report_a_speed_sum = 0;
    uint32_t report_sample_count = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t distance_count;
    int32_t feedforward_correction;

    straight_drive_config_pd_test(&drive_config);
    straight_pid_reset(&pid);
    pid.kp = PD_TEST_KP;
    pid.ki = 0;
    pid.kd = PD_TEST_KD;
    pid.i_limit = 0;
    pid.corr_max = PD_TEST_CORR_MAX;
    pid.integral = 0;
    pid.last_error = 0;
    feedforward_correction = straight_drive_feedforward(&drive_config);

    encoder_reset_distance_counts();
    lc_printf("PD motor stream start: B_base=%d A_base=%d target_diff=%d ff_gain=%d ff_corr=%ld d_div=%d d_max=%d kp=%d kd=%d corr_max=%d period=%dms report=%dms\r\n",
        drive_config.base_b_pwm,
        drive_config.base_a_pwm,
        drive_config.target_speed_diff,
        drive_config.diff_ff_gain,
        feedforward_correction,
        drive_config.distance_corr_divisor,
        drive_config.distance_corr_max,
        PD_TEST_KP,
        PD_TEST_KD,
        drive_config.correction_max,
        CONTROL_PERIOD_MS,
        PID_REPORT_PERIOD_MS);
    encoder_enable_interrupts();
    lc_printf("PD encoder IRQ enabled, B=PA14/PA15 A=PA16/PA17\r\n");
    TB6612_SetDifferential((int16_t)drive_config.base_b_pwm,
        (int16_t)drive_config.base_a_pwm);

    while (1) {
        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (debug_uart_stop_requested() != 0U) {
            TB6612_Brake();
            encoder_get_total_counts(&motor_b_total, &motor_a_total);
            distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
            lc_printf("PD motor stream stop: UART0 00 B_total=%ld A_total=%ld dist_count=%ld d_err=%ld\r\n",
                motor_b_total,
                motor_a_total,
                distance_count,
                motor_b_total - motor_a_total);
            return;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        straight_drive_update(&pid,
            &drive_config,
            motor_b_delta,
            motor_a_delta,
            motor_b_total,
            motor_a_total,
            &drive);
        report_b_speed_sum += drive.motor_b_speed;
        report_a_speed_sum += drive.motor_a_speed;
        report_sample_count++;

        TB6612_SetDifferential((int16_t)drive.motor_b_pwm,
            (int16_t)drive.motor_a_pwm);

        if (report_elapsed_ms >= PID_REPORT_PERIOD_MS) {
            int32_t b_speed_avg = (report_sample_count != 0U) ?
                (report_b_speed_sum / (int32_t)report_sample_count) : drive.motor_b_speed;
            int32_t a_speed_avg = (report_sample_count != 0U) ?
                (report_a_speed_sum / (int32_t)report_sample_count) : drive.motor_a_speed;
            int32_t diff_avg = b_speed_avg - a_speed_avg;

            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_sample_count = 0;
            lc_printf("PD t=%lu B_cnt=%ld A_cnt=%ld B_total=%ld A_total=%ld dist_count=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld diff=%ld B_avg=%ld A_avg=%ld diff_avg=%ld target_diff=%d err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld corr=%ld B_pwm=%ld A_pwm=%ld\r\n",
                elapsed_ms,
                motor_b_delta, motor_a_delta,
                motor_b_total,
                motor_a_total,
                drive.distance_count,
                drive.distance_error,
                drive.distance_correction,
                drive.motor_b_speed, drive.motor_a_speed,
                drive.speed_diff,
                b_speed_avg,
                a_speed_avg,
                diff_avg,
                drive_config.target_speed_diff,
                drive.pid_error, drive.p_term, drive.i_term, drive.d_term,
                drive.feedforward_correction,
                drive.feedback_correction,
                drive.correction,
                drive.motor_b_pwm, drive.motor_a_pwm);
        }
    }
}

/* 八路红外循迹调试：根据红外误差直接差速转向。 */
/**
 * @brief Convert IR line error into differential PWM turn amount.
 */
static inline int32_t line_follow_calculate_turn(int32_t error)
{
    int32_t turn = (error * LINE_FOLLOW_TURN_SIGN) / LINE_FOLLOW_TURN_DIVISOR;

    return clamp_i32(turn, -LINE_FOLLOW_TURN_LIMIT, LINE_FOLLOW_TURN_LIMIT);
}

/**
 * @brief Continuous pure-IR line-follow debug mode.
 */
static inline void run_line_follow_test(void)
{
    ir_tracking_sample_t sample;
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;

    IRTracking_Init();
    TB6612_Brake();
    delay_ms(300);

    lc_printf("\r\nIR line follow start\r\n");
    lc_printf("base=%d max=%d turn_div=%d turn_limit=%d report=%dms\r\n",
        LINE_FOLLOW_BASE_PWM,
        LINE_FOLLOW_MAX_PWM,
        LINE_FOLLOW_TURN_DIVISOR,
        LINE_FOLLOW_TURN_LIMIT,
        LINE_FOLLOW_REPORT_PERIOD_MS);
    lc_printf("L=base+turn R=base-turn, err<0 line left, err>0 line right\r\n");

    while (1) {
        int32_t turn;
        int32_t left_pwm;
        int32_t right_pwm;

        delay_ms(LINE_FOLLOW_PERIOD_MS);
        elapsed_ms += LINE_FOLLOW_PERIOD_MS;
        report_elapsed_ms += LINE_FOLLOW_PERIOD_MS;

        if (IRTracking_ReadSample(&sample) == 0U) {
            TB6612_Brake();

            if (report_elapsed_ms >= LINE_FOLLOW_REPORT_PERIOD_MS) {
                report_elapsed_ms = 0;
                lc_printf("LINE t=%lu read_fail=1 L=0 R=0 brake=1\r\n", elapsed_ms);
            }

            continue;
        }

        if (sample.line_lost != 0U) {
            TB6612_Brake();

            if (report_elapsed_ms >= LINE_FOLLOW_REPORT_PERIOD_MS) {
                report_elapsed_ms = 0;
                lc_printf("LINE t=%lu raw=0x%02X mask=0x%02X cnt=%u lost=1 err=%d L=0 R=0 brake=1\r\n",
                    elapsed_ms,
                    sample.raw,
                    sample.line_mask,
                    sample.active_count,
                    sample.error);
            }

            continue;
        }

        turn = line_follow_calculate_turn(sample.error);
        left_pwm = clamp_i32(LINE_FOLLOW_BASE_PWM + turn,
            LINE_FOLLOW_MIN_PWM, LINE_FOLLOW_MAX_PWM);
        right_pwm = clamp_i32(LINE_FOLLOW_BASE_PWM - turn,
            LINE_FOLLOW_MIN_PWM, LINE_FOLLOW_MAX_PWM);

        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (report_elapsed_ms >= LINE_FOLLOW_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("LINE t=%lu raw=0x%02X mask=0x%02X cnt=%u lost=0 err=%d target=%ld turn=%ld L=%ld R=%ld\r\n",
                elapsed_ms,
                sample.raw,
                sample.line_mask,
                sample.active_count,
                sample.error,
                turn,
                turn,
                left_pwm,
                right_pwm);
        }
    }
}

/* 八路红外模块串口打印测试：只读 I2C 并打印，不驱动电机。 */
/**
 * @brief Read-only IR tracking UART print test.
 */
static inline void run_ir_tracking_uart_test(void)
{
    ir_tracking_sample_t sample;
    uint32_t elapsed_ms = 0;

    IRTracking_Init();
    TB6612_Brake();

    lc_printf("\r\nIR tracking UART test start\r\n");
    lc_printf("I2C: addr=0x%02X reg=0x%02X SDA=PB3 SCL=PB2 period=%dms\r\n",
        IR_TRACKING_I2C_ADDR, IR_TRACKING_DATA_REG, IR_TRACKING_TEST_PERIOD_MS);
    lc_printf("raw bit7..bit0 = X1..X8, mask bit0..bit7 = X1..X8 black line\r\n");
    lc_printf("error: left negative, center zero, right positive\r\n");

    while (1) {
        if (IRTracking_ReadSample(&sample) != 0U) {
            lc_printf("IR t=%lu ", elapsed_ms);
            IRTracking_PrintSample(&sample);
        } else {
            lc_printf("IR t=%lu read failed, check VCC/GND/SDA/SCL/I2C address\r\n", elapsed_ms);
        }

        delay_ms(IR_TRACKING_TEST_PERIOD_MS);
        elapsed_ms += IR_TRACKING_TEST_PERIOD_MS;
    }
}

#endif
