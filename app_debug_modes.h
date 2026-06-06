#ifndef APP_DEBUG_MODES_H
#define APP_DEBUG_MODES_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
#include "app_straight.h"
#include "board.h"
#include "bsp_encoder.h"
#include "bsp_ir_tracking.h"
#include "bsp_tb6612.h"

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
static inline void run_motor_pid_stream(void)
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

    straight_drive_config_pid_test(&drive_config);
    straight_pid_reset(&pid);
    straight_pid_set_limits(&pid, PID_TEST_I_LIMIT, PID_TEST_CORR_MAX);
    feedforward_correction = straight_drive_feedforward(&drive_config);

    /* 每次进入 05 模式都从 0 开始累计，方便标定 COUNTS_PER_CM。 */
    encoder_reset_distance_counts();
    lc_printf("PID motor stream start: B_base=%d A_base=%d target_diff=%d ff_gain=%d ff_corr=%ld d_div=%d d_max=%d i_limit=%d corr_max=%d period=%dms report=%dms\r\n",
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
        PID_REPORT_PERIOD_MS);
    encoder_enable_interrupts();
    lc_printf("PID encoder IRQ enabled, B=PA14/PA15 A=PA16/PA17\r\n");
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
            lc_printf("PID motor stream stop: UART0 00 B_total=%ld A_total=%ld dist_count=%ld d_err=%ld\r\n",
                motor_b_total,
                motor_a_total,
                distance_count,
                motor_b_total - motor_a_total);
            return;
        }

        /* 用 20 ms 时间窗口内的编码器增量作为简化速度估计。 */
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

        /* TB6612_SetDifferential(left, right)：左轮对应 B 电机，右轮对应 A 电机。 */
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
            lc_printf("PID t=%lu B_cnt=%ld A_cnt=%ld B_total=%ld A_total=%ld dist_count=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld diff=%ld B_avg=%ld A_avg=%ld diff_avg=%ld target_diff=%d err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld corr=%ld B_pwm=%ld A_pwm=%ld\r\n",
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

/* 07 差速 PD 调参：关闭积分，保留目标差速数学前馈，累计距离差只作观测。 */
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
static inline int32_t line_follow_calculate_turn(int32_t error)
{
    int32_t turn = (error * LINE_FOLLOW_TURN_SIGN) / LINE_FOLLOW_TURN_DIVISOR;

    return clamp_i32(turn, -LINE_FOLLOW_TURN_LIMIT, LINE_FOLLOW_TURN_LIMIT);
}

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


/* UART 10/11：纯红外跑图测试，保持和任务三、任务四的正式逻辑独立。 */
typedef struct {
    uint8_t reason;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    ir_tracking_sample_t sample;
} task11_line_result_t;

static inline const char *task11_reason_name(uint8_t reason)
{
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
        return "distance";
    }
    return "timeout";
}

static inline const char *task11_phase_name(uint8_t phase)
{
    static const char * const names[4] = {
        "AC",
        "CB",
        "BD",
        "DA"
    };

    return (phase < 4U) ? names[phase] : "UNK";
}

static inline uint8_t task11_mask_count(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count += (uint8_t)(value & 0x01U);
        value >>= 1;
    }

    return count;
}

static inline uint8_t task11_peek_yaw(int32_t *yaw_cdeg, int32_t *gzlp_mdps)
{
    jy62_navigation_t nav;
    uint8_t nav_ok = JY62_PeekNavigation(&nav);

    if (nav_ok != 0U) {
        *yaw_cdeg = nav.yaw_relative_cdeg;
        *gzlp_mdps = nav.gyro_z_filtered_mdps;
    } else {
        *yaw_cdeg = 0;
        *gzlp_mdps = 0;
    }

    return nav_ok;
}

static inline void task11_print_point_lap(uint8_t lap,
    const char *segment,
    const char *name,
    const task11_line_result_t *result)
{
    lc_printf("TASK11_POINT lap=%u seg=%s name=%s reason=%s t=%lu dist=%ld yaw=%ld yprog=%ld mask=0x%02X cnt=%u lost=%u err=%ld\r\n",
        lap,
        segment,
        name,
        task11_reason_name(result->reason),
        result->elapsed_ms,
        result->distance_count,
        result->yaw_cdeg,
        result->yaw_progress_cdeg,
        (result->ir_ok != 0U) ? result->sample.line_mask : 0U,
        (result->ir_ok != 0U) ? result->sample.active_count : 0U,
        (result->ir_ok != 0U) ? result->sample.line_lost : 1U,
        (result->ir_ok != 0U) ? result->sample.error : 0);
}

static inline void run_task10_ab_zero_test(void)
{
    int32_t yaw_cdeg;
    int32_t gzlp_mdps;
    uint8_t ok;
    uint8_t nav_ok;

    TB6612_Brake();
    ok = jy62_zero_to_current("TASK10_AB_ZERO", 0U);
    nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
    lc_printf("TASK10 zero_ab: ok=%u nav=%u rel=%ld gzlp=%ld\r\n",
        ok,
        nav_ok,
        yaw_cdeg,
        gzlp_mdps);
}

static inline void run_task11_ir_map_test(void)
{
    task11_line_result_t result;
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t report_b_speed_sum = 0;
    int32_t report_a_speed_sum = 0;
    int32_t report_speed_diff_sum = 0;
    uint32_t report_sample_count = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t phase_start_count = 0;
    int32_t filtered_error = 0;
    int32_t last_filtered_error = 0;
    int32_t last_turn = 0;
    int32_t yaw_start = 0;
    int32_t yaw_cdeg;
    int32_t gzlp_mdps;
    int32_t yaw_progress_cdeg = 0;
    uint8_t lap_count = 0U;
    uint8_t phase = 0U;
    uint8_t nav_ok;
    uint8_t exit_line_seen = 0U;
    uint8_t straight_point_confirm = 0U;
    uint8_t arc_point_confirm = 0U;
    uint8_t ir_ok = 0U;
    uint8_t stop_reason = 0U;

    TB6612_Brake();
    delay_ms_with_st011(TASK11_POINT_SETTLE_MS);
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = task11_peek_yaw(&yaw_start, &gzlp_mdps);
    yaw_cdeg = yaw_start;
    lc_printf("TASK11 start: pure_ir laps=%u yaw=%ld nav=%u straight=%d/%d arc=%d/%d dstart=%d report=%d/%d data=PD+wheel\r\n",
        TASK11_TARGET_LAPS,
        yaw_cdeg,
        nav_ok,
        TASK11_STRAIGHT_B_BASE_PWM,
        TASK11_STRAIGHT_A_BASE_PWM,
        TASK11_ARC_B_BASE_PWM,
        TASK11_ARC_A_BASE_PWM,
        TASK11_STRAIGHT_ARC_START_COUNT,
        TASK11_LINE_REPORT_PERIOD_MS,
        TASK11_ARC_REPORT_PERIOD_MS);

    while ((elapsed_ms < TASK11_TOTAL_MAX_RUN_MS) && (lap_count < TASK11_TARGET_LAPS)) {
        int32_t total_distance_count;
        int32_t phase_distance_count;
        int32_t motor_b_speed;
        int32_t motor_a_speed;
        int32_t speed_diff;
        int32_t distance_error;
        int32_t raw_error = 0;
        int32_t derivative = 0;
        int32_t p_component = 0;
        int32_t d_component = 0;
        int32_t turn = 0;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t pwm_max;
        int32_t kp_divisor;
        int32_t kd_divisor;
        int32_t derivative_limit;
        int32_t turn_limit;
        int32_t lost_base_drop;
        int32_t lost_turn_limit;
        int32_t left_pwm;
        int32_t right_pwm;
        int32_t pwm_diff;
        int32_t point_arm_count;
        int32_t force_count;
        int32_t yaw_arm_cdeg;
        int32_t report_period_ms;
        int32_t expected_turn_dir;
        const char *point_name;
        const char *force_name;
        const char *distance_name;
        const char *segment_name;
        uint8_t arc_mode;
        uint8_t line_valid;
        uint8_t left_count;
        uint8_t center_count;
        uint8_t right_count;
        uint8_t left_seen;
        uint8_t center_seen;
        uint8_t right_seen;
        uint8_t expected_side_seen;
        uint8_t straight_wide_cross;
        uint8_t straight_cross_seen;
        uint8_t straight_point_side_seen;
        uint8_t straight_point_candidate;
        uint8_t arc_point_candidate;
        uint8_t distance_arc_start_ready;
        uint8_t point_ready = 0U;

        if (phase == 0U) {
            point_name = "C_LINE";
            force_name = "C_FORCE";
            distance_name = "C_START";
            arc_mode = 0U;
            expected_turn_dir = 0;
            point_arm_count = TASK11_STRAIGHT_POINT_ARM_COUNT;
            force_count = TASK11_STRAIGHT_FORCE_COUNT;
            yaw_arm_cdeg = 0;
        } else if (phase == 1U) {
            point_name = "B_EXIT";
            force_name = "B_FORCE";
            distance_name = "B_START";
            arc_mode = 1U;
            expected_turn_dir = TASK3_ARC_TURN_LEFT;
            point_arm_count = TASK11_ARC_POINT_ARM_COUNT;
            force_count = TASK11_ARC_FORCE_COUNT;
            yaw_arm_cdeg = TASK11_ARC_POINT_YAW_ARM_CDEG;
        } else if (phase == 2U) {
            point_name = "D_LINE";
            force_name = "D_FORCE";
            distance_name = "D_START";
            arc_mode = 0U;
            expected_turn_dir = 0;
            point_arm_count = TASK11_STRAIGHT_POINT_ARM_COUNT;
            force_count = TASK11_STRAIGHT_FORCE_COUNT;
            yaw_arm_cdeg = 0;
        } else {
            point_name = "A_FINISH";
            force_name = "A_FORCE";
            distance_name = "A_START";
            arc_mode = 1U;
            expected_turn_dir = TASK3_ARC_TURN_RIGHT;
            point_arm_count = TASK11_ARC_POINT_ARM_COUNT;
            force_count = TASK11_ARC_FORCE_COUNT;
            yaw_arm_cdeg = TASK11_ARC_POINT_YAW_ARM_CDEG;
        }

        segment_name = task11_phase_name(phase);
        report_period_ms = (arc_mode != 0U) ?
            TASK11_ARC_REPORT_PERIOD_MS : TASK11_LINE_REPORT_PERIOD_MS;
        if (arc_mode != 0U) {
            base_b_pwm = TASK11_ARC_B_BASE_PWM;
            base_a_pwm = TASK11_ARC_A_BASE_PWM;
            pwm_max = TASK11_ARC_MAX_PWM;
            kp_divisor = TASK11_ARC_KP_DIVISOR;
            kd_divisor = TASK11_ARC_KD_DIVISOR;
            derivative_limit = TASK11_ARC_DERIV_LIMIT;
            turn_limit = TASK11_ARC_TOTAL_TURN_LIMIT;
            lost_base_drop = TASK11_ARC_LOST_BASE_DROP;
            lost_turn_limit = TASK11_ARC_LOST_TURN;
        } else {
            base_b_pwm = TASK11_STRAIGHT_B_BASE_PWM;
            base_a_pwm = TASK11_STRAIGHT_A_BASE_PWM;
            pwm_max = TASK11_STRAIGHT_MAX_PWM;
            kp_divisor = TASK11_STRAIGHT_KP_DIVISOR;
            kd_divisor = TASK11_STRAIGHT_KD_DIVISOR;
            derivative_limit = TASK11_STRAIGHT_DERIV_LIMIT;
            turn_limit = TASK11_STRAIGHT_TURN_LIMIT;
            lost_base_drop = TASK11_STRAIGHT_LOST_BASE_DROP;
            lost_turn_limit = TASK11_STRAIGHT_LOST_TURN;
        }

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (debug_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        total_distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        phase_distance_count = total_distance_count - phase_start_count;
        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);
        speed_diff = motor_b_speed - motor_a_speed;
        distance_error = motor_b_total - motor_a_total;
        report_b_speed_sum += motor_b_speed;
        report_a_speed_sum += motor_a_speed;
        report_speed_diff_sum += speed_diff;
        report_sample_count++;
        nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
        yaw_progress_cdeg = (nav_ok != 0U) ? abs_i32(normalize_cdeg(yaw_cdeg - yaw_start)) : 0;
        ir_ok = IRTracking_ReadSample(&sample);
        line_valid = ((ir_ok != 0U) && (sample.line_lost == 0U)) ? 1U : 0U;
        left_count = (ir_ok != 0U) ?
            task11_mask_count((uint8_t)(sample.line_mask & TASK11_LEFT_SENSOR_MASK)) : 0U;
        center_count = (ir_ok != 0U) ?
            task11_mask_count((uint8_t)(sample.line_mask & TASK11_CENTER_SENSOR_MASK)) : 0U;
        right_count = (ir_ok != 0U) ?
            task11_mask_count((uint8_t)(sample.line_mask & TASK11_RIGHT_SENSOR_MASK)) : 0U;
        left_seen = (left_count != 0U) ? 1U : 0U;
        center_seen = (center_count != 0U) ? 1U : 0U;
        right_seen = (right_count != 0U) ? 1U : 0U;
        expected_side_seen = (expected_turn_dir < 0) ? left_seen :
            ((expected_turn_dir > 0) ? right_seen : 0U);
        straight_wide_cross = ((arc_mode == 0U) &&
            (line_valid != 0U) &&
            (sample.active_count >= TASK11_CROSS_ACTIVE_COUNT)) ? 1U : 0U;
        straight_cross_seen = ((straight_wide_cross != 0U) &&
            (phase_distance_count < TASK11_STRAIGHT_CROSS_IGNORE_COUNT)) ? 1U : 0U;
        straight_point_side_seen = (phase == 0U) ? left_seen :
            ((phase == 2U) ? right_seen : 0U);

        if ((arc_mode != 0U) &&
            (phase_distance_count >= point_arm_count) &&
            (expected_side_seen != 0U)) {
            exit_line_seen = 1U;
        }

        if (arc_mode == 0U) {
            straight_point_candidate = ((phase_distance_count >= point_arm_count) &&
                (straight_cross_seen == 0U) &&
                (straight_wide_cross == 0U) &&
                (line_valid != 0U) &&
                (straight_point_side_seen != 0U) &&
                ((abs_i32(sample.error) >= TASK11_STRAIGHT_POINT_ERROR_MIN) ||
                 (sample.active_count >= TASK11_STRAIGHT_POINT_WIDE_COUNT))) ? 1U : 0U;
            if (straight_point_candidate != 0U) {
                if (straight_point_confirm < TASK11_STRAIGHT_POINT_CONFIRM_COUNT) {
                    straight_point_confirm++;
                }
            } else {
                straight_point_confirm = 0U;
            }
            point_ready = (straight_point_confirm >= TASK11_STRAIGHT_POINT_CONFIRM_COUNT) ? 1U : 0U;
        } else {
            arc_point_candidate = ((phase_distance_count >= point_arm_count) &&
                (yaw_progress_cdeg >= yaw_arm_cdeg) &&
                (exit_line_seen != 0U) &&
                ((line_valid == 0U) ||
                 (sample.active_count >= TASK11_ARC_POINT_WIDE_COUNT) ||
                 (expected_side_seen == 0U) ||
                 (center_seen != 0U))) ? 1U : 0U;
            if (arc_point_candidate != 0U) {
                if (arc_point_confirm < TASK11_ARC_POINT_CONFIRM_COUNT) {
                    arc_point_confirm++;
                }
            } else {
                arc_point_confirm = 0U;
            }
            point_ready = (arc_point_confirm >= TASK11_ARC_POINT_CONFIRM_COUNT) ? 1U : 0U;
        }
        distance_arc_start_ready = ((arc_mode == 0U) &&
            (phase_distance_count >= TASK11_STRAIGHT_ARC_START_COUNT)) ? 1U : 0U;

        if (point_ready != 0U) {
            result.reason = 1U;
            result.elapsed_ms = elapsed_ms;
            result.distance_count = phase_distance_count;
            result.yaw_cdeg = yaw_cdeg;
            result.yaw_progress_cdeg = yaw_progress_cdeg;
            result.ir_ok = ir_ok;
            result.sample = sample;
            task11_print_point_lap(lap_count, segment_name, point_name, &result);
            st011_start_pulse(TASK11_POINT_ALARM_MS);

            if (phase == 3U) {
                lap_count++;
                if (lap_count >= TASK11_TARGET_LAPS) {
                    stop_reason = 1U;
                    break;
                }
                phase = 0U;
            } else {
                phase++;
            }
            phase_start_count = total_distance_count;
            yaw_start = yaw_cdeg;
            exit_line_seen = 0U;
            straight_point_confirm = 0U;
            arc_point_confirm = 0U;
            filtered_error = 0;
            last_filtered_error = 0;
            last_turn = 0;
            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_speed_diff_sum = 0;
            report_sample_count = 0;
            continue;
        } else if (distance_arc_start_ready != 0U) {
            result.reason = 6U;
            result.elapsed_ms = elapsed_ms;
            result.distance_count = phase_distance_count;
            result.yaw_cdeg = yaw_cdeg;
            result.yaw_progress_cdeg = yaw_progress_cdeg;
            result.ir_ok = ir_ok;
            result.sample = sample;
            task11_print_point_lap(lap_count, segment_name, distance_name, &result);

            phase++;
            phase_start_count = total_distance_count;
            yaw_start = yaw_cdeg;
            exit_line_seen = 0U;
            straight_point_confirm = 0U;
            arc_point_confirm = 0U;
            filtered_error = 0;
            last_filtered_error = 0;
            last_turn = 0;
            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_speed_diff_sum = 0;
            report_sample_count = 0;
            continue;
        } else if (phase_distance_count >= force_count) {
            result.reason = 2U;
            result.elapsed_ms = elapsed_ms;
            result.distance_count = phase_distance_count;
            result.yaw_cdeg = yaw_cdeg;
            result.yaw_progress_cdeg = yaw_progress_cdeg;
            result.ir_ok = ir_ok;
            result.sample = sample;
            task11_print_point_lap(lap_count, segment_name, force_name, &result);

            if (phase == 3U) {
                lap_count++;
                if (lap_count >= TASK11_TARGET_LAPS) {
                    stop_reason = 2U;
                    break;
                }
                phase = 0U;
            } else {
                phase++;
            }
            phase_start_count = total_distance_count;
            yaw_start = yaw_cdeg;
            exit_line_seen = 0U;
            straight_point_confirm = 0U;
            arc_point_confirm = 0U;
            filtered_error = 0;
            last_filtered_error = 0;
            last_turn = 0;
            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_speed_diff_sum = 0;
            report_sample_count = 0;
            continue;
        }

        if (line_valid != 0U) {
            raw_error = sample.error;
            filtered_error += (raw_error - filtered_error) / TASK11_LINE_ERROR_FILTER_DIVISOR;
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -derivative_limit,
                derivative_limit);
            last_filtered_error = filtered_error;
            p_component = filtered_error / kp_divisor;
            d_component = derivative / kd_divisor;
            turn = p_component + d_component;
            turn = clamp_i32(turn, -turn_limit, turn_limit);
            last_turn = turn;
        } else {
            base_b_pwm -= lost_base_drop;
            base_a_pwm -= lost_base_drop;
            if (last_turn != 0) {
                turn = clamp_i32(last_turn, -lost_turn_limit, lost_turn_limit);
            } else {
                turn = expected_turn_dir * lost_turn_limit;
            }
        }

        left_pwm = clamp_i32(base_b_pwm + turn,
            TASK11_LINE_MIN_PWM,
            pwm_max);
        right_pwm = clamp_i32(base_a_pwm - turn,
            TASK11_LINE_MIN_PWM,
            pwm_max);
        pwm_diff = left_pwm - right_pwm;
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (report_elapsed_ms >= (uint32_t)report_period_ms) {
            int32_t b_speed_avg = (report_sample_count != 0U) ?
                (report_b_speed_sum / (int32_t)report_sample_count) : motor_b_speed;
            int32_t a_speed_avg = (report_sample_count != 0U) ?
                (report_a_speed_sum / (int32_t)report_sample_count) : motor_a_speed;
            int32_t speed_diff_avg = (report_sample_count != 0U) ?
                (report_speed_diff_sum / (int32_t)report_sample_count) : speed_diff;

            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_speed_diff_sum = 0;
            report_sample_count = 0;
            lc_printf("TASK11_DATA lap=%u seg=%s ph=%u t=%lu total=%ld dist=%ld yaw=%ld yprog=%ld gz=%ld mask=0x%02X cnt=%u LCR=%u/%u/%u cross=%u side=%u lost=%u err=%ld filt=%ld der=%ld P=%ld D=%ld turn=%ld Bspd=%ld Aspd=%ld vdiff=%ld Bavg=%ld Aavg=%ld davg=%ld derr=%ld pwm=%ld/%ld pdiff=%ld\r\n",
                lap_count,
                segment_name,
                phase,
                elapsed_ms,
                total_distance_count,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                gzlp_mdps,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                left_count,
                center_count,
                right_count,
                straight_wide_cross,
                expected_side_seen,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                filtered_error,
                derivative,
                p_component,
                d_component,
                turn,
                motor_b_speed,
                motor_a_speed,
                speed_diff,
                b_speed_avg,
                a_speed_avg,
                speed_diff_avg,
                distance_error,
                left_pwm,
                right_pwm,
                pwm_diff);
        }
    }

    TB6612_Brake();
    if (g_st011_pulse_remaining_ms != 0U) {
        delay_ms_with_st011(g_st011_pulse_remaining_ms);
    }
    encoder_reset_distance_counts();
    if (stop_reason == 0U) {
        stop_reason = (lap_count >= TASK11_TARGET_LAPS) ? 1U :
            ((elapsed_ms >= TASK11_TOTAL_MAX_RUN_MS) ? 4U : 2U);
    }
    lc_printf("TASK11 complete: pure_ir reason=%s t=%lu lap=%u phase=%u\r\n",
        task11_reason_name(stop_reason),
        elapsed_ms,
        lap_count,
        phase);
}

#endif
