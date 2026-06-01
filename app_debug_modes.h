#ifndef APP_DEBUG_MODES_H
#define APP_DEBUG_MODES_H

#include <stdint.h>

#include "app_config.h"
#include "app_control.h"
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
    int32_t feedforward_correction =
        -((int32_t)PID_TEST_TARGET_SPEED_DIFF * (int32_t)PID_TEST_DIFF_FF_GAIN);

    straight_pid_reset(&pid);
    straight_pid_set_limits(&pid, PID_TEST_I_LIMIT, PID_TEST_CORR_MAX);
    feedforward_correction = clamp_i32(feedforward_correction,
        -PID_TEST_CORR_MAX, PID_TEST_CORR_MAX);

    /* 每次进入 05 模式都从 0 开始累计，方便标定 COUNTS_PER_CM。 */
    encoder_reset_distance_counts();
    lc_printf("PID motor stream start: B_base=%d A_base=%d target_diff=%d ff_gain=%d ff_corr=%ld d_div=%d d_max=%d i_limit=%d corr_max=%d period=%dms report=%dms\r\n",
        STRAIGHT_B_BASE_PWM,
        STRAIGHT_A_BASE_PWM,
        PID_TEST_TARGET_SPEED_DIFF,
        PID_TEST_DIFF_FF_GAIN,
        feedforward_correction,
        PID_TEST_DISTANCE_CORR_DIVISOR,
        PID_TEST_DISTANCE_CORR_MAX,
        PID_TEST_I_LIMIT,
        PID_TEST_CORR_MAX,
        CONTROL_PERIOD_MS,
        PID_REPORT_PERIOD_MS);
    encoder_enable_interrupts();
    lc_printf("PID encoder IRQ enabled, B=PA14/PA15 A=PA16/PA17\r\n");
    TB6612_SetDifferential(STRAIGHT_B_BASE_PWM, STRAIGHT_A_BASE_PWM);

    while (1) {
        int32_t motor_b_speed;
        int32_t motor_a_speed;
        int32_t speed_diff;
        int32_t error;
        int32_t p_term;
        int32_t i_term;
        int32_t d_term;
        int32_t feedback_correction;
        int32_t distance_error;
        int32_t distance_correction;
        int32_t correction;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;

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
        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        speed_diff = motor_b_speed - motor_a_speed;
        report_b_speed_sum += motor_b_speed;
        report_a_speed_sum += motor_a_speed;
        report_sample_count++;

        feedback_correction = straight_pid_update(&pid,
            motor_b_speed, motor_a_speed,
            PID_TEST_TARGET_SPEED_DIFF,
            &error, &p_term, &i_term, &d_term);
        distance_error = motor_b_total - motor_a_total;
        distance_correction = clamp_i32(distance_error / PID_TEST_DISTANCE_CORR_DIVISOR,
            -PID_TEST_DISTANCE_CORR_MAX, PID_TEST_DISTANCE_CORR_MAX);
        correction = clamp_i32(feedforward_correction + feedback_correction + distance_correction,
            -PID_TEST_CORR_MAX, PID_TEST_CORR_MAX);

        /*
         * correction 为正时，说明 B 轮偏快，所以 B_pwm 减小、A_pwm 增大；
         * correction 为负时，说明 A 轮偏快，所以 B_pwm 增大、A_pwm 减小。
         */
        motor_b_pwm = clamp_i32(STRAIGHT_B_BASE_PWM - correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);
        motor_a_pwm = clamp_i32(STRAIGHT_A_BASE_PWM + correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);

        /* TB6612_SetDifferential(left, right)：左轮对应 B 电机，右轮对应 A 电机。 */
        TB6612_SetDifferential((int16_t)motor_b_pwm, (int16_t)motor_a_pwm);

        if (report_elapsed_ms >= PID_REPORT_PERIOD_MS) {
            int32_t b_speed_avg = (report_sample_count != 0U) ?
                (report_b_speed_sum / (int32_t)report_sample_count) : motor_b_speed;
            int32_t a_speed_avg = (report_sample_count != 0U) ?
                (report_a_speed_sum / (int32_t)report_sample_count) : motor_a_speed;
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
                distance_count,
                distance_error,
                distance_correction,
                motor_b_speed, motor_a_speed,
                speed_diff,
                b_speed_avg,
                a_speed_avg,
                diff_avg,
                PID_TEST_TARGET_SPEED_DIFF,
                error, p_term, i_term, d_term,
                feedforward_correction,
                feedback_correction,
                correction,
                motor_b_pwm, motor_a_pwm);
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

#endif
