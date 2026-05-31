#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "app_config.h"
#include "app_control.h"
#include "app_debug_modes.h"
#include "bsp_encoder.h"

/*
 * 当前主程序说明：
 *
 * 1. 上电后先由 SysConfig 初始化时钟、GPIO、PWM、UART、I2C 等外设。
 * 2. 初始化编码器状态和 TB6612 电机驱动。
 * 3. 如果 ENABLE_IR_TRACKING_UART_TEST 为 1，进入八路红外循迹模块测试：
 *    通过 I2C 读取模块数据，再通过 UART0 串口打印，电机保持刹车。
 * 4. 如果 ENABLE_IR_TRACKING_UART_TEST 为 0，进入后驱两轮直行 PID 测试：
 *    利用 A/B 电机编码器计数修正左右轮 PWM，使小车尽量直行。
 *
 * 目前等待蓝牙串口模块到货，下一步可以把 UART0 输出接到蓝牙模块，
 * 或者新增控制命令解析，实现无线启动、停车和参数调试。
 */

static uint8_t g_jy62_zero_ready;
static uint32_t g_st011_pulse_remaining_ms;

static void st011_set_active(uint8_t active)
{
#if ST011_ACTIVE_LOW
    if (active != 0U) {
        DL_GPIO_clearPins(ST011_PORT, ST011_TRIG_PIN);
    } else {
        DL_GPIO_setPins(ST011_PORT, ST011_TRIG_PIN);
    }
#else
    if (active != 0U) {
        DL_GPIO_setPins(ST011_PORT, ST011_TRIG_PIN);
    } else {
        DL_GPIO_clearPins(ST011_PORT, ST011_TRIG_PIN);
    }
#endif
}

static void st011_service(uint32_t elapsed_ms)
{
    if (g_st011_pulse_remaining_ms == 0U) {
        return;
    }

    if (elapsed_ms >= g_st011_pulse_remaining_ms) {
        g_st011_pulse_remaining_ms = 0U;
        st011_set_active(0U);
    } else {
        g_st011_pulse_remaining_ms -= elapsed_ms;
    }
}

static void delay_ms_with_st011(uint32_t total_ms)
{
    while (total_ms > 0U) {
        uint32_t step_ms = total_ms;

        if ((g_st011_pulse_remaining_ms != 0U) &&
            (step_ms > g_st011_pulse_remaining_ms)) {
            step_ms = g_st011_pulse_remaining_ms;
        }

        delay_ms(step_ms);
        st011_service(step_ms);
        total_ms -= step_ms;
    }
}

static void st011_start_pulse(uint32_t pulse_ms)
{
    if (pulse_ms == 0U) {
        return;
    }

    g_st011_pulse_remaining_ms = pulse_ms;
    st011_set_active(1U);
}

static void st011_pulse(uint32_t pulse_ms)
{
    st011_start_pulse(pulse_ms);
    delay_ms_with_st011(pulse_ms);
}

static int32_t normalize_cdeg(int32_t angle_cdeg)
{
    while (angle_cdeg > 18000L) {
        angle_cdeg -= 36000L;
    }

    while (angle_cdeg < -18000L) {
        angle_cdeg += 36000L;
    }

    return angle_cdeg;
}

typedef enum {
    TASK_ID_NONE = 0,
    TASK_ID_1 = 1,
    TASK_ID_2 = 2,
    TASK_ID_5 = 5,
    TASK_ID_STOP = 255
} task_id_t;

static task_id_t task_uart_read_command(void)
{
    static uint8_t seen_zero = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);

        if (ch == 0x00U) {
            seen_zero = 0U;
            return TASK_ID_STOP;
        }

        if ((ch == '1') || (ch == 0x01U)) {
            seen_zero = 0U;
            return TASK_ID_1;
        }

        if ((ch == '2') || (ch == 0x02U)) {
            seen_zero = 0U;
            return TASK_ID_2;
        }

        if ((ch == '5') || (ch == 0x05U)) {
            seen_zero = 0U;
            return TASK_ID_5;
        }

        if (seen_zero != 0U) {
            seen_zero = 0U;
            if (ch == '1') {
                return TASK_ID_1;
            }
            if (ch == '2') {
                return TASK_ID_2;
            }
            if (ch == '5') {
                return TASK_ID_5;
            }
            if (ch == '0') {
                return TASK_ID_STOP;
            }
        }

        if (ch == '0') {
            seen_zero = 1U;
        }
    }

    return TASK_ID_NONE;
}

static uint8_t task_uart_stop_requested(void)
{
    return (task_uart_read_command() == TASK_ID_STOP) ? 1U : 0U;
}

static uint8_t task_button_pin_is_pressed(uint32_t pin)
{
    return ((DL_GPIO_readPins(KEYS_PORT, pin) & pin) == 0U) ? 1U : 0U;
}

static task_id_t task_button_read(void)
{
    if (task_button_pin_is_pressed(KEYS_KEY1_PIN) != 0U) {
        return TASK_ID_1;
    }

    if (task_button_pin_is_pressed(KEYS_KEY2_PIN) != 0U) {
        return TASK_ID_2;
    }

    return TASK_ID_NONE;
}

static void jy62_print_navigation_line(const char *mode, uint32_t elapsed_ms)
{
#if ENABLE_JY62_NAV
    jy62_navigation_t nav;
    uint32_t frame_delta = JY62_GetNavigation(&nav);

    if ((g_jy62_zero_ready == 0U) && (nav.valid != 0U)) {
        JY62_SetYawZeroToCurrent();
        g_jy62_zero_ready = 1U;
        frame_delta = JY62_GetNavigation(&nav);
    }

    lc_printf("JY62 mode=%s t=%lu df=%lu ok=%u flags=0x%02X yaw_cdeg=%ld rel_cdeg=%ld gz_mdps=%ld gzlp_mdps=%ld rx=%lu head=%lu frames=%lu err=%lu/%lu/%lu\r\n",
        mode,
        elapsed_ms,
        frame_delta,
        nav.valid,
        nav.update_flags,
        nav.yaw_cdeg,
        nav.yaw_relative_cdeg,
        nav.gyro_z_mdps,
        nav.gyro_z_filtered_mdps,
        nav.rx_byte_count,
        nav.header_count,
        nav.frame_count,
        nav.checksum_error,
        nav.uart_error_count,
        nav.overrun_count);
#else
    (void)mode;
    (void)elapsed_ms;
#endif
}

static uint8_t jy62_zero_to_current(const char *mode, uint32_t elapsed_ms)
{
#if ENABLE_JY62_NAV
    jy62_navigation_t nav;

    (void)JY62_GetNavigation(&nav);
    if (nav.valid != 0U) {
        JY62_SetYawZeroToCurrent();
        g_jy62_zero_ready = 1U;
        jy62_print_navigation_line(mode, elapsed_ms);
        return 1U;
    }

    lc_printf("JY62 mode=%s t=%lu zero=0 ok=%u rx=%lu head=%lu frames=%lu err=%lu/%lu/%lu\r\n",
        mode,
        elapsed_ms,
        nav.valid,
        nav.rx_byte_count,
        nav.header_count,
        nav.frame_count,
        nav.checksum_error,
        nav.uart_error_count,
        nav.overrun_count);
    return 0U;
#else
    (void)mode;
    (void)elapsed_ms;
    return 0U;
#endif
}

static task_id_t wait_task_uart_command(void)
{
    task_id_t task_id;

    while (1) {
        task_id = task_uart_read_command();
        if (task_id != TASK_ID_NONE) {
            if (task_id == TASK_ID_STOP) {
                TB6612_Brake();
                lc_printf("TASK UART command accepted: id=0 stop\r\n");
            } else {
                lc_printf("TASK UART command accepted: id=%u\r\n", task_id);
            }
            return task_id;
        }

        task_id = task_button_read();
        if (task_id != TASK_ID_NONE) {
            delay_ms_with_st011(TASK_BUTTON_DEBOUNCE_MS);
            if (task_button_read() == task_id) {
                while (task_button_read() == task_id) {
                    (void)task_uart_read_command();
                    delay_ms_with_st011(TASK_BUTTON_IDLE_MS);
                }
                delay_ms_with_st011(TASK_BUTTON_DEBOUNCE_MS);
                return task_id;
            }
        }

        TB6612_Brake();
        delay_ms_with_st011(TASK_BUTTON_IDLE_MS);
    }
}

static uint8_t run_straight_to_line_segment(const char *tag,
    uint8_t zero_heading,
    int32_t heading_target_cdeg,
    uint8_t heading_only,
    uint32_t start_alarm_ms,
    uint32_t stop_alarm_ms)
{
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
    uint8_t stop_nav_ok;

    if (start_alarm_ms != 0U) {
        TB6612_Brake();
        st011_pulse(start_alarm_ms);
        delay_ms_with_st011(TASK1_START_SETTLE_MS);
    }
    if (zero_heading != 0U) {
        (void)jy62_zero_to_current(tag,
            (start_alarm_ms != 0U) ? TASK1_START_SETTLE_MS : 0U);
    }
    if (start_alarm_ms != 0U) {
        delay_ms_with_st011(TASK1_AFTER_ZERO_DELAY_MS);
    }

    IRTracking_Init();
    straight_pid_reset(&pid);
    heading_filter_reset(&heading_filter);
    encoder_reset_distance_counts();
    encoder_enable_interrupts();

    TB6612_SetDifferential(TASK1_RAMP_B_START_PWM, TASK1_RAMP_A_START_PWM);

    while (elapsed_ms < TASK1_MAX_RUN_MS) {
        int32_t motor_b_speed;
        int32_t motor_a_speed;
        int32_t motor_b_total;
        int32_t motor_a_total;
        int32_t distance_count;
        int32_t error;
        int32_t p_term;
        int32_t i_term;
        int32_t d_term;
        int32_t speed_correction;
        int32_t distance_error;
        int32_t distance_correction;
        int32_t balance_correction;
        int32_t heading_raw;
        int32_t heading_raw_error;
        int32_t heading_error;
        int32_t heading_filtered;
        int32_t heading_gain;
        int32_t heading_gyro_z;
        int32_t heading_correction;
        int32_t heading_corr_divisor;
        int32_t heading_corr_max;
        int32_t correction;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;
        uint8_t nav_ok;
        uint8_t ir_armed;
        uint8_t heading_priority;
        uint8_t heading_wobble;

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

        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        ir_armed = (distance_count >= TASK1_B_LINE_ARM_COUNT) ? 1U : 0U;
        nav_ok = JY62_PeekNavigation(&nav);
        heading_raw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
        heading_gyro_z = (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0;
        heading_raw_error = (nav_ok != 0U) ?
            normalize_cdeg(heading_raw - heading_target_cdeg) : 0;
        heading_filtered = 0;
        heading_gain = 0;
        heading_wobble = 0U;
        heading_error = (nav_ok != 0U) ?
            heading_filter_update(&heading_filter,
                heading_raw_error,
                heading_gyro_z,
                &heading_filtered,
                &heading_gain,
                &heading_wobble) : 0;
        if (heading_target_cdeg != 0) {
            heading_error = heading_raw_error;
        }
        heading_corr_divisor = (heading_target_cdeg != 0) ?
            TASK2_CD_HEADING_CORR_DIVISOR : TASK1_HEADING_CORR_DIVISOR;
        heading_corr_max = (heading_target_cdeg != 0) ?
            TASK2_CD_HEADING_CORR_MAX : TASK1_HEADING_CORR_MAX;
        heading_correction = (heading_error * TASK1_HEADING_CORR_SIGN) / heading_corr_divisor;
        if (heading_target_cdeg != 0) {
            heading_correction -= heading_gyro_z / TASK2_CD_HEADING_GYRO_DAMP_DIVISOR;
        }
        heading_correction = clamp_i32(heading_correction, -heading_corr_max, heading_corr_max);

        ir_ok = IRTracking_ReadSample(&sample);
        if ((ir_armed != 0U) &&
            (ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (sample.active_count >= TASK1_STOP_MIN_IR_COUNT)) {
            stop_reason = 1U;
            break;
        }

        if (distance_count >= TASK1_FORCE_STOP_COUNT) {
            stop_reason = 2U;
            break;
        }

        speed_correction = straight_pid_update(&pid,
            motor_b_speed, motor_a_speed,
            STRAIGHT_TARGET_SPEED_DIFF,
            &error, &p_term, &i_term, &d_term);
        distance_error = motor_b_total - motor_a_total;
        distance_correction = clamp_i32(distance_error / TASK1_DISTANCE_CORR_DIVISOR,
            -TASK1_DISTANCE_CORR_MAX, TASK1_DISTANCE_CORR_MAX);
        balance_correction = speed_correction + distance_correction;
        heading_priority = 0U;

        if ((abs_i32(heading_error) >= TASK1_HEADING_PRIORITY_CDEG) &&
            (abs_i32(error) <= TASK1_HEADING_PRIORITY_MAX_VERR) &&
            (abs_i32(distance_error) <= TASK1_HEADING_PRIORITY_MAX_DERR) &&
            (heading_correction != 0) &&
            (((balance_correction > 0) && (heading_correction < 0)) ||
             ((balance_correction < 0) && (heading_correction > 0)))) {
            balance_correction = 0;
            pid.integral = 0;
            heading_priority = 1U;
        }
        if (heading_only != 0U) {
            balance_correction = 0;
            pid.integral = 0;
            heading_priority = 2U;
        }

        correction = clamp_i32(balance_correction + heading_correction,
            -STRAIGHT_CORR_MAX, STRAIGHT_CORR_MAX);

        base_b_pwm = ramp_i32(TASK1_RAMP_B_START_PWM, STRAIGHT_B_BASE_PWM,
            elapsed_ms, TASK1_START_RAMP_MS);
        base_a_pwm = ramp_i32(TASK1_RAMP_A_START_PWM, STRAIGHT_A_BASE_PWM,
            elapsed_ms, TASK1_START_RAMP_MS);

        motor_b_pwm = clamp_i32(base_b_pwm - correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);
        motor_a_pwm = clamp_i32(base_a_pwm + correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);

        TB6612_SetDifferential((int16_t)motor_b_pwm, (int16_t)motor_a_pwm);

        if (report_elapsed_ms >= TASK1_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("%s t=%lu dist=%ld arm=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u h_raw=%ld h_tgt=%ld h_flt=%ld h_use=%ld h_gain=%ld h_wob=%u gzlp=%ld h_corr=%ld hp=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                ir_armed,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                ir_ok,
                nav_ok,
                heading_raw,
                heading_target_cdeg,
                heading_filtered,
                heading_error,
                heading_gain,
                heading_wobble,
                heading_gyro_z,
                heading_correction,
                heading_priority,
                motor_b_total,
                motor_a_total,
                distance_error,
                distance_correction,
                motor_b_speed,
                motor_a_speed,
                (int32_t)STRAIGHT_TARGET_SPEED_DIFF,
                error,
                p_term,
                i_term,
                d_term,
                speed_correction,
                balance_correction,
                correction,
                base_b_pwm,
                base_a_pwm,
                motor_b_pwm,
                motor_a_pwm);
        }

        if (jy62_report_elapsed_ms >= JY62_TASK_REPORT_PERIOD_MS) {
            jy62_report_elapsed_ms = 0;
            jy62_print_navigation_line(tag, elapsed_ms);
        }
    }

    if ((stop_alarm_ms != 0U) || (stop_reason != 1U)) {
        TB6612_Brake();
    }
    encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
    encoder_get_total_counts(&motor_b_delta, &motor_a_delta);
    stop_nav_ok = JY62_PeekNavigation(&nav);
    lc_printf("%s stop: reason=%s t=%lu dist=%ld arm=%d force=%d raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u rel_cdeg=%ld B_total=%ld A_total=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "line" : ((stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "uart_stop" : "timeout")),
        elapsed_ms,
        (abs_i32(motor_b_delta) + abs_i32(motor_a_delta)) / 2,
        TASK1_B_LINE_ARM_COUNT,
        TASK1_FORCE_STOP_COUNT,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        ir_ok,
        stop_nav_ok,
        nav.yaw_relative_cdeg,
        motor_b_delta,
        motor_a_delta);
    if (stop_alarm_ms != 0U) {
        st011_pulse(stop_alarm_ms);
    }

    return stop_reason;
}

static uint8_t task2_arc_yaw_in_exit_window(int32_t yaw_cdeg)
{
    return (abs_i32(TASK2_ARC_ALIGN_TARGET_CDEG - abs_i32(yaw_cdeg)) <=
        TASK2_ARC_ALIGN_TOL_CDEG) ? 1U : 0U;
}

static int32_t task2_arc_yaw_align_error(int32_t yaw_cdeg)
{
    return TASK2_ARC_ALIGN_TARGET_CDEG - abs_i32(yaw_cdeg);
}

static int32_t task2_arc_align_delta_cdeg(int32_t yaw_cdeg, int32_t last_turn)
{
    int32_t target_cdeg;

    if (yaw_cdeg < 0) {
        target_cdeg = -TASK2_ARC_ALIGN_TARGET_CDEG;
    } else if (yaw_cdeg > 0) {
        target_cdeg = TASK2_ARC_ALIGN_TARGET_CDEG;
    } else {
        target_cdeg = (last_turn >= 0) ?
            -TASK2_ARC_ALIGN_TARGET_CDEG : TASK2_ARC_ALIGN_TARGET_CDEG;
    }

    return target_cdeg - yaw_cdeg;
}

typedef enum {
    ARC_TURN_LEFT = -1,
    ARC_TURN_RIGHT = 1
} arc_turn_dir_t;

static int32_t task2_arc_model_target_diff(arc_turn_dir_t turn_dir)
{
    int32_t target_diff = ((int32_t)TASK2_ARC_CENTER_SPEED_COUNT *
        (int32_t)TASK2_ARC_WHEEL_BASE_MM) / (int32_t)TASK2_ARC_RADIUS_MM;

    target_diff = clamp_i32(target_diff, 0, TASK2_ARC_TARGET_DIFF_MAX);
    return ((int32_t)turn_dir > 0) ? target_diff : -target_diff;
}

static uint8_t run_arc_line_follow_segment(const char *tag,
    uint32_t stop_alarm_ms,
    arc_turn_dir_t turn_dir)
{
    straight_pid_t diff_pid;
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t filtered_error = 0;
    int32_t last_filtered_error = 0;
    int32_t arc_start_yaw = 0;
    int32_t model_target_diff = task2_arc_model_target_diff(turn_dir);
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t head_exit_seen = 0U;

    IRTracking_Init();
    straight_pid_reset(&diff_pid);
    straight_pid_set_limits(&diff_pid, TASK2_ARC_PID_I_LIMIT, TASK2_ARC_PID_CORR_MAX);
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = JY62_PeekNavigation(&nav);
    arc_start_yaw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;

    lc_printf("%s start: arc model follow dir=%d R=%dcm W=%dmm sensor_axis=%dmm cpcm=%d arc_len=%ld exit_arm=%ld finish=%ld model_diff=%ld base=%d/%d\r\n",
        tag,
        (int32_t)turn_dir,
        TASK2_ARC_RADIUS_CM,
        TASK2_ARC_WHEEL_BASE_MM,
        TASK2_ARC_SENSOR_TO_AXIS_MM,
        COUNTS_PER_CM,
        (int32_t)TASK2_ARC_LENGTH_COUNT,
        (int32_t)TASK2_ARC_EXIT_ARM_COUNT,
        (int32_t)TASK2_ARC_FINISH_COUNT,
        model_target_diff,
        TASK2_ARC_B_BASE_PWM,
        TASK2_ARC_A_BASE_PWM);

    while (elapsed_ms < TASK2_ARC_MAX_RUN_MS) {
        int32_t motor_b_total;
        int32_t motor_a_total;
        int32_t motor_b_speed;
        int32_t motor_a_speed;
        int32_t distance_count;
        int32_t raw_error;
        int32_t error;
        int32_t derivative;
        int32_t ir_correction = 0;
        int32_t yaw_correction = 0;
        int32_t target_diff;
        int32_t pid_error;
        int32_t p_term;
        int32_t i_term;
        int32_t d_term;
        int32_t feedback_correction;
        int32_t feedforward_correction;
        int32_t pwm_correction;
        int32_t left_pwm;
        int32_t right_pwm;
        int32_t yaw_rel;
        int32_t yaw_delta;
        int32_t yaw_progress;
        int32_t theta_ref;
        int32_t yaw_lag;
        uint8_t line_lost;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 5U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;

        nav_ok = JY62_PeekNavigation(&nav);
        yaw_rel = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
        yaw_delta = (nav_ok != 0U) ? normalize_cdeg(yaw_rel - arc_start_yaw) : 0;
        yaw_progress = abs_i32(yaw_delta);
        theta_ref = (int32_t)(((int64_t)distance_count * 18000LL) /
            (int64_t)TASK2_ARC_LENGTH_COUNT);
        theta_ref = clamp_i32(theta_ref, 0, 18000);
        yaw_lag = theta_ref - yaw_progress;
        yaw_correction = clamp_i32(yaw_lag / TASK2_ARC_YAW_CORR_DIVISOR,
            -TASK2_ARC_YAW_CORR_MAX, TASK2_ARC_YAW_CORR_MAX);
        yaw_correction *= (int32_t)turn_dir;

        ir_ok = IRTracking_ReadSample(&sample);
        line_lost = ((ir_ok == 0U) || (sample.line_lost != 0U)) ? 1U : 0U;

        if ((line_lost != 0U) && (distance_count >= TASK2_ARC_EXIT_ARM_COUNT)) {
            head_exit_seen = 1U;
        }

        if (line_lost == 0U) {
            raw_error = sample.error;
            error = (abs_i32(raw_error) < TASK2_ARC_ERROR_DEADBAND) ? 0 : raw_error;
            filtered_error += (error - filtered_error) / TASK2_ARC_ERROR_FILTER_DIVISOR;
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -TASK2_ARC_DERIVATIVE_LIMIT, TASK2_ARC_DERIVATIVE_LIMIT);
            last_filtered_error = filtered_error;

            ir_correction = (filtered_error / TASK2_ARC_IR_KP_DIVISOR) +
                (derivative / TASK2_ARC_IR_KD_DIVISOR);
            ir_correction = clamp_i32(ir_correction,
                -TASK2_ARC_IR_CORR_MAX, TASK2_ARC_IR_CORR_MAX);
        } else {
            raw_error = 0;
            error = 0;
            derivative = 0;
        }

        target_diff = model_target_diff + ir_correction + yaw_correction;
        target_diff = clamp_i32(target_diff,
            -TASK2_ARC_TARGET_DIFF_MAX, TASK2_ARC_TARGET_DIFF_MAX);
        feedback_correction = straight_pid_update(&diff_pid,
            motor_b_speed,
            motor_a_speed,
            target_diff,
            &pid_error,
            &p_term,
            &i_term,
            &d_term);
        feedforward_correction = clamp_i32(-(target_diff * TASK2_ARC_DIFF_FF_GAIN),
            -TASK2_ARC_PID_CORR_MAX, TASK2_ARC_PID_CORR_MAX);
        pwm_correction = clamp_i32(feedforward_correction + feedback_correction,
            -TASK2_ARC_PID_CORR_MAX, TASK2_ARC_PID_CORR_MAX);

        left_pwm = clamp_i32(TASK2_ARC_B_BASE_PWM - pwm_correction,
            TASK2_ARC_MIN_PWM, TASK2_ARC_MAX_PWM);
        right_pwm = clamp_i32(TASK2_ARC_A_BASE_PWM + pwm_correction,
            TASK2_ARC_MIN_PWM, TASK2_ARC_MAX_PWM);

        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if ((distance_count >= TASK2_ARC_FINISH_COUNT) &&
            ((nav_ok == 0U) || (yaw_progress >= TASK2_ARC_YAW_DONE_CDEG))) {
            stop_reason = 3U;
            break;
        }

        if ((head_exit_seen != 0U) &&
            (yaw_progress >= TASK2_ARC_ALIGN_TARGET_CDEG)) {
            stop_reason = 3U;
            break;
        }

        if (distance_count >= TASK2_ARC_FORCE_STOP_COUNT) {
            stop_reason = 2U;
            break;
        }

        if (report_elapsed_ms >= TASK2_ARC_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("%s t=%lu dist=%ld arc_len=%ld exit=%u yaw_rel=%ld yaw_prog=%ld theta=%ld yaw_lag=%ld nav=%u ir=%u lost=%u raw=0x%02X mask=0x%02X cnt=%u ir_err=%ld ir_corr=%ld yaw_corr=%ld model=%ld tgt=%ld B_spd=%ld A_spd=%ld pid_err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld pwm_corr=%ld L=%ld R=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                (int32_t)TASK2_ARC_LENGTH_COUNT,
                head_exit_seen,
                yaw_rel,
                yaw_progress,
                theta_ref,
                yaw_lag,
                nav_ok,
                ir_ok,
                line_lost,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                error,
                ir_correction,
                yaw_correction,
                model_target_diff,
                target_diff,
                motor_b_speed,
                motor_a_speed,
                pid_error,
                p_term,
                i_term,
                d_term,
                feedforward_correction,
                feedback_correction,
                pwm_correction,
                left_pwm,
                right_pwm);
        }
    }

    if ((stop_alarm_ms != 0U) || (stop_reason != 3U)) {
        TB6612_Brake();
    }
    encoder_get_total_counts(&motor_b_delta, &motor_a_delta);
    nav_ok = JY62_PeekNavigation(&nav);
    lc_printf("%s stop: reason_id=%u reason=%s t=%lu dist=%ld finish=%ld yaw_rel=%ld yaw_prog=%ld nav=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        stop_reason,
        (stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "arc_done" : ((stop_reason == 5U) ? "uart_stop" : "timeout")),
        elapsed_ms,
        (abs_i32(motor_b_delta) + abs_i32(motor_a_delta)) / 2,
        (int32_t)TASK2_ARC_FINISH_COUNT,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? abs_i32(normalize_cdeg(nav.yaw_relative_cdeg - arc_start_yaw)) : 0,
        nav_ok,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U);

    if (stop_alarm_ms != 0U) {
        st011_pulse(stop_alarm_ms);
    }

    return stop_reason;
}

static uint8_t task2_arc_stop_is_success(uint8_t stop_reason)
{
    return ((stop_reason == 1U) || (stop_reason == 3U)) ? 1U : 0U;
}

static void run_task1_ab(void)
{
    (void)run_straight_to_line_segment("TASK1_AB",
        0U,
        0,
        1U,
        TASK1_START_ALARM_MS,
        TASK1_FINISH_ALARM_MS);
}

static void run_task2_abcd(void)
{
    uint8_t reason;

    lc_printf("TASK2 start: A->B straight, B->C arc, C->D straight, D->A arc\r\n");

    reason = run_straight_to_line_segment("TASK2_AB",
        0U,
        0,
        1U,
        TASK1_START_ALARM_MS,
        0U);
    if (reason != 1U) {
        lc_printf("TASK2 abort after AB: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_arc_line_follow_segment("TASK2_BC", 0U, ARC_TURN_RIGHT);
    if (task2_arc_stop_is_success(reason) == 0U) {
        lc_printf("TASK2 abort after BC: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);
    encoder_reset_distance_counts();
    lc_printf("TASK2 BC complete: distance reset, heading correction continues on CD straight\r\n");

    reason = run_straight_to_line_segment("TASK2_CD",
        0U,
        TASK2_CD_HEADING_TARGET_CDEG,
        1U,
        0U,
        0U);
    if (reason != 1U) {
        lc_printf("TASK2 abort after CD: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_arc_line_follow_segment("TASK2_DA", 0U, ARC_TURN_RIGHT);
    if (task2_arc_stop_is_success(reason) == 0U) {
        lc_printf("TASK2 abort after DA: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);
    encoder_reset_distance_counts();

    TB6612_Brake();
    lc_printf("TASK2 complete: stopped at A, distance reset\r\n");
}

static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: UART0 01 or PB00=task1, UART0 02 or PB21=task2, UART0 05=PWM test, UART0 00=stop\r\n");

    while (1) {
        task_id = wait_task_uart_command();

        if (task_id == TASK_ID_STOP) {
            TB6612_Brake();
            continue;
        }

        if ((task_id == TASK_ID_1) || (task_id == TASK_ID_2)) {
            TB6612_Brake();
            (void)jy62_zero_to_current("task_start_zero", 0U);
        }

        if (task_id == TASK_ID_1) {
            run_task1_ab();
        } else if (task_id == TASK_ID_2) {
            run_task2_abcd();
        } else if (task_id == TASK_ID_5) {
            TB6612_Brake();
            run_motor_pid_stream();
        } else {
            TB6612_Brake();
            st011_pulse(TASK1_START_ALARM_MS);
            lc_printf("TASK id=%u not implemented yet\r\n", task_id);
        }
    }
}

int main(void)
{
    /* SysConfig 初始化时钟、GPIO、PWM、UART、I2C 和中断路由。 */
    SYSCFG_DL_init();
    st011_set_active(0U);
    lc_printf("\r\nBOOT: UART OK, IR line follow firmware\r\n");

#if ENABLE_JY62_NAV
    JY62_Init();
    g_jy62_zero_ready = 0U;
    lc_printf("BOOT: JY62 UART1 ready, PA08 TX -> RXD, PA09 RX <- TXD, baud=%lu\r\n",
        (uint32_t)JY62_UART_BAUD_RATE);
    delay_ms(JY62_BOOT_ZERO_DELAY_MS);
    (void)jy62_zero_to_current("boot_zero", JY62_BOOT_ZERO_DELAY_MS);
#endif

    delay_ms(200);

    encoder_init_runtime();
    lc_printf("BOOT: encoder state loaded, B=PA14/PA15 A=PA16/PA17\r\n");
    delay_ms(200);

    TB6612_Init();
    lc_printf("BOOT: TB6612 ready, A motor and B motor enabled\r\n");
    st011_set_active(0U);
    delay_ms(1000);

#if ENABLE_CONTEST_TASKS
    run_task_dispatcher();
#elif ENABLE_LINE_FOLLOW_TEST
    run_line_follow_test();
#elif ENABLE_IR_TRACKING_UART_TEST
    /*
     * 当前默认进入红外模块串口打印测试。
     * 想重新跑直行 PID 时，把 ENABLE_IR_TRACKING_UART_TEST 改为 0 后重新编译烧录。
     */
    run_ir_tracking_uart_test();
#else

    /* 默认关闭自检；只有小车架空时才建议打开。 */
    if ((ENABLE_ENCODER_SELF_TEST != 0) && (encoder_motor_self_test() == 0U)) {
        lc_printf("Self-test failed. Fix wiring/direction before PID run.\r\n");
        while (1) {
            TB6612_Brake();
            delay_ms(1000);
        }
    }

    run_motor_pid_stream();
#endif

    while (1) {
    }
}

/* GPIOA 中断服务函数，负责处理所有编码器引脚。 */
void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN |
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN);

    if ((status & (ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN)) != 0U) {
        encoder_update_motor_b();
    }

    if ((status & (ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN)) != 0U) {
        encoder_update_motor_a();
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, status);
}

#if ENABLE_JY62_NAV
void UART_1_INST_IRQHandler(void)
{
    JY62_UART1_IRQHandler();
}
#endif
