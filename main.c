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
 * 2. 初始化 JY62、编码器状态和 TB6612 电机驱动。
 * 3. 正式模式下等待四个实体任务按键或 UART0 命令：
 *    A26/01 启动任务一，A24/02 启动任务二，B24/03 启动任务三，
 *    A22/04 启动任务四，05 进入轮速测试，00 强制停车。
 * 4. 调试模式仍可通过 app_config.h 中的开关进入红外打印、
 *    纯红外循迹或基础轮速 PID 测试。
 */

static uint8_t g_jy62_zero_ready;
static uint32_t g_st011_pulse_remaining_ms;
static uint8_t g_task6_c_turn_requested;

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
    TASK_ID_3 = 3,
    TASK_ID_4 = 4,
    TASK_ID_5 = 5,
    TASK_ID_6 = 6,
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

        if ((ch == '3') || (ch == 0x03U)) {
            seen_zero = 0U;
            return TASK_ID_3;
        }

        if ((ch == '4') || (ch == 0x04U)) {
            seen_zero = 0U;
            return TASK_ID_4;
        }

        if ((ch == '5') || (ch == 0x05U)) {
            seen_zero = 0U;
            return TASK_ID_5;
        }

        if ((ch == '6') || (ch == 0x06U)) {
            seen_zero = 0U;
            return TASK_ID_6;
        }

        if (seen_zero != 0U) {
            seen_zero = 0U;
            if (ch == '1') {
                return TASK_ID_1;
            }
            if (ch == '2') {
                return TASK_ID_2;
            }
            if (ch == '3') {
                return TASK_ID_3;
            }
            if (ch == '4') {
                return TASK_ID_4;
            }
            if (ch == '5') {
                return TASK_ID_5;
            }
            if (ch == '6') {
                return TASK_ID_6;
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

static uint8_t task_button_pin_is_pressed(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) == 0U) ? 1U : 0U;
}

static task_id_t task_button_read(void)
{
    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY1_PIN) != 0U) {
        return TASK_ID_1;
    }

    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY2_PIN) != 0U) {
        return TASK_ID_2;
    }

    if (task_button_pin_is_pressed(KEYS_B_PORT, KEYS_B_KEY3_PIN) != 0U) {
        return TASK_ID_3;
    }

    if (task_button_pin_is_pressed(KEYS_A_PORT, KEYS_A_KEY4_PIN) != 0U) {
        return TASK_ID_4;
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

static int32_t task2_straight_search_direction(uint8_t fast_correction,
    int32_t heading_target_cdeg)
{
    if (fast_correction != 0U) {
        return -1;
    }

    if (heading_target_cdeg != 0) {
        return 1;
    }

    return 1;
}

static uint8_t run_line_fast_turn(const char *tag,
    uint32_t ac_elapsed_ms,
    int32_t ac_distance_count,
    const ir_tracking_sample_t *line_sample,
    int32_t line_yaw_cdeg,
    int32_t target_cdeg,
    int16_t motor_b_pwm,
    int16_t motor_a_pwm,
    int32_t handoff_turn_dir,
    uint8_t target_is_absolute,
    uint8_t enable_line_stop,
    uint8_t brake_after_turn,
    uint8_t report_samples);

static uint8_t task4_ac_debug_enabled(const char *tag)
{
    return ((tag[0] == 'T') &&
        (tag[1] == 'A') &&
        (tag[2] == 'S') &&
        (tag[3] == 'K') &&
        (tag[4] == '4') &&
        (tag[5] == '_') &&
        (tag[6] == 'L') &&
        (tag[8] == '_') &&
        (tag[9] == 'A') &&
        (tag[10] == 'C') &&
        (tag[11] == '\0')) ? 1U : 0U;
}

static const char *task4_ac_turn_tag(const char *tag)
{
    static const char * const turn_tags[TASK4_LAP_COUNT] = {
        "TASK4_L1_C_TURN",
        "TASK4_L2_C_TURN",
        "TASK4_L3_C_TURN",
        "TASK4_L4_C_TURN"
    };
    uint8_t lap_index = (uint8_t)(tag[7] - '1');

    if (lap_index < TASK4_LAP_COUNT) {
        return turn_tags[lap_index];
    }

    return "TASK4_C_TURN";
}

static uint8_t task4_bd_debug_enabled(const char *tag)
{
    if ((tag[0] == 'T') &&
        (tag[1] == 'A') &&
        (tag[2] == 'S') &&
        (tag[3] == 'K') &&
        (tag[4] == '3') &&
        (tag[5] == '_') &&
        (tag[6] == 'B') &&
        (tag[7] == 'D') &&
        (tag[8] == '\0')) {
        return 1U;
    }

    return ((tag[0] == 'T') &&
        (tag[1] == 'A') &&
        (tag[2] == 'S') &&
        (tag[3] == 'K') &&
        (tag[4] == '4') &&
        (tag[5] == '_') &&
        (tag[6] == 'L') &&
        (tag[8] == '_') &&
        (tag[9] == 'B') &&
        (tag[10] == 'D') &&
        (tag[11] == '\0')) ? 1U : 0U;
}

static const char *task4_bd_turn_tag(const char *tag)
{
    static const char * const turn_tags[TASK4_LAP_COUNT] = {
        "TASK4_L1_D_TURN",
        "TASK4_L2_D_TURN",
        "TASK4_L3_D_TURN",
        "TASK4_L4_D_TURN"
    };
    uint8_t lap_index;

    if ((tag[0] == 'T') &&
        (tag[1] == 'A') &&
        (tag[2] == 'S') &&
        (tag[3] == 'K') &&
        (tag[4] == '3')) {
        return "TASK3_D_TURN";
    }

    lap_index = (uint8_t)(tag[7] - '1');

    if (lap_index < TASK4_LAP_COUNT) {
        return turn_tags[lap_index];
    }

    return "TASK4_D_TURN";
}

static const char *task4_a_turn_tag(uint8_t lap_index)
{
    static const char * const turn_tags[TASK4_LAP_COUNT] = {
        "TASK4_L1_A_TURN",
        "TASK4_L2_A_TURN",
        "TASK4_L3_A_TURN",
        "TASK4_L4_A_TURN"
    };

    if (lap_index < TASK4_LAP_COUNT) {
        return turn_tags[lap_index];
    }

    return "TASK4_A_TURN";
}

static void task4_print_ac_line_debug(const char *tag,
    uint32_t elapsed_ms,
    int32_t distance_count,
    const ir_tracking_sample_t *line_sample,
    uint8_t nav_ok,
    const jy62_navigation_t *nav)
{
    lc_printf("%s line_debug: ac_t=%lu ac_dist=%ld line_yaw=%ld line_gzlp=%ld line_raw=0x%02X line_mask=0x%02X line_cnt=%u line_lost=%u line_err=%ld nav=%u\r\n",
        tag,
        elapsed_ms,
        distance_count,
        (nav_ok != 0U) ? nav->yaw_relative_cdeg : 0,
        (nav_ok != 0U) ? nav->gyro_z_filtered_mdps : 0,
        (line_sample != 0) ? line_sample->raw : 0xFFU,
        (line_sample != 0) ? line_sample->line_mask : 0U,
        (line_sample != 0) ? line_sample->active_count : 0U,
        (line_sample != 0) ? line_sample->line_lost : 1U,
        (line_sample != 0) ? line_sample->error : 0,
        nav_ok);
}

static uint8_t run_straight_to_line_segment(const char *tag,
    uint8_t zero_heading,
    int32_t heading_target_cdeg,
    uint8_t heading_only,
    uint8_t fast_correction,
    uint8_t line_search_protect,
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
    int32_t start_heading_corr_divisor;
    int32_t start_heading_corr_max;
    int32_t start_distance_corr_divisor;
    int32_t start_distance_corr_max;
    int32_t start_line_arm_count;
    int32_t start_search_start_count;
    int32_t start_search_sweep_start_count;
    int32_t start_search_sweep_period_ms;
    uint8_t task6_turn_hook_ran = 0U;

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

    if (fast_correction != 0U) {
        start_heading_corr_divisor = TASK2_AB_HEADING_CORR_DIVISOR;
        start_heading_corr_max = TASK2_AB_HEADING_CORR_MAX;
        start_distance_corr_divisor = TASK2_AB_DISTANCE_CORR_DIVISOR;
        start_distance_corr_max = TASK2_AB_DISTANCE_CORR_MAX;
    } else if (heading_only != 0U) {
        start_heading_corr_divisor = TASK3_BD_HEADING_CORR_DIVISOR;
        start_heading_corr_max = TASK3_BD_HEADING_CORR_MAX;
        start_distance_corr_divisor = TASK1_DISTANCE_CORR_DIVISOR;
        start_distance_corr_max = TASK1_DISTANCE_CORR_MAX;
    } else if (heading_target_cdeg != 0) {
        start_heading_corr_divisor = TASK2_CD_HEADING_CORR_DIVISOR;
        start_heading_corr_max = TASK2_CD_HEADING_CORR_MAX;
        start_distance_corr_divisor = TASK1_DISTANCE_CORR_DIVISOR;
        start_distance_corr_max = TASK1_DISTANCE_CORR_MAX;
    } else {
        start_heading_corr_divisor = TASK1_HEADING_CORR_DIVISOR;
        start_heading_corr_max = TASK1_HEADING_CORR_MAX;
        start_distance_corr_divisor = TASK1_DISTANCE_CORR_DIVISOR;
        start_distance_corr_max = TASK1_DISTANCE_CORR_MAX;
    }
    if (line_search_protect >= 2U) {
        start_line_arm_count = TASK3_STRAIGHT_LINE_ARM_COUNT;
        start_search_start_count = TASK3_STRAIGHT_SEARCH_START_COUNT;
        start_search_sweep_start_count = TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        start_search_sweep_period_ms = TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS;
    } else if (line_search_protect != 0U) {
        start_line_arm_count = TASK1_B_LINE_ARM_COUNT;
        start_search_start_count = TASK2_STRAIGHT_SEARCH_START_COUNT;
        start_search_sweep_start_count = TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        start_search_sweep_period_ms = 0;
    } else {
        start_line_arm_count = TASK1_B_LINE_ARM_COUNT;
        start_search_start_count = 0;
        start_search_sweep_start_count = 0;
        start_search_sweep_period_ms = 0;
    }

    lc_printf("%s start: zero=%u heading_only=%u fast=%u line_protect=%u B_base=%d A_base=%d ramp=%d/%d h_div=%ld h_max=%ld d_div=%ld d_max=%ld arm=%ld search=%ld sweep=%ld period=%ld\r\n",
        tag,
        zero_heading,
        heading_only,
        fast_correction,
        line_search_protect,
        STRAIGHT_B_BASE_PWM,
        STRAIGHT_A_BASE_PWM,
        TASK1_RAMP_B_START_PWM,
        TASK1_RAMP_A_START_PWM,
        start_heading_corr_divisor,
        start_heading_corr_max,
        start_distance_corr_divisor,
        start_distance_corr_max,
        start_line_arm_count,
        start_search_start_count,
        start_search_sweep_start_count,
        start_search_sweep_period_ms);

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
        int32_t distance_corr_divisor;
        int32_t distance_corr_max;
        int32_t correction_limit;
        int32_t line_arm_count;
        int32_t search_start_count;
        int32_t search_sweep_start_count;
        int32_t search_corr_divisor;
        int32_t search_corr_max;
        int32_t search_soft_corr;
        int32_t search_sweep_corr;
        int32_t search_base_drop;
        int32_t stop_error_max;
        int32_t search_correction;
        int32_t search_direction;
        int32_t correction;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;
        uint8_t nav_ok;
        uint8_t ir_armed;
        uint8_t stop_mask;
        uint8_t line_centered;
        uint8_t line_stop_ready;
        uint8_t search_mode;
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
        line_arm_count = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_LINE_ARM_COUNT : TASK1_B_LINE_ARM_COUNT;
        ir_armed = (distance_count >= line_arm_count) ? 1U : 0U;
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
        if ((heading_target_cdeg != 0) || (heading_only != 0U)) {
            heading_error = heading_raw_error;
        } else if ((fast_correction != 0U) && (nav_ok != 0U)) {
            heading_error = (abs_i32(heading_raw_error) < TASK2_AB_HEADING_DEADBAND_CDEG) ?
                0 : heading_raw_error;
        }
        if (fast_correction != 0U) {
            heading_corr_divisor = TASK2_AB_HEADING_CORR_DIVISOR;
            heading_corr_max = TASK2_AB_HEADING_CORR_MAX;
        } else if (heading_only != 0U) {
            heading_corr_divisor = TASK3_BD_HEADING_CORR_DIVISOR;
            heading_corr_max = TASK3_BD_HEADING_CORR_MAX;
        } else if (heading_target_cdeg != 0) {
            heading_corr_divisor = TASK2_CD_HEADING_CORR_DIVISOR;
            heading_corr_max = TASK2_CD_HEADING_CORR_MAX;
        } else {
            heading_corr_divisor = TASK1_HEADING_CORR_DIVISOR;
            heading_corr_max = TASK1_HEADING_CORR_MAX;
        }
        heading_correction = (heading_error * TASK1_HEADING_CORR_SIGN) / heading_corr_divisor;
        if ((heading_target_cdeg != 0) || (heading_only != 0U)) {
            heading_correction -= heading_gyro_z / TASK2_CD_HEADING_GYRO_DAMP_DIVISOR;
        }
        heading_correction = clamp_i32(heading_correction, -heading_corr_max, heading_corr_max);

        ir_ok = IRTracking_ReadSample(&sample);
        stop_mask = (fast_correction != 0U) ?
            TASK2_AB_STOP_MASK : TASK2_CD_STOP_CENTER_MASK;
        stop_error_max = (fast_correction != 0U) ?
            TASK2_AB_STOP_ERROR_MAX : TASK2_CD_STOP_ERROR_MAX;
        line_centered = ((ir_ok != 0U) &&
            ((sample.line_mask & stop_mask) != 0U) &&
            (abs_i32(sample.error) <= stop_error_max)) ? 1U : 0U;
        if (line_search_protect >= 2U) {
            line_stop_ready = ((ir_ok != 0U) &&
                ((sample.line_mask & TASK3_STRAIGHT_STOP_MASK) != 0U) &&
                (sample.active_count >= TASK3_STRAIGHT_STOP_MIN_IR_COUNT)) ? 1U : 0U;
        } else {
            line_stop_ready = line_centered;
        }
        search_start_count = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_START_COUNT : TASK2_STRAIGHT_SEARCH_START_COUNT;
        search_sweep_start_count = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT : TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        search_corr_divisor = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_CORR_DIVISOR : TASK2_STRAIGHT_SEARCH_CORR_DIVISOR;
        search_corr_max = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_CORR_MAX : TASK2_STRAIGHT_SEARCH_CORR_MAX;
        search_soft_corr = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_SOFT_CORR : TASK2_STRAIGHT_SEARCH_SOFT_CORR;
        search_sweep_corr = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_SWEEP_CORR : TASK2_STRAIGHT_SEARCH_SWEEP_CORR;
        search_base_drop = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_SEARCH_BASE_DROP : TASK2_STRAIGHT_SEARCH_BASE_DROP;
        correction_limit = ((line_search_protect >= 2U) || (heading_only != 0U)) ?
            TASK3_STRAIGHT_CORR_MAX : STRAIGHT_CORR_MAX;
        search_mode = ((line_search_protect != 0U) &&
            ((distance_count >= search_start_count) ||
             ((ir_armed != 0U) && (ir_ok != 0U) &&
              (sample.line_lost == 0U) && (line_centered == 0U)))) ? 1U : 0U;
        if ((ir_armed != 0U) &&
            (ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (sample.active_count >= TASK1_STOP_MIN_IR_COUNT) &&
            ((line_search_protect == 0U) || (line_stop_ready != 0U))) {
            stop_reason = 1U;
            if (g_task6_c_turn_requested != 0U) {
                g_task6_c_turn_requested = 0U;
                task6_turn_hook_ran = 1U;
                st011_start_pulse(TASK3_POINT_ALARM_MS);
                (void)run_line_fast_turn("TASK6_C_TURN",
                    elapsed_ms,
                    distance_count,
                    &sample,
                    heading_raw,
                    TASK6_C_TURN_TARGET_CDEG,
                    TASK6_C_TURN_B_PWM,
                    TASK6_C_TURN_A_PWM,
                    TASK3_ARC_TURN_LEFT,
                    0U,
                    1U,
                    1U,
                    1U);
            } else if (task4_ac_debug_enabled(tag) != 0U) {
                task4_print_ac_line_debug(tag,
                    elapsed_ms,
                    distance_count,
                    &sample,
                    nav_ok,
                    &nav);
                task6_turn_hook_ran = 1U;
                st011_start_pulse(TASK3_POINT_ALARM_MS);
                (void)run_line_fast_turn(task4_ac_turn_tag(tag),
                    elapsed_ms,
                    distance_count,
                    &sample,
                    heading_raw,
                    TASK6_C_TURN_TARGET_CDEG,
                    TASK6_C_TURN_B_PWM,
                    TASK6_C_TURN_A_PWM,
                    TASK3_ARC_TURN_LEFT,
                    0U,
                    1U,
                    0U,
                    0U);
            } else if (task4_bd_debug_enabled(tag) != 0U) {
                task6_turn_hook_ran = 1U;
                st011_start_pulse(TASK3_POINT_ALARM_MS);
                (void)run_line_fast_turn(task4_bd_turn_tag(tag),
                    elapsed_ms,
                    distance_count,
                    &sample,
                    heading_raw,
                    TASK4_D_TURN_TARGET_CDEG,
                    TASK4_D_TURN_B_PWM,
                    TASK4_D_TURN_A_PWM,
                    TASK3_ARC_TURN_RIGHT,
                    0U,
                    1U,
                    0U,
                    0U);
            }
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
        distance_corr_divisor = (fast_correction != 0U) ?
            TASK2_AB_DISTANCE_CORR_DIVISOR : TASK1_DISTANCE_CORR_DIVISOR;
        distance_corr_max = (fast_correction != 0U) ?
            TASK2_AB_DISTANCE_CORR_MAX : TASK1_DISTANCE_CORR_MAX;
        distance_correction = clamp_i32(distance_error / distance_corr_divisor,
            -distance_corr_max, distance_corr_max);
        balance_correction = speed_correction + distance_correction;
        heading_priority = 0U;

        if ((fast_correction == 0U) &&
            (abs_i32(heading_error) >= TASK1_HEADING_PRIORITY_CDEG) &&
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
            -correction_limit, correction_limit);
        if (fast_correction != 0U) {
            correction = clamp_i32(correction + TASK2_AB_BIAS_CORRECTION,
                -correction_limit, correction_limit);
        }

        base_b_pwm = ramp_i32(TASK1_RAMP_B_START_PWM, STRAIGHT_B_BASE_PWM,
            elapsed_ms, TASK1_START_RAMP_MS);
        base_a_pwm = ramp_i32(TASK1_RAMP_A_START_PWM, STRAIGHT_A_BASE_PWM,
            elapsed_ms, TASK1_START_RAMP_MS);
        search_correction = 0;
        if (search_mode != 0U) {
            base_b_pwm -= search_base_drop;
            base_a_pwm -= search_base_drop;
            if (line_search_protect >= 2U) {
                search_direction =
                    (((elapsed_ms / TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS) & 0x01U) == 0U) ?
                    1 : -1;
            } else {
                search_direction = task2_straight_search_direction(fast_correction,
                    heading_target_cdeg);
            }
            if ((ir_ok != 0U) && (sample.line_lost == 0U)) {
                search_correction = clamp_i32(-(sample.error / search_corr_divisor),
                    -search_corr_max,
                    search_corr_max);
                if (search_correction == 0) {
                    search_correction = search_direction * search_soft_corr;
                }
            } else if (distance_count >= search_sweep_start_count) {
                search_correction = search_direction * search_sweep_corr;
            } else {
                search_correction = search_direction * search_soft_corr;
            }
            correction = clamp_i32(correction + search_correction,
                -correction_limit, correction_limit);
        }

        motor_b_pwm = clamp_i32(base_b_pwm - correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);
        motor_a_pwm = clamp_i32(base_a_pwm + correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);

        TB6612_SetDifferential((int16_t)motor_b_pwm, (int16_t)motor_a_pwm);

        if (report_elapsed_ms >= TASK1_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("%s t=%lu dist=%ld arm=%u arm_cnt=%ld fast=%u find=%u centered=%u stopok=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u ir_err=%ld nav=%u h_raw=%ld h_tgt=%ld h_flt=%ld h_use=%ld h_gain=%ld h_wob=%u gzlp=%ld h_corr=%ld hp=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld search=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                ir_armed,
                line_arm_count,
                fast_correction,
                search_mode,
                line_centered,
                line_stop_ready,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                ir_ok,
                (ir_ok != 0U) ? sample.error : 0,
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
                search_correction,
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

    if (task6_turn_hook_ran == 0U) {
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
            (line_search_protect >= 2U) ? TASK3_STRAIGHT_LINE_ARM_COUNT : TASK1_B_LINE_ARM_COUNT,
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
    }

    return stop_reason;
}

static uint8_t run_line_fast_turn(const char *tag,
    uint32_t ac_elapsed_ms,
    int32_t ac_distance_count,
    const ir_tracking_sample_t *line_sample,
    int32_t line_yaw_cdeg,
    int32_t target_cdeg,
    int16_t motor_b_pwm,
    int16_t motor_a_pwm,
    int32_t handoff_turn_dir,
    uint8_t target_is_absolute,
    uint8_t enable_line_stop,
    uint8_t brake_after_turn,
    uint8_t report_samples)
{
    static uint32_t log_t[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_yaw[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_turn[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_gz[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_b_total[TASK6_C_TURN_SAMPLE_MAX];
    static int32_t log_a_total[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_raw[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_mask[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_count[TASK6_C_TURN_SAMPLE_MAX];
    static uint8_t log_lost[TASK6_C_TURN_SAMPLE_MAX];
    jy62_navigation_t nav = {0};
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint8_t sample_index = 0U;
    uint8_t stop_reason = 0U;
    uint8_t nav_ok;
    uint8_t ir_ok;
    uint8_t line_stop_ready;
    int32_t start_yaw;
    int32_t yaw = 0;
    int32_t turn_cdeg = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t overshoot;
    int32_t handoff_turn;
    int32_t handoff_b_pwm;
    int32_t handoff_a_pwm;
    int32_t turn_abs;
    int32_t turn_target_cdeg;
    uint8_t index;

    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok == 0U) {
        TB6612_Brake();
        lc_printf("%s abort: nav invalid before turn ac_t=%lu ac_dist=%ld\r\n",
            tag,
            ac_elapsed_ms,
            ac_distance_count);
        return 0U;
    }

    start_yaw = nav.yaw_relative_cdeg;
    turn_target_cdeg = (target_is_absolute != 0U) ?
        normalize_cdeg(target_cdeg - start_yaw) : target_cdeg;
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential(motor_b_pwm, motor_a_pwm);

    while (elapsed_ms < TASK6_C_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok == 0U) {
            stop_reason = 4U;
            break;
        }

        yaw = nav.yaw_relative_cdeg;
        turn_cdeg = normalize_cdeg(yaw - start_yaw);
        turn_abs = abs_i32(turn_cdeg);
        ir_ok = IRTracking_ReadSample(&sample);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        line_stop_ready = ((enable_line_stop != 0U) &&
            (turn_abs >= TASK6_C_TURN_LINE_ARM_CDEG) &&
            (ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            ((sample.line_mask & TASK6_C_TURN_LINE_STOP_MASK) != 0U) &&
            (sample.active_count >= TASK6_C_TURN_LINE_STOP_MIN_COUNT)) ? 1U : 0U;

        if ((report_elapsed_ms >= TASK6_C_TURN_REPORT_PERIOD_MS) &&
            (sample_index < TASK6_C_TURN_SAMPLE_MAX)) {
            report_elapsed_ms = 0;
            log_t[sample_index] = elapsed_ms;
            log_yaw[sample_index] = yaw;
            log_turn[sample_index] = turn_cdeg;
            log_gz[sample_index] = nav.gyro_z_filtered_mdps;
            log_b_total[sample_index] = motor_b_total;
            log_a_total[sample_index] = motor_a_total;
            log_raw[sample_index] = (ir_ok != 0U) ? sample.raw : 0xFFU;
            log_mask[sample_index] = (ir_ok != 0U) ? sample.line_mask : 0U;
            log_count[sample_index] = (ir_ok != 0U) ? sample.active_count : 0U;
            log_lost[sample_index] = (ir_ok != 0U) ? sample.line_lost : 1U;
            sample_index++;
        }

        if (((turn_target_cdeg >= 0) && (turn_cdeg >= turn_target_cdeg)) ||
            ((turn_target_cdeg < 0) && (turn_cdeg <= turn_target_cdeg))) {
            stop_reason = 1U;
            break;
        }

        if (line_stop_ready != 0U) {
            stop_reason = 5U;
            break;
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok != 0U) {
        yaw = nav.yaw_relative_cdeg;
        turn_cdeg = normalize_cdeg(yaw - start_yaw);
    }
    ir_ok = IRTracking_ReadSample(&sample);
    overshoot = (turn_target_cdeg >= 0) ?
        (turn_cdeg - turn_target_cdeg) : (turn_target_cdeg - turn_cdeg);

    if ((brake_after_turn != 0U) ||
        ((stop_reason != 1U) && (stop_reason != 5U))) {
        TB6612_Brake();
    } else if (handoff_turn_dir == 0) {
        TB6612_SetDifferential(TASK1_RAMP_B_START_PWM, TASK1_RAMP_A_START_PWM);
    } else {
        handoff_turn = handoff_turn_dir * TASK3_ARC_ENTRY_TURN;
        handoff_b_pwm = clamp_i32((TASK3_ARC_B_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP) + handoff_turn,
            TASK3_ARC_MIN_PWM,
            TASK3_ARC_MAX_PWM);
        handoff_a_pwm = clamp_i32((TASK3_ARC_A_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP) - handoff_turn,
            TASK3_ARC_MIN_PWM,
            TASK3_ARC_MAX_PWM);
        TB6612_SetDifferential((int16_t)handoff_b_pwm, (int16_t)handoff_a_pwm);
    }

    if (report_samples != 0U) {
        lc_printf("%s start: ac_t=%lu ac_dist=%ld line_yaw=%ld line_raw=0x%02X line_mask=0x%02X line_cnt=%u line_err=%ld target=%d turn_target=%ld pwm=%d/%d yaw0=%ld\r\n",
            tag,
            ac_elapsed_ms,
            ac_distance_count,
            line_yaw_cdeg,
            (line_sample != 0) ? line_sample->raw : 0xFFU,
            (line_sample != 0) ? line_sample->line_mask : 0U,
            (line_sample != 0) ? line_sample->active_count : 0U,
            (line_sample != 0) ? line_sample->error : 0,
            target_cdeg,
            turn_target_cdeg,
            motor_b_pwm,
            motor_a_pwm,
            start_yaw);

        for (index = 0U; index < sample_index; index++) {
            lc_printf("%s sample n=%u t=%lu yaw=%ld turn=%ld gzlp=%ld B=%ld A=%ld ir=0x%02X/0x%02X/%u lost=%u\r\n",
                tag,
                index,
                log_t[index],
                log_yaw[index],
                log_turn[index],
                log_gz[index],
                log_b_total[index],
                log_a_total[index],
                log_raw[index],
                log_mask[index],
                log_count[index],
                log_lost[index]);
        }
    }

    lc_printf("%s stop: reason=%s t=%lu yaw=%ld turn=%ld target=%d turn_target=%ld overshoot=%ld nav=%u brake=%u B_total=%ld A_total=%ld ir=0x%02X/0x%02X/%u lost=%u err=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "target" : ((stop_reason == 2U) ? "timeout" : ((stop_reason == 3U) ? "uart_stop" : ((stop_reason == 5U) ? "line" : "nav_invalid"))),
        elapsed_ms,
        yaw,
        turn_cdeg,
        target_cdeg,
        turn_target_cdeg,
        overshoot,
        nav_ok,
        ((brake_after_turn != 0U) ||
            ((stop_reason != 1U) && (stop_reason != 5U))) ? 1U : 0U,
        motor_b_total,
        motor_a_total,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0);

    encoder_reset_distance_counts();
    return ((stop_reason == 1U) || (stop_reason == 5U)) ? 1U : 0U;
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
    arc_turn_dir_t turn_dir,
    uint8_t stop_on_head_exit,
    int32_t finish_count,
    int32_t yaw_done_cdeg)
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
    uint8_t head_exit_virtual = 0U;
    uint8_t head_exit_line_seen = 0U;

    IRTracking_Init();
    straight_pid_reset(&diff_pid);
    straight_pid_set_limits(&diff_pid, TASK2_ARC_PID_I_LIMIT, TASK2_ARC_PID_CORR_MAX);
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = JY62_PeekNavigation(&nav);
    arc_start_yaw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;

    lc_printf("%s start: arc model follow dir=%d stop_on_head_exit=%u R=%dcm W=%dmm sensor_axis=%dmm cpcm=%d arc_len=%ld exit_arm=%ld finish=%ld yaw_done=%ld head_exit=%d+-%d ff_x10=%d model_diff=%ld base=%d/%d\r\n",
        tag,
        (int32_t)turn_dir,
        stop_on_head_exit,
        TASK2_ARC_RADIUS_CM,
        TASK2_ARC_WHEEL_BASE_MM,
        TASK2_ARC_SENSOR_TO_AXIS_MM,
        COUNTS_PER_CM,
        (int32_t)TASK2_ARC_LENGTH_COUNT,
        (int32_t)TASK2_ARC_EXIT_ARM_COUNT,
        finish_count,
        yaw_done_cdeg,
        TASK2_ARC_HEAD_EXIT_CDEG,
        TASK2_ARC_HEAD_EXIT_TOL_CDEG,
        TASK2_ARC_DIFF_FF_GAIN_X10,
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
        uint8_t head_exit_yaw_valid;

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
        head_exit_yaw_valid =
            (yaw_progress >= (TASK2_ARC_HEAD_EXIT_CDEG - TASK2_ARC_HEAD_EXIT_TOL_CDEG)) ?
            1U : 0U;
        if ((line_lost == 0U) && (head_exit_yaw_valid != 0U)) {
            head_exit_line_seen = 1U;
        }

        if ((head_exit_seen == 0U) &&
            (yaw_progress >= (TASK2_ARC_HEAD_EXIT_CDEG + TASK2_ARC_HEAD_EXIT_TOL_CDEG))) {
            head_exit_seen = 1U;
            head_exit_virtual = 1U;
        }

        if ((line_lost != 0U) && (head_exit_yaw_valid != 0U)) {
            head_exit_seen = 1U;
            if ((stop_on_head_exit != 0U) && (head_exit_line_seen != 0U)) {
                stop_reason = 3U;
                break;
            }
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
        feedforward_correction = clamp_i32(-((target_diff * TASK2_ARC_DIFF_FF_GAIN_X10) / 10),
            -TASK2_ARC_PID_CORR_MAX, TASK2_ARC_PID_CORR_MAX);
        pwm_correction = clamp_i32(feedforward_correction + feedback_correction,
            -TASK2_ARC_PID_CORR_MAX, TASK2_ARC_PID_CORR_MAX);

        left_pwm = clamp_i32(TASK2_ARC_B_BASE_PWM - pwm_correction,
            TASK2_ARC_MIN_PWM, TASK2_ARC_MAX_PWM);
        right_pwm = clamp_i32(TASK2_ARC_A_BASE_PWM + pwm_correction,
            TASK2_ARC_MIN_PWM, TASK2_ARC_MAX_PWM);

        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if ((distance_count >= finish_count) &&
            ((nav_ok == 0U) || (yaw_progress >= yaw_done_cdeg))) {
            stop_reason = 3U;
            break;
        }

        if ((head_exit_seen != 0U) &&
            (yaw_progress >= TASK2_ARC_ALIGN_TARGET_CDEG)) {
            stop_reason = 3U;
            break;
        }

        if (distance_count >= (finish_count + 1800L)) {
            stop_reason = 2U;
            break;
        }

        if (report_elapsed_ms >= TASK2_ARC_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("%s t=%lu dist=%ld arc_len=%ld exit=%u vexit=%u seen=%u yaw_ok=%u yaw_rel=%ld yaw_prog=%ld theta=%ld yaw_lag=%ld nav=%u ir=%u lost=%u raw=0x%02X mask=0x%02X cnt=%u ir_err=%ld ir_corr=%ld yaw_corr=%ld model=%ld tgt=%ld B_spd=%ld A_spd=%ld pid_err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld pwm_corr=%ld L=%ld R=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                (int32_t)TASK2_ARC_LENGTH_COUNT,
                head_exit_seen,
                head_exit_virtual,
                head_exit_line_seen,
                head_exit_yaw_valid,
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
    lc_printf("%s stop: reason_id=%u reason=%s t=%lu dist=%ld finish=%ld exit=%u vexit=%u seen=%u yaw_rel=%ld yaw_prog=%ld nav=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        stop_reason,
        (stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "arc_done" : ((stop_reason == 5U) ? "uart_stop" : "timeout")),
        elapsed_ms,
        (abs_i32(motor_b_delta) + abs_i32(motor_a_delta)) / 2,
        finish_count,
        head_exit_seen,
        head_exit_virtual,
        head_exit_line_seen,
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
        1U,
        0,
        0U,
        0U,
        0U,
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
        0U,
        1U,
        1U,
        TASK1_START_ALARM_MS,
        0U);
    if (reason != 1U) {
        lc_printf("TASK2 abort after AB: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_arc_line_follow_segment("TASK2_BC",
        0U,
        ARC_TURN_RIGHT,
        0U,
        TASK2_BC_FINISH_COUNT,
        TASK2_BC_YAW_DONE_CDEG);
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
        0U,
        0U,
        0U);
    if (reason != 1U) {
        lc_printf("TASK2 abort after CD: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_arc_line_follow_segment("TASK2_DA",
        0U,
        ARC_TURN_RIGHT,
        1U,
        TASK2_ARC_FINISH_COUNT,
        TASK2_ARC_YAW_DONE_CDEG);
    if (task2_arc_stop_is_success(reason) == 0U) {
        lc_printf("TASK2 abort after DA: stop_reason=%u\r\n", reason);
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);
    encoder_reset_distance_counts();

    TB6612_Brake();
    lc_printf("TASK2 complete: stopped at A, distance reset\r\n");
}

static uint8_t task3_arc_stop_is_success(uint8_t stop_reason)
{
    return ((stop_reason == 1U) || (stop_reason == 3U)) ? 1U : 0U;
}

static uint8_t run_task3_yaw_align_segment(const char *tag, int32_t target_cdeg)
{
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint8_t stable_count = 0U;
    uint8_t stop_reason = 0U;

    TB6612_Brake();
    delay_ms_with_st011(120);
    lc_printf("%s start: yaw align target=%ld tol=%d fast=%d slow=%d\r\n",
        tag,
        target_cdeg,
        TASK3_ALIGN_TOL_CDEG,
        TASK3_ALIGN_FAST_PWM,
        TASK3_ALIGN_SLOW_PWM);

    while (elapsed_ms < TASK3_ALIGN_TIMEOUT_MS) {
        int32_t yaw_rel;
        int32_t error;
        int32_t turn_pwm;
        uint8_t nav_ok;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 2U;
            break;
        }

        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok == 0U) {
            TB6612_Brake();
            continue;
        }

        yaw_rel = nav.yaw_relative_cdeg;
        error = normalize_cdeg(target_cdeg - yaw_rel);
        if (abs_i32(error) <= TASK3_ALIGN_TOL_CDEG) {
            stable_count++;
            TB6612_Brake();
            if (stable_count >= TASK3_ALIGN_STABLE_COUNT) {
                stop_reason = 1U;
                break;
            }
            continue;
        }

        stable_count = 0U;
        turn_pwm = (abs_i32(error) <= TASK3_ALIGN_SLOW_ZONE_CDEG) ?
            TASK3_ALIGN_SLOW_PWM : TASK3_ALIGN_FAST_PWM;
        if (error > 0) {
            TB6612_SetDifferential((int16_t)-turn_pwm, (int16_t)turn_pwm);
        } else {
            TB6612_SetDifferential((int16_t)turn_pwm, (int16_t)-turn_pwm);
        }

        if ((elapsed_ms % 100U) == 0U) {
            lc_printf("%s t=%lu yaw=%ld target=%ld err=%ld pwm=%ld stable=%u\r\n",
                tag,
                elapsed_ms,
                yaw_rel,
                target_cdeg,
                error,
                turn_pwm,
                stable_count);
        }
    }

    TB6612_Brake();
    (void)JY62_PeekNavigation(&nav);
    lc_printf("%s stop: reason=%s t=%lu yaw=%ld target=%ld err=%ld stable=%u\r\n",
        tag,
        (stop_reason == 1U) ? "aligned" : ((stop_reason == 2U) ? "uart_stop" : "timeout"),
        elapsed_ms,
        nav.yaw_relative_cdeg,
        target_cdeg,
        normalize_cdeg(target_cdeg - nav.yaw_relative_cdeg),
        stable_count);

    return stop_reason;
}

static uint8_t run_task3_arc_line_follow_segment(const char *tag,
    int32_t turn_dir,
    uint8_t stop_on_final_line,
    uint32_t stop_alarm_ms)
{
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t filtered_error = 0;
    int32_t last_filtered_error = 0;
    int32_t last_turn = 0;
    int32_t last_yaw_rel = 0;
    int32_t yaw_progress = 0;
    uint8_t stop_reason = 0U;
    uint8_t line_seen = 0U;
    uint8_t lost_count = 0U;
    uint8_t exit_confirm_count = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t b_exit_heading_latched = 0U;
    uint8_t exit_line_seen = 0U;

    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = JY62_PeekNavigation(&nav);
    last_yaw_rel = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;

    lc_printf("%s start: task3 IR arc dir=%ld base=%d/%d finish=%ld exit_ignore=%ld yaw_done=%d final_arm=%d entry=%d/%d final_line=%u\r\n",
        tag,
        turn_dir,
        TASK3_ARC_B_BASE_PWM,
        TASK3_ARC_A_BASE_PWM,
        (int32_t)TASK3_ARC_FINISH_COUNT,
        (int32_t)TASK3_ARC_EXIT_IGNORE_COUNT,
        TASK3_ARC_YAW_DONE_CDEG,
        TASK3_ARC_EXIT_ARM_CDEG,
        TASK3_ARC_ENTRY_COUNT,
        TASK3_ARC_ENTRY_TURN,
        stop_on_final_line);

    while (elapsed_ms < TASK3_ARC_MAX_RUN_MS) {
        int32_t motor_b_total;
        int32_t motor_a_total;
        int32_t distance_count;
        int32_t raw_error;
        int32_t error;
        int32_t derivative;
        int32_t turn;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t left_pwm;
        int32_t right_pwm;
        int32_t yaw_rel;
        int32_t yaw_delta;
        int32_t yaw_step;
        uint8_t line_valid;
        uint8_t wide_line;
        uint8_t final_line;
        uint8_t entry_zone;
        uint8_t yaw_exit_ready;
        uint8_t distance_exit_ready;
        uint8_t final_exit_ready;
        uint8_t b_exit_heading_ready;
        uint8_t exit_candidate;
        uint8_t exit_confirmed;
        uint8_t mode;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 5U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;

        nav_ok = JY62_PeekNavigation(&nav);
        yaw_rel = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
        if (nav_ok != 0U) {
            yaw_delta = normalize_cdeg(yaw_rel - last_yaw_rel);
            yaw_step = yaw_delta * -turn_dir;
            if (yaw_step > 0) {
                yaw_progress += yaw_step;
            }
            last_yaw_rel = yaw_rel;
        } else {
            yaw_delta = 0;
            yaw_step = 0;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        wide_line = ((ir_ok != 0U) &&
            (sample.active_count >= TASK3_ARC_WIDE_LINE_MIN_COUNT)) ? 1U : 0U;
        line_valid = ((ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (wide_line == 0U)) ? 1U : 0U;
        yaw_exit_ready = (yaw_progress >= TASK3_ARC_YAW_DONE_CDEG) ? 1U : 0U;
        distance_exit_ready = (distance_count >= TASK3_ARC_EXIT_IGNORE_COUNT) ? 1U : 0U;
        if ((distance_exit_ready != 0U) && (line_valid != 0U)) {
            exit_line_seen = 1U;
        }
        final_exit_ready = ((stop_on_final_line != 0U) &&
            (distance_exit_ready != 0U) &&
            (yaw_progress >= TASK3_ARC_EXIT_ARM_CDEG)) ? 1U : 0U;
        b_exit_heading_ready = ((stop_on_final_line == 0U) &&
            (nav_ok != 0U) &&
            (abs_i32(TASK3_B_EXIT_HEADING_TARGET_CDEG - abs_i32(yaw_rel)) <=
             TASK3_B_EXIT_HEADING_TOL_CDEG)) ? 1U : 0U;
        exit_candidate = ((ir_ok != 0U) &&
            ((sample.line_lost != 0U) || (wide_line != 0U))) ? 1U : 0U;
        if ((stop_on_final_line == 0U) &&
            (distance_exit_ready != 0U) &&
            (b_exit_heading_ready != 0U) &&
            (exit_line_seen != 0U)) {
            b_exit_heading_latched = 1U;
        }
        final_line = ((stop_on_final_line != 0U) &&
            (final_exit_ready != 0U) &&
            (exit_line_seen != 0U) &&
            ((exit_candidate != 0U) ||
             ((ir_ok != 0U) &&
              (sample.active_count >= TASK3_ARC_FINAL_LINE_MIN_COUNT)))) ? 1U : 0U;
        entry_zone = (distance_count < TASK3_ARC_ENTRY_COUNT) ? 1U : 0U;
        if ((stop_on_final_line == 0U) &&
            (distance_exit_ready != 0U) &&
            (b_exit_heading_latched != 0U) &&
            (exit_line_seen != 0U) &&
            (exit_candidate != 0U)) {
            if (exit_confirm_count < 255U) {
                exit_confirm_count++;
            }
        } else {
            exit_confirm_count = 0U;
        }
        exit_confirmed = (exit_confirm_count >= TASK3_ARC_EXIT_CONFIRM_COUNT) ? 1U : 0U;

        if (final_line != 0U) {
            stop_reason = (stop_alarm_ms == 0U) ? 3U : 1U;
            break;
        }

        if (exit_confirmed != 0U) {
            stop_reason = 3U;
            break;
        }

        raw_error = 0;
        error = 0;
        derivative = 0;
        mode = 0U;
        base_b_pwm = TASK3_ARC_B_BASE_PWM;
        base_a_pwm = TASK3_ARC_A_BASE_PWM;

        if (line_valid != 0U) {
            line_seen = 1U;
            lost_count = 0U;
            raw_error = sample.error;
            error = (abs_i32(raw_error) < TASK3_ARC_ERROR_DEADBAND) ? 0 : raw_error;
            if ((error != 0) &&
                (filtered_error != 0) &&
                (((error > 0) && (filtered_error < 0)) ||
                 ((error < 0) && (filtered_error > 0)))) {
                filtered_error = error;
            } else {
                filtered_error += (error - filtered_error) / TASK3_ARC_ERROR_FILTER_DIVISOR;
            }
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -TASK3_ARC_DERIVATIVE_LIMIT, TASK3_ARC_DERIVATIVE_LIMIT);
            last_filtered_error = filtered_error;
            if (entry_zone != 0U) {
                turn = (filtered_error / TASK3_ARC_ENTRY_KP_DIVISOR) +
                    (derivative / TASK3_ARC_ENTRY_KD_DIVISOR);
                turn = clamp_i32(turn,
                    -TASK3_ARC_ENTRY_TURN_MAX, TASK3_ARC_ENTRY_TURN_MAX);
            } else {
                turn = (filtered_error / TASK3_ARC_KP_DIVISOR) +
                    (derivative / TASK3_ARC_KD_DIVISOR);
                turn = clamp_i32(turn, -TASK3_ARC_TURN_MAX, TASK3_ARC_TURN_MAX);
            }
        } else {
            if (lost_count < 255U) {
                lost_count++;
            }
            mode = 3U;
            base_b_pwm -= TASK3_ARC_LOST_BASE_DROP;
            base_a_pwm -= TASK3_ARC_LOST_BASE_DROP;
            if (entry_zone != 0U) {
                turn = turn_dir * TASK3_ARC_ENTRY_LOST_TURN;
            } else if (line_seen != 0U) {
                turn = clamp_i32(last_turn,
                    -TASK3_ARC_LOST_TURN, TASK3_ARC_LOST_TURN);
                if (turn == 0) {
                    turn = turn_dir * TASK3_ARC_LOST_TURN;
                }
            } else {
                turn = turn_dir * TASK3_ARC_LOST_TURN;
            }
        }

        if (entry_zone != 0U) {
            mode = (line_valid != 0U) ? 2U : 1U;
            base_b_pwm = TASK3_ARC_B_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP;
            base_a_pwm = TASK3_ARC_A_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP;
            turn = clamp_i32(turn + (turn_dir * TASK3_ARC_ENTRY_TURN),
                -TASK3_ARC_ENTRY_TURN_MAX, TASK3_ARC_ENTRY_TURN_MAX);
        }
        if (line_valid != 0U) {
            last_turn = turn;
        }

        left_pwm = clamp_i32(base_b_pwm + turn,
            TASK3_ARC_MIN_PWM, TASK3_ARC_MAX_PWM);
        right_pwm = clamp_i32(base_a_pwm - turn,
            TASK3_ARC_MIN_PWM, TASK3_ARC_MAX_PWM);
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (distance_count >= TASK3_ARC_FORCE_STOP_COUNT) {
            stop_reason = 2U;
            break;
        }

        if (report_elapsed_ms >= TASK3_ARC_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            lc_printf("%s t=%lu dist=%ld yaw=%ld ystep=%ld yprog=%ld nav=%u ir=%u valid=%u wide=%u final=%u yrdy=%u drdy=%u frdy=%u brdy=%u bmark=%u xcnt=%u seen=%u xseen=%u lost_cnt=%u entry=%u mode=%u raw=0x%02X mask=0x%02X cnt=%u err=%ld filt=%ld der=%ld turn=%ld L=%ld R=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                yaw_rel,
                yaw_step,
                yaw_progress,
                nav_ok,
                ir_ok,
                line_valid,
                wide_line,
                final_line,
                yaw_exit_ready,
                distance_exit_ready,
                final_exit_ready,
                b_exit_heading_ready,
                b_exit_heading_latched,
                exit_confirm_count,
                line_seen,
                exit_line_seen,
                lost_count,
                entry_zone,
                mode,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                error,
                filtered_error,
                derivative,
                turn,
                left_pwm,
                right_pwm);
        }
    }

    if ((stop_alarm_ms != 0U) || (stop_reason != 3U)) {
        TB6612_Brake();
    }
    encoder_get_total_counts(&motor_b_delta, &motor_a_delta);
    nav_ok = JY62_PeekNavigation(&nav);
    lc_printf("%s stop: reason_id=%u reason=%s t=%lu dist=%ld yaw=%ld yprog=%ld bmark=%u xseen=%u xcnt=%u nav=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u\r\n",
        tag,
        stop_reason,
        (stop_reason == 1U) ? "line" : ((stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "arc_done" : ((stop_reason == 5U) ? "uart_stop" : "timeout"))),
        elapsed_ms,
        (abs_i32(motor_b_delta) + abs_i32(motor_a_delta)) / 2,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        yaw_progress,
        b_exit_heading_latched,
        exit_line_seen,
        exit_confirm_count,
        nav_ok,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U);

    if ((stop_alarm_ms != 0U) && (stop_reason == 1U)) {
        st011_pulse(stop_alarm_ms);
    }

    return stop_reason;
}

static void run_task3_acbda(void)
{
    jy62_navigation_t nav;
    int32_t b_exit_heading;
    int32_t bd_heading_target;
    uint8_t reason;
    uint8_t nav_ok;

    lc_printf("TASK3 start: A->C straight, C->B IR arc, B->D relative straight, D->A immediate IR arc. BAC=%d corr=%d B_rel=%d sensor=%dmm\r\n",
        TASK3_BAC_THEORY_CDEG,
        TASK3_AC_TANGENT_CORR_CDEG,
        TASK3_BD_RELATIVE_TURN_CDEG,
        TASK3_SENSOR_TO_AXIS_MM);

    reason = run_straight_to_line_segment("TASK3_AC",
        1U,
        0,
        0U,
        0U,
        2U,
        TASK1_START_ALARM_MS,
        0U);
    if (reason != 1U) {
        lc_printf("TASK3 abort after AC: stop_reason=%u\r\n", reason);
        return;
    }
    /* C 点：AC 直线扫到入线后触发。 */
    st011_start_pulse(TASK3_POINT_ALARM_MS);

    reason = run_task3_arc_line_follow_segment("TASK3_CB",
        TASK3_ARC_TURN_LEFT,
        0U,
        0U);
    if (reason != 3U) {
        lc_printf("TASK3 abort after CB: stop_reason=%u\r\n", reason);
        return;
    }
    /* B 点：CB 弧线确认出线后触发声光，不刹车，直接进入 BD 边走边转。 */
    st011_start_pulse(TASK3_POINT_ALARM_MS);

    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok == 0U) {
        TB6612_Brake();
        lc_printf("TASK3 abort after CB: JY62 invalid before BD\r\n");
        return;
    }
    b_exit_heading = nav.yaw_relative_cdeg;
    bd_heading_target = normalize_cdeg(b_exit_heading + TASK3_BD_RELATIVE_TURN_CDEG);
    lc_printf("TASK3_BD target: B_exit=%ld B_nom=%d rel_turn=%d target=%ld mode=relative\r\n",
        b_exit_heading,
        TASK3_B_EXIT_HEADING_TARGET_CDEG,
        TASK3_BD_RELATIVE_TURN_CDEG,
        bd_heading_target);

    reason = run_straight_to_line_segment("TASK3_BD",
        0U,
        bd_heading_target,
        1U,
        0U,
        2U,
        0U,
        0U);
    if (reason != 1U) {
        lc_printf("TASK3 abort after BD: stop_reason=%u\r\n", reason);
        return;
    }
    /* D 点：BD 直线扫到入线后触发。 */
    reason = run_task3_arc_line_follow_segment("TASK3_DA",
        TASK3_ARC_TURN_RIGHT,
        1U,
        TASK1_FINISH_ALARM_MS);
    if (task3_arc_stop_is_success(reason) == 0U) {
        lc_printf("TASK3 abort after DA: stop_reason=%u\r\n", reason);
        return;
    }

    encoder_reset_distance_counts();
    TB6612_Brake();
    lc_printf("TASK3 complete: stopped at A, distance reset\r\n");
}

static const char *task4_segment_tag(uint8_t lap_index, uint8_t segment_index)
{
    static const char * const tags[TASK4_LAP_COUNT][4] = {
        {"TASK4_L1_AC", "TASK4_L1_CB", "TASK4_L1_BD", "TASK4_L1_DA"},
        {"TASK4_L2_AC", "TASK4_L2_CB", "TASK4_L2_BD", "TASK4_L2_DA"},
        {"TASK4_L3_AC", "TASK4_L3_CB", "TASK4_L3_BD", "TASK4_L3_DA"},
        {"TASK4_L4_AC", "TASK4_L4_CB", "TASK4_L4_BD", "TASK4_L4_DA"}
    };

    if ((lap_index < TASK4_LAP_COUNT) && (segment_index < 4U)) {
        return tags[lap_index][segment_index];
    }

    return "TASK4_SEG";
}

static uint8_t run_task4_lap(uint8_t lap_index,
    int32_t ac_heading_target,
    uint8_t zero_ac_heading,
    uint8_t final_lap,
    int32_t *a_exit_heading_out)
{
    jy62_navigation_t nav;
    const char *tag_ac = task4_segment_tag(lap_index, 0U);
    const char *tag_cb = task4_segment_tag(lap_index, 1U);
    const char *tag_bd = task4_segment_tag(lap_index, 2U);
    const char *tag_da = task4_segment_tag(lap_index, 3U);
    int32_t b_exit_heading;
    int32_t bd_heading_target;
    uint8_t reason;
    uint8_t nav_ok;

    lc_printf("TASK4 lap=%u start: AC_target=%ld zero=%u final=%u\r\n",
        (uint8_t)(lap_index + 1U),
        ac_heading_target,
        zero_ac_heading,
        final_lap);

    if (zero_ac_heading == 0U) {
        lc_printf("%s preturn: lap=%u rel_turn=%d target=%ld pwm=%d/%d\r\n",
            tag_ac,
            (uint8_t)(lap_index + 1U),
            TASK4_AC_RELATIVE_TURN_CDEG,
            ac_heading_target,
            TASK4_A_TURN_B_PWM,
            TASK4_A_TURN_A_PWM);
        reason = run_line_fast_turn(task4_a_turn_tag(lap_index),
            0U,
            0,
            0,
            ac_heading_target,
            ac_heading_target,
            TASK4_A_TURN_B_PWM,
            TASK4_A_TURN_A_PWM,
            0,
            1U,
            0U,
            0U,
            0U);
        if (reason == 0U) {
            lc_printf("TASK4 abort before %s: lap=%u A preturn failed\r\n",
                tag_ac,
                (uint8_t)(lap_index + 1U));
            return 0U;
        }
    }

    reason = run_straight_to_line_segment(tag_ac,
        zero_ac_heading,
        ac_heading_target,
        (zero_ac_heading == 0U) ? 1U : 0U,
        0U,
        (zero_ac_heading == 0U) ? TASK4_AC_LINE_SEARCH_PROTECT : 2U,
        (zero_ac_heading != 0U) ? TASK1_START_ALARM_MS : 0U,
        0U);
    if (reason != 1U) {
        lc_printf("TASK4 abort after %s: lap=%u stop_reason=%u\r\n",
            tag_ac,
            (uint8_t)(lap_index + 1U),
            reason);
        return 0U;
    }

    reason = run_task3_arc_line_follow_segment(tag_cb,
        TASK3_ARC_TURN_LEFT,
        0U,
        0U);
    if (reason != 3U) {
        lc_printf("TASK4 abort after %s: lap=%u stop_reason=%u\r\n",
            tag_cb,
            (uint8_t)(lap_index + 1U),
            reason);
        return 0U;
    }
    st011_start_pulse(TASK3_POINT_ALARM_MS);

    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok == 0U) {
        TB6612_Brake();
        lc_printf("TASK4 abort after %s: JY62 invalid before BD\r\n", tag_cb);
        return 0U;
    }
    b_exit_heading = nav.yaw_relative_cdeg;
    bd_heading_target = normalize_cdeg(b_exit_heading + TASK3_BD_RELATIVE_TURN_CDEG);
    lc_printf("%s target: lap=%u B_exit=%ld rel_turn=%d target=%ld mode=relative\r\n",
        tag_bd,
        (uint8_t)(lap_index + 1U),
        b_exit_heading,
        TASK3_BD_RELATIVE_TURN_CDEG,
        bd_heading_target);

    reason = run_straight_to_line_segment(tag_bd,
        0U,
        bd_heading_target,
        1U,
        0U,
        2U,
        0U,
        0U);
    if (reason != 1U) {
        lc_printf("TASK4 abort after %s: lap=%u stop_reason=%u\r\n",
            tag_bd,
            (uint8_t)(lap_index + 1U),
            reason);
        return 0U;
    }

    reason = run_task3_arc_line_follow_segment(tag_da,
        TASK3_ARC_TURN_RIGHT,
        1U,
        (final_lap != 0U) ? TASK1_FINISH_ALARM_MS : 0U);
    if (task3_arc_stop_is_success(reason) == 0U) {
        lc_printf("TASK4 abort after %s: lap=%u stop_reason=%u\r\n",
            tag_da,
            (uint8_t)(lap_index + 1U),
            reason);
        return 0U;
    }

    if (final_lap != 0U) {
        return 1U;
    }

    st011_start_pulse(TASK3_POINT_ALARM_MS);
    nav_ok = JY62_PeekNavigation(&nav);
    if (nav_ok == 0U) {
        TB6612_Brake();
        lc_printf("TASK4 abort after %s: JY62 invalid before next AC\r\n", tag_da);
        return 0U;
    }

    *a_exit_heading_out = nav.yaw_relative_cdeg;
    lc_printf("TASK4_A target seed: lap=%u A_exit=%ld rel_turn=%d next_target=%ld mode=relative\r\n",
        (uint8_t)(lap_index + 1U),
        *a_exit_heading_out,
        TASK4_AC_RELATIVE_TURN_CDEG,
        normalize_cdeg(*a_exit_heading_out + TASK4_AC_RELATIVE_TURN_CDEG));

    return 1U;
}

static void run_task4_four_laps(void)
{
    uint8_t lap_index;
    int32_t ac_heading_target = 0;
    int32_t a_exit_heading = 0;

    lc_printf("TASK4 start: task3 route x%d, B_rel=%d, A_rel=%d\r\n",
        TASK4_LAP_COUNT,
        TASK3_BD_RELATIVE_TURN_CDEG,
        TASK4_AC_RELATIVE_TURN_CDEG);

    for (lap_index = 0U; lap_index < TASK4_LAP_COUNT; lap_index++) {
        uint8_t final_lap = ((lap_index + 1U) >= TASK4_LAP_COUNT) ? 1U : 0U;
        uint8_t zero_ac_heading = (lap_index == 0U) ? 1U : 0U;

        if (run_task4_lap(lap_index,
            ac_heading_target,
            zero_ac_heading,
            final_lap,
            &a_exit_heading) == 0U) {
            TB6612_Brake();
            return;
        }

        if (final_lap == 0U) {
            ac_heading_target = normalize_cdeg(a_exit_heading + TASK4_AC_RELATIVE_TURN_CDEG);
        }
    }

    encoder_reset_distance_counts();
    TB6612_Brake();
    lc_printf("TASK4 complete: %d laps stopped at A, distance reset\r\n", TASK4_LAP_COUNT);
}

static void run_task6_ac_c_turn_test(void)
{
    uint8_t reason;

    lc_printf("TASK6 start: UART 06, run task3 AC only, then fast left turn at C target=%d pwm=%d/%d\r\n",
        TASK6_C_TURN_TARGET_CDEG,
        TASK6_C_TURN_B_PWM,
        TASK6_C_TURN_A_PWM);

    g_task6_c_turn_requested = 1U;
    reason = run_straight_to_line_segment("TASK6_AC",
        1U,
        0,
        0U,
        0U,
        2U,
        TASK1_START_ALARM_MS,
        0U);
    g_task6_c_turn_requested = 0U;

    if (reason != 1U) {
        TB6612_Brake();
        encoder_reset_distance_counts();
        lc_printf("TASK6 abort after AC: stop_reason=%u\r\n", reason);
        return;
    }

    TB6612_Brake();
    encoder_reset_distance_counts();
    lc_printf("TASK6 complete: AC line detected, C fast turn test finished\r\n");
}

static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: A26/UART0 01=task1, A24/UART0 02=task2, B24/UART0 03=task3, A22/UART0 04=task4, UART0 05=PWM test, UART0 06=C turn test, UART0 00=stop\r\n");

    while (1) {
        task_id = wait_task_uart_command();

        if (task_id == TASK_ID_STOP) {
            TB6612_Brake();
            continue;
        }

        if (task_id == TASK_ID_2) {
            TB6612_Brake();
            (void)jy62_zero_to_current("task_start_zero", 0U);
        } else if ((task_id == TASK_ID_1) ||
            (task_id == TASK_ID_3) || (task_id == TASK_ID_4)) {
            TB6612_Brake();
        }

        if (task_id == TASK_ID_1) {
            run_task1_ab();
        } else if (task_id == TASK_ID_2) {
            run_task2_abcd();
        } else if (task_id == TASK_ID_3) {
            run_task3_acbda();
        } else if (task_id == TASK_ID_4) {
            run_task4_four_laps();
        } else if (task_id == TASK_ID_5) {
            TB6612_Brake();
            run_motor_pid_stream();
        } else if (task_id == TASK_ID_6) {
            TB6612_Brake();
            run_task6_ac_c_turn_test();
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
