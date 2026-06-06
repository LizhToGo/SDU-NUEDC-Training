#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "app_config.h"
#include "app_control.h"
#include "app_straight.h"
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

#if TASK11_UART_LOG_ENABLE
#define task11_log_printf(...) lc_printf(__VA_ARGS__)
#else
#define task11_log_printf(...) ((void)0)
#endif

#if TASK11_REALTIME_EVENT_LOG_ENABLE
#define task11_event_printf(...) lc_printf(__VA_ARGS__)
#else
#define task11_event_printf(...) ((void)0)
#endif

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
    TASK_ID_7 = 7,
    TASK_ID_10 = 10,
    TASK_ID_11 = 11,
    TASK_ID_STOP = 255
} task_id_t;

static task_id_t task_uart_command_from_number(uint8_t number)
{
    switch (number) {
    case 0:
        return TASK_ID_STOP;
    case 1:
        return TASK_ID_1;
    case 2:
        return TASK_ID_2;
    case 3:
        return TASK_ID_3;
    case 4:
        return TASK_ID_4;
    case 5:
        return TASK_ID_5;
    case 6:
        return TASK_ID_6;
    case 7:
        return TASK_ID_7;
    case 10:
        return TASK_ID_10;
    case 11:
        return TASK_ID_11;
    default:
        return TASK_ID_NONE;
    }
}

static task_id_t task_uart_read_command(void)
{
    static uint8_t ascii_value = 0U;
    static uint8_t ascii_count = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        uint8_t ch = DL_UART_Main_receiveData(UART_0_INST);

        if (ch == 0x00U) {
            ascii_value = 0U;
            ascii_count = 0U;
            return TASK_ID_STOP;
        }

        if ((ch == '\r') || (ch == '\n') || (ch == ' ') || (ch == '\t')) {
            ascii_value = 0U;
            ascii_count = 0U;
            continue;
        }

        if ((ch >= 0x01U) && (ch <= 0x07U)) {
            ascii_value = 0U;
            ascii_count = 0U;
            return (task_id_t)ch;
        }

        if ((ch >= '0') && (ch <= '9')) {
            task_id_t command;

            if (ascii_count == 0U) {
                ascii_value = (uint8_t)(ch - '0');
                ascii_count = 1U;
                continue;
            }

            ascii_value = (uint8_t)((ascii_value * 10U) + (ch - '0'));
            command = task_uart_command_from_number(ascii_value);
            ascii_value = 0U;
            ascii_count = 0U;
            if (command != TASK_ID_NONE) {
                return command;
            }
            continue;
        }

        ascii_value = 0U;
        ascii_count = 0U;
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
    int32_t start_force_stop_count;
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
        start_force_stop_count = TASK3_STRAIGHT_FORCE_STOP_COUNT;
        start_search_start_count = TASK3_STRAIGHT_SEARCH_START_COUNT;
        start_search_sweep_start_count = TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        start_search_sweep_period_ms = TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS;
    } else if (line_search_protect != 0U) {
        start_line_arm_count = TASK1_B_LINE_ARM_COUNT;
        start_force_stop_count = TASK1_FORCE_STOP_COUNT;
        start_search_start_count = TASK2_STRAIGHT_SEARCH_START_COUNT;
        start_search_sweep_start_count = TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT;
        start_search_sweep_period_ms = 0;
    } else {
        start_line_arm_count = TASK1_B_LINE_ARM_COUNT;
        start_force_stop_count = TASK1_FORCE_STOP_COUNT;
        start_search_start_count = 0;
        start_search_sweep_start_count = 0;
        start_search_sweep_period_ms = 0;
    }

    lc_printf("%s start: zero=%u heading_only=%u fast=%u line_protect=%u B_base=%d A_base=%d ramp=%d/%d h_div=%ld h_max=%ld d_div=%ld d_max=%ld arm=%ld force=%ld search=%ld sweep=%ld period=%ld\r\n",
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
        start_force_stop_count,
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
        uint8_t task4_ac_start_boost;
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
        task4_ac_start_boost = 0U;
        line_arm_count = (line_search_protect >= 2U) ?
            TASK3_STRAIGHT_LINE_ARM_COUNT : TASK1_B_LINE_ARM_COUNT;
        ir_armed = (distance_count >= line_arm_count) ? 1U : 0U;
        nav_ok = JY62_PeekNavigation(&nav);
        heading_raw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
        heading_gyro_z = (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0;
        heading_raw_error = (nav_ok != 0U) ?
            normalize_cdeg(heading_raw - heading_target_cdeg) : 0;
        task4_ac_start_boost = ((heading_only != 0U) &&
            (nav_ok != 0U) &&
            (task4_ac_debug_enabled(tag) != 0U) &&
            (tag[7] != '1') &&
            (distance_count < TASK4_AC_START_TURN_COUNT) &&
            (abs_i32(heading_raw_error) >= TASK4_AC_START_BOOST_MIN_ERR_CDEG)) ? 1U : 0U;
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
        if (task4_ac_start_boost != 0U) {
            heading_corr_divisor = TASK4_AC_START_HEADING_CORR_DIVISOR;
            heading_corr_max = TASK4_AC_START_HEADING_CORR_MAX;
        } else if (fast_correction != 0U) {
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
        correction_limit = (task4_ac_start_boost != 0U) ?
            TASK4_AC_START_CORR_MAX :
            (((line_search_protect >= 2U) || (heading_only != 0U)) ?
                TASK3_STRAIGHT_CORR_MAX : STRAIGHT_CORR_MAX);
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
                    0U,
                    0U,
                    0U);
            }
            break;
        }

        if (distance_count >= start_force_stop_count) {
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
        if ((fast_correction == 0U) &&
            (heading_only == 0U) &&
            (line_search_protect == 0U) &&
            (distance_count >= TASK1_APPROACH_SLOW_COUNT)) {
            base_b_pwm = TASK1_APPROACH_B_BASE_PWM;
            base_a_pwm = TASK1_APPROACH_A_BASE_PWM;
        }
        search_correction = 0;
        if (search_mode != 0U) {
            base_b_pwm -= search_base_drop;
            base_a_pwm -= search_base_drop;
            if (line_search_protect >= 2U) {
                search_direction = 0;
            } else {
                search_direction = task2_straight_search_direction(fast_correction,
                    heading_target_cdeg);
            }
            if ((ir_ok != 0U) && (sample.line_lost == 0U)) {
                search_correction = clamp_i32(-(sample.error / search_corr_divisor),
                    -search_corr_max,
                    search_corr_max);
                if ((search_correction == 0) && (search_direction != 0)) {
                    search_correction = search_direction * search_soft_corr;
                }
            } else if ((line_search_protect < 2U) &&
                (distance_count >= search_sweep_start_count)) {
                search_correction = search_direction * search_sweep_corr;
            } else if (line_search_protect < 2U) {
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
            lc_printf("%s t=%lu dist=%ld arm=%u arm_cnt=%ld fast=%u find=%u centered=%u stopok=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u ir_err=%ld nav=%u h_raw=%ld h_tgt=%ld h_flt=%ld h_use=%ld h_gain=%ld h_wob=%u gzlp=%ld h_corr=%ld hp=%u boost=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld search=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
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
                task4_ac_start_boost,
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
            start_force_stop_count,
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
    int32_t arc_start_yaw = 0;
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
    arc_start_yaw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;

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
        int32_t previous_yaw_progress;
        int32_t theta_ref;
        int32_t yaw_lag;
        int32_t yaw_correction;
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
        uint8_t entry_edge_recover;
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
            previous_yaw_progress = yaw_progress;
            yaw_delta = normalize_cdeg(yaw_rel - arc_start_yaw);
            yaw_progress = abs_i32(yaw_delta);
            yaw_step = yaw_progress - previous_yaw_progress;
            if (yaw_step < 0) {
                yaw_step = 0;
            }
        } else {
            yaw_delta = 0;
            yaw_step = 0;
        }
        theta_ref = (int32_t)(((int64_t)distance_count * 18000LL) /
            (int64_t)TASK3_ARC_LENGTH_COUNT);
        theta_ref = clamp_i32(theta_ref, 0, 18000);
        yaw_lag = (nav_ok != 0U) ? (theta_ref - yaw_progress) : 0;
        yaw_correction = clamp_i32((yaw_lag * turn_dir) / TASK3_ARC_YAW_CORR_DIVISOR,
            -TASK3_ARC_YAW_CORR_MAX, TASK3_ARC_YAW_CORR_MAX);

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
        entry_edge_recover = 0U;
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
                if ((((turn_dir < 0) && (raw_error <= -TASK3_ARC_ENTRY_EDGE_ERROR)) ||
                     ((turn_dir > 0) && (raw_error >= TASK3_ARC_ENTRY_EDGE_ERROR)))) {
                    entry_edge_recover = 1U;
                    turn = turn_dir * TASK3_ARC_ENTRY_EDGE_RECOVER_TURN;
                }
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
                if (line_seen != 0U) {
                    turn = clamp_i32(last_turn,
                        -TASK3_ARC_ENTRY_LOST_TURN, TASK3_ARC_ENTRY_LOST_TURN);
                    if (turn == 0) {
                        turn = turn_dir * TASK3_ARC_ENTRY_LOST_TURN;
                    }
                } else {
                    turn = turn_dir * TASK3_ARC_ENTRY_LOST_TURN;
                }
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
            mode = (entry_edge_recover != 0U) ? 4U : ((line_valid != 0U) ? 2U : 1U);
            base_b_pwm = TASK3_ARC_B_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP;
            base_a_pwm = TASK3_ARC_A_BASE_PWM - TASK3_ARC_ENTRY_BASE_DROP;
            if (entry_edge_recover == 0U) {
                turn = clamp_i32(turn + (turn_dir * TASK3_ARC_ENTRY_TURN),
                    -TASK3_ARC_ENTRY_TURN_MAX, TASK3_ARC_ENTRY_TURN_MAX);
            }
        }
        turn = clamp_i32(turn + yaw_correction,
            (entry_zone != 0U) ? -TASK3_ARC_ENTRY_TURN_MAX : -TASK3_ARC_TURN_MAX,
            (entry_zone != 0U) ? TASK3_ARC_ENTRY_TURN_MAX : TASK3_ARC_TURN_MAX);
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
            lc_printf("%s t=%lu dist=%ld yaw=%ld ystep=%ld yprog=%ld theta=%ld ylag=%ld ycorr=%ld nav=%u ir=%u valid=%u wide=%u final=%u yrdy=%u drdy=%u frdy=%u brdy=%u bmark=%u xcnt=%u seen=%u xseen=%u lost_cnt=%u entry=%u mode=%u raw=0x%02X mask=0x%02X cnt=%u err=%ld filt=%ld der=%ld turn=%ld L=%ld R=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                yaw_rel,
                yaw_step,
                yaw_progress,
                theta_ref,
                yaw_lag,
                yaw_correction,
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

    lc_printf("TASK3 start: A->C straight, C->B IR arc, B->D absolute straight, D->A immediate IR arc. AC_abs=%d BD_abs=%d sensor=%dmm\r\n",
        TASK3_AC_HEADING_TARGET_CDEG,
        TASK3_BD_HEADING_TARGET_CDEG,
        TASK3_SENSOR_TO_AXIS_MM);

    reason = run_straight_to_line_segment("TASK3_AC",
        0U,
        TASK3_AC_HEADING_TARGET_CDEG,
        1U,
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
    bd_heading_target = TASK3_BD_HEADING_TARGET_CDEG;
    lc_printf("TASK3_BD target: B_exit=%ld target=%ld mode=absolute\r\n",
        b_exit_heading,
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

    reason = run_straight_to_line_segment(tag_ac,
        zero_ac_heading,
        ac_heading_target,
        (zero_ac_heading == 0U) ? 1U : 0U,
        0U,
        (zero_ac_heading == 0U) ? TASK4_AC_LINE_SEARCH_PROTECT : 2U,
        (lap_index == 0U) ? TASK1_START_ALARM_MS : 0U,
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
    bd_heading_target = TASK4_BD_HEADING_TARGET_CDEG;
    lc_printf("%s target: lap=%u B_exit=%ld target=%ld mode=absolute\r\n",
        tag_bd,
        (uint8_t)(lap_index + 1U),
        b_exit_heading,
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
    lc_printf("TASK4_A target seed: lap=%u A_exit=%ld next_target=%d mode=absolute\r\n",
        (uint8_t)(lap_index + 1U),
        *a_exit_heading_out,
        TASK4_AC_HEADING_TARGET_CDEG);

    return 1U;
}

static void run_task4_four_laps(void)
{
    uint8_t lap_index;
    int32_t ac_heading_target = TASK4_AC_HEADING_TARGET_CDEG;
    int32_t a_exit_heading = 0;

    lc_printf("TASK4 start: task3 route x%d, AC_abs=%d, BD_abs=%d\r\n",
        TASK4_LAP_COUNT,
        TASK4_AC_HEADING_TARGET_CDEG,
        TASK4_BD_HEADING_TARGET_CDEG);

    for (lap_index = 0U; lap_index < TASK4_LAP_COUNT; lap_index++) {
        uint8_t final_lap = ((lap_index + 1U) >= TASK4_LAP_COUNT) ? 1U : 0U;
        uint8_t zero_ac_heading = 0U;

        if (run_task4_lap(lap_index,
            ac_heading_target,
            zero_ac_heading,
            final_lap,
            &a_exit_heading) == 0U) {
            TB6612_Brake();
            return;
        }

        if (final_lap == 0U) {
            ac_heading_target = TASK4_AC_HEADING_TARGET_CDEG;
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
        0U,
        TASK3_AC_HEADING_TARGET_CDEG,
        1U,
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

typedef struct {
    uint8_t reason;
    uint32_t elapsed_ms;
    int32_t distance_count;
    int32_t yaw_cdeg;
    int32_t yaw_progress_cdeg;
    uint8_t ir_ok;
    ir_tracking_sample_t sample;
} task11_line_result_t;

static const char *task11_reason_name(uint8_t reason)
{
    if (reason == 0U) {
        return "none";
    }
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
    return "timeout";
}

static const char *task11_phase_name(uint8_t phase)
{
    if (phase == 0U) {
        return "AC";
    }
    if (phase == 1U) {
        return "CB";
    }
    if (phase == 2U) {
        return "BD";
    }
    return "DA";
}

enum {
    TASK11_RAM_EVENT_START = 1,
    TASK11_RAM_EVENT_POINT = 2,
    TASK11_RAM_EVENT_FORCE = 3,
    TASK11_RAM_EVENT_ADVANCE_START = 4,
    TASK11_RAM_EVENT_ADVANCE_STOP = 5,
    TASK11_RAM_EVENT_TURN_START = 6,
    TASK11_RAM_EVENT_TURN_STOP = 7,
    TASK11_RAM_EVENT_COMPLETE = 8
};

#if TASK11_RAM_LOG_ENABLE
typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    int16_t yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t line_error;
    int16_t filtered_error;
    int16_t ir_turn;
    int16_t target_diff;
    int16_t speed_diff;
    int16_t pd_error;
    int16_t correction;
    int16_t pwm_b;
    int16_t pwm_a;
    uint8_t lap;
    uint8_t phase;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
} task11_window_log_t;

typedef struct {
    uint32_t t_ms;
    uint16_t dist_count;
    int16_t yaw_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t gzlp_x100_mdps;
    int16_t line_error;
    int16_t motor_b_total;
    int16_t motor_a_total;
    uint8_t lap;
    uint8_t phase;
    uint8_t event;
    uint8_t reason;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    uint8_t flags;
} task11_event_log_t;

typedef struct {
    uint32_t start_ms;
    uint32_t end_ms;
    uint16_t dist_count;
    uint16_t sample_count;
    uint16_t lost_count;
    uint16_t max_abs_error;
    int16_t yaw_start_cdeg;
    int16_t yaw_end_cdeg;
    int16_t yaw_progress_cdeg;
    int16_t avg_abs_error;
    int16_t avg_speed_diff;
    int16_t avg_pd_error;
    int16_t avg_correction;
    int16_t avg_ir_turn;
    int16_t avg_target_diff;
    uint8_t lap;
    uint8_t phase;
    uint8_t reason;
    uint8_t point_mask;
} task11_summary_log_t;

typedef struct {
    uint32_t start_ms;
    uint32_t sample_count;
    uint32_t lost_count;
    int32_t yaw_start_cdeg;
    int32_t sum_abs_error;
    int32_t max_abs_error;
    int32_t sum_speed_diff;
    int32_t sum_pd_error;
    int32_t sum_correction;
    int32_t sum_ir_turn;
    int32_t sum_target_diff;
    uint8_t lap;
    uint8_t phase;
} task11_segment_accum_t;

static task11_window_log_t g_task11_window_log[TASK11_RAM_WINDOW_CAPACITY];
static task11_event_log_t g_task11_event_log[TASK11_RAM_EVENT_CAPACITY];
static task11_summary_log_t g_task11_summary_log[TASK11_RAM_SUMMARY_CAPACITY];
static task11_segment_accum_t g_task11_segment_accum;
static uint16_t g_task11_window_log_count;
static uint16_t g_task11_event_log_count;
static uint8_t g_task11_summary_log_count;
static uint16_t g_task11_window_log_overflow;
static uint16_t g_task11_event_log_overflow;
static uint8_t g_task11_summary_log_overflow;
static uint8_t g_task11_log_lap;
static uint8_t g_task11_log_phase;

static int16_t task11_sat_i16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t task11_sat_u16(int32_t value)
{
    if (value <= 0) {
        return 0U;
    }
    if (value > 65535) {
        return 65535U;
    }
    return (uint16_t)value;
}

static const char *task11_ram_event_name(uint8_t event)
{
    if (event == TASK11_RAM_EVENT_START) {
        return "start";
    }
    if (event == TASK11_RAM_EVENT_POINT) {
        return "point";
    }
    if (event == TASK11_RAM_EVENT_FORCE) {
        return "force";
    }
    if (event == TASK11_RAM_EVENT_ADVANCE_START) {
        return "advance_start";
    }
    if (event == TASK11_RAM_EVENT_ADVANCE_STOP) {
        return "advance_stop";
    }
    if (event == TASK11_RAM_EVENT_TURN_START) {
        return "turn_start";
    }
    if (event == TASK11_RAM_EVENT_TURN_STOP) {
        return "turn_stop";
    }
    if (event == TASK11_RAM_EVENT_COMPLETE) {
        return "complete";
    }
    return "unknown";
}

static void task11_ram_log_reset(void)
{
    g_task11_window_log_count = 0U;
    g_task11_event_log_count = 0U;
    g_task11_summary_log_count = 0U;
    g_task11_window_log_overflow = 0U;
    g_task11_event_log_overflow = 0U;
    g_task11_summary_log_overflow = 0U;
    g_task11_log_lap = 0U;
    g_task11_log_phase = 0U;
    g_task11_segment_accum.sample_count = 0U;
}

static void task11_ram_log_set_context(uint8_t lap, uint8_t phase)
{
    g_task11_log_lap = lap;
    g_task11_log_phase = phase;
}

static void task11_ram_log_segment_reset(uint8_t lap,
    uint8_t phase,
    uint32_t start_ms,
    int32_t yaw_start_cdeg)
{
    g_task11_segment_accum.start_ms = start_ms;
    g_task11_segment_accum.sample_count = 0U;
    g_task11_segment_accum.lost_count = 0U;
    g_task11_segment_accum.yaw_start_cdeg = yaw_start_cdeg;
    g_task11_segment_accum.sum_abs_error = 0;
    g_task11_segment_accum.max_abs_error = 0;
    g_task11_segment_accum.sum_speed_diff = 0;
    g_task11_segment_accum.sum_pd_error = 0;
    g_task11_segment_accum.sum_correction = 0;
    g_task11_segment_accum.sum_ir_turn = 0;
    g_task11_segment_accum.sum_target_diff = 0;
    g_task11_segment_accum.lap = lap;
    g_task11_segment_accum.phase = phase;
    task11_ram_log_set_context(lap, phase);
}

static void task11_ram_log_segment_sample(uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    const straight_drive_output_t *drive,
    int32_t ir_turn,
    int32_t target_diff)
{
    int32_t abs_error = 0;

    g_task11_segment_accum.sample_count++;
    if ((ir_ok == 0U) || (sample->line_lost != 0U)) {
        g_task11_segment_accum.lost_count++;
    } else {
        abs_error = abs_i32(sample->error);
        g_task11_segment_accum.sum_abs_error += abs_error;
        if (abs_error > g_task11_segment_accum.max_abs_error) {
            g_task11_segment_accum.max_abs_error = abs_error;
        }
    }
    g_task11_segment_accum.sum_speed_diff += drive->speed_diff;
    g_task11_segment_accum.sum_pd_error += drive->pid_error;
    g_task11_segment_accum.sum_correction += drive->correction;
    g_task11_segment_accum.sum_ir_turn += ir_turn;
    g_task11_segment_accum.sum_target_diff += target_diff;
}

static void task11_ram_log_segment_finish(uint8_t reason,
    uint32_t end_ms,
    int32_t dist_count,
    int32_t yaw_end_cdeg,
    int32_t yaw_progress_cdeg,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample)
{
    task11_summary_log_t *log;
    int32_t sample_count = (int32_t)g_task11_segment_accum.sample_count;

    if (sample_count <= 0) {
        return;
    }
    if (g_task11_segment_accum.lap >= TASK11_RAM_LOG_MAX_LAPS) {
        return;
    }
    if (g_task11_summary_log_count >= TASK11_RAM_SUMMARY_CAPACITY) {
        g_task11_summary_log_overflow++;
        return;
    }

    log = &g_task11_summary_log[g_task11_summary_log_count++];
    log->start_ms = g_task11_segment_accum.start_ms;
    log->end_ms = end_ms;
    log->dist_count = task11_sat_u16(dist_count);
    log->sample_count = task11_sat_u16(sample_count);
    log->lost_count = task11_sat_u16((int32_t)g_task11_segment_accum.lost_count);
    log->max_abs_error = task11_sat_u16(g_task11_segment_accum.max_abs_error);
    log->yaw_start_cdeg = task11_sat_i16(g_task11_segment_accum.yaw_start_cdeg);
    log->yaw_end_cdeg = task11_sat_i16(yaw_end_cdeg);
    log->yaw_progress_cdeg = task11_sat_i16(yaw_progress_cdeg);
    log->avg_abs_error = task11_sat_i16(g_task11_segment_accum.sum_abs_error / sample_count);
    log->avg_speed_diff = task11_sat_i16(g_task11_segment_accum.sum_speed_diff / sample_count);
    log->avg_pd_error = task11_sat_i16(g_task11_segment_accum.sum_pd_error / sample_count);
    log->avg_correction = task11_sat_i16(g_task11_segment_accum.sum_correction / sample_count);
    log->avg_ir_turn = task11_sat_i16(g_task11_segment_accum.sum_ir_turn / sample_count);
    log->avg_target_diff = task11_sat_i16(g_task11_segment_accum.sum_target_diff / sample_count);
    log->lap = g_task11_segment_accum.lap;
    log->phase = g_task11_segment_accum.phase;
    log->reason = reason;
    log->point_mask = ((ir_ok != 0U) && (sample != 0)) ? sample->line_mask : 0U;
}

static uint8_t task11_ram_window_should_log(uint8_t lap,
    int32_t phase_distance_count,
    int32_t point_arm_count)
{
    int32_t window_start = point_arm_count - TASK11_RAM_WINDOW_BEFORE_COUNT;

    if (lap >= TASK11_RAM_LOG_MAX_LAPS) {
        return 0U;
    }
    if (window_start < 0) {
        window_start = 0;
    }
    return (phase_distance_count >= window_start) ? 1U : 0U;
}

static void task11_ram_log_window_sample(uint8_t lap,
    uint8_t phase,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    uint8_t edge_seen,
    uint32_t elapsed_ms,
    int32_t phase_distance_count,
    int32_t yaw_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t filtered_error,
    int32_t ir_turn,
    int32_t target_diff,
    const straight_drive_output_t *drive,
    int32_t left_pwm,
    int32_t right_pwm)
{
    task11_window_log_t *log;
    uint8_t line_lost = ((ir_ok == 0U) || (sample->line_lost != 0U)) ? 1U : 0U;

    if (g_task11_window_log_count >= TASK11_RAM_WINDOW_CAPACITY) {
        g_task11_window_log_overflow++;
        return;
    }

    log = &g_task11_window_log[g_task11_window_log_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = task11_sat_u16(phase_distance_count);
    log->yaw_cdeg = task11_sat_i16(yaw_cdeg);
    log->yaw_progress_cdeg = task11_sat_i16(yaw_progress_cdeg);
    log->line_error = task11_sat_i16((ir_ok != 0U) ? sample->error : 0);
    log->filtered_error = task11_sat_i16(filtered_error);
    log->ir_turn = task11_sat_i16(ir_turn);
    log->target_diff = task11_sat_i16(target_diff);
    log->speed_diff = task11_sat_i16(drive->speed_diff);
    log->pd_error = task11_sat_i16(drive->pid_error);
    log->correction = task11_sat_i16(drive->correction);
    log->pwm_b = task11_sat_i16(left_pwm);
    log->pwm_a = task11_sat_i16(right_pwm);
    log->lap = lap;
    log->phase = phase;
    log->raw = (ir_ok != 0U) ? sample->raw : 0xFFU;
    log->line_mask = (ir_ok != 0U) ? sample->line_mask : 0U;
    log->active_count = (ir_ok != 0U) ? sample->active_count : 0U;
    log->flags = (uint8_t)((ir_ok != 0U) ? 0x01U : 0U);
    log->flags |= (uint8_t)((line_lost != 0U) ? 0x02U : 0U);
    log->flags |= (uint8_t)((edge_seen != 0U) ? 0x04U : 0U);
}

static void task11_ram_log_event(uint8_t event,
    uint8_t reason,
    uint8_t lap,
    uint8_t phase,
    uint32_t elapsed_ms,
    int32_t distance_count,
    int32_t yaw_cdeg,
    int32_t yaw_progress_cdeg,
    int32_t gzlp_mdps,
    uint8_t ir_ok,
    const ir_tracking_sample_t *sample,
    int32_t motor_b_total,
    int32_t motor_a_total)
{
    task11_event_log_t *log;
    uint8_t line_lost = ((ir_ok == 0U) || ((sample != 0) && (sample->line_lost != 0U))) ? 1U : 0U;

    if ((lap >= TASK11_RAM_LOG_MAX_LAPS) && (event != TASK11_RAM_EVENT_COMPLETE)) {
        return;
    }
    if (g_task11_event_log_count >= TASK11_RAM_EVENT_CAPACITY) {
        g_task11_event_log_overflow++;
        return;
    }

    log = &g_task11_event_log[g_task11_event_log_count++];
    log->t_ms = elapsed_ms;
    log->dist_count = task11_sat_u16(distance_count);
    log->yaw_cdeg = task11_sat_i16(yaw_cdeg);
    log->yaw_progress_cdeg = task11_sat_i16(yaw_progress_cdeg);
    log->gzlp_x100_mdps = task11_sat_i16(gzlp_mdps / 100);
    log->line_error = task11_sat_i16(((ir_ok != 0U) && (sample != 0)) ? sample->error : 0);
    log->motor_b_total = task11_sat_i16(motor_b_total);
    log->motor_a_total = task11_sat_i16(motor_a_total);
    log->lap = lap;
    log->phase = phase;
    log->event = event;
    log->reason = reason;
    log->raw = ((ir_ok != 0U) && (sample != 0)) ? sample->raw : 0xFFU;
    log->line_mask = ((ir_ok != 0U) && (sample != 0)) ? sample->line_mask : 0U;
    log->active_count = ((ir_ok != 0U) && (sample != 0)) ? sample->active_count : 0U;
    log->flags = (uint8_t)((ir_ok != 0U) ? 0x01U : 0U);
    log->flags |= (uint8_t)((line_lost != 0U) ? 0x02U : 0U);
}

static void task11_ram_log_dump(void)
{
    uint16_t i;

    lc_printf("TASK11_RAM_BEGIN win=%u/%u win_ov=%u ev=%u/%u ev_ov=%u sum=%u/%u sum_ov=%u max_laps=%u\r\n",
        g_task11_window_log_count,
        TASK11_RAM_WINDOW_CAPACITY,
        g_task11_window_log_overflow,
        g_task11_event_log_count,
        TASK11_RAM_EVENT_CAPACITY,
        g_task11_event_log_overflow,
        g_task11_summary_log_count,
        TASK11_RAM_SUMMARY_CAPACITY,
        g_task11_summary_log_overflow,
        TASK11_RAM_LOG_MAX_LAPS);

    for (i = 0U; i < g_task11_summary_log_count; i++) {
        const task11_summary_log_t *log = &g_task11_summary_log[i];
        lc_printf("TASK11_SUM lap=%u seg=%s phase=%u reason=%s t=%lu/%lu dist=%u n=%u lost=%u yaw=%d/%d yprog=%d avg_abs_err=%d max_err=%u avg_diff=%d avg_pd=%d avg_corr=%d avg_ir=%d avg_tdiff=%d pmask=0x%02X\r\n",
            log->lap,
            task11_phase_name(log->phase),
            log->phase,
            task11_reason_name(log->reason),
            log->start_ms,
            log->end_ms,
            log->dist_count,
            log->sample_count,
            log->lost_count,
            log->yaw_start_cdeg,
            log->yaw_end_cdeg,
            log->yaw_progress_cdeg,
            log->avg_abs_error,
            log->max_abs_error,
            log->avg_speed_diff,
            log->avg_pd_error,
            log->avg_correction,
            log->avg_ir_turn,
            log->avg_target_diff,
            log->point_mask);
    }

    for (i = 0U; i < g_task11_event_log_count; i++) {
        const task11_event_log_t *log = &g_task11_event_log[i];
        lc_printf("TASK11_EVT lap=%u seg=%s phase=%u event=%s reason=%s t=%lu dist=%u yaw=%d yprog=%d gz100=%d raw=0x%02X mask=0x%02X cnt=%u flags=0x%02X err=%d B=%d A=%d\r\n",
            log->lap,
            task11_phase_name(log->phase),
            log->phase,
            task11_ram_event_name(log->event),
            task11_reason_name(log->reason),
            log->t_ms,
            log->dist_count,
            log->yaw_cdeg,
            log->yaw_progress_cdeg,
            log->gzlp_x100_mdps,
            log->raw,
            log->line_mask,
            log->active_count,
            log->flags,
            log->line_error,
            log->motor_b_total,
            log->motor_a_total);
    }

    for (i = 0U; i < g_task11_window_log_count; i++) {
        const task11_window_log_t *log = &g_task11_window_log[i];
        lc_printf("TASK11_WIN lap=%u seg=%s phase=%u t=%lu dist=%u yaw=%d yprog=%d raw=0x%02X mask=0x%02X cnt=%u flags=0x%02X err=%d filt=%d ir=%d tdiff=%d diff=%d pd=%d corr=%d pwm=%d/%d\r\n",
            log->lap,
            task11_phase_name(log->phase),
            log->phase,
            log->t_ms,
            log->dist_count,
            log->yaw_cdeg,
            log->yaw_progress_cdeg,
            log->raw,
            log->line_mask,
            log->active_count,
            log->flags,
            log->line_error,
            log->filtered_error,
            log->ir_turn,
            log->target_diff,
            log->speed_diff,
            log->pd_error,
            log->correction,
            log->pwm_b,
            log->pwm_a);
    }

    lc_printf("TASK11_RAM_END\r\n");
}
#else
#define task11_ram_log_reset() ((void)0)
#define task11_ram_log_set_context(lap, phase) ((void)0)
#define task11_ram_log_segment_reset(lap, phase, start_ms, yaw_start_cdeg) ((void)0)
#define task11_ram_log_segment_sample(ir_ok, sample, drive, ir_turn, target_diff) ((void)0)
#define task11_ram_log_segment_finish(reason, end_ms, dist_count, yaw_end_cdeg, yaw_progress_cdeg, ir_ok, sample) ((void)0)
#define task11_ram_window_should_log(lap, phase_distance_count, point_arm_count) (0U)
#define task11_ram_log_window_sample(lap, phase, ir_ok, sample, edge_seen, elapsed_ms, phase_distance_count, yaw_cdeg, yaw_progress_cdeg, filtered_error, ir_turn, target_diff, drive, left_pwm, right_pwm) ((void)0)
#define task11_ram_log_event(event, reason, lap, phase, elapsed_ms, distance_count, yaw_cdeg, yaw_progress_cdeg, gzlp_mdps, ir_ok, sample, motor_b_total, motor_a_total) ((void)0)
#define task11_ram_log_dump() ((void)0)
#endif

static uint8_t task11_peek_yaw(int32_t *yaw_cdeg, int32_t *gzlp_mdps)
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

static void task11_print_point(const char *name, const task11_line_result_t *result)
{
    task11_log_printf("TASK11_POINT %s reason=%s t=%lu dist=%ld yaw=%ld mask=0x%02X err=%ld\r\n",
        name,
        task11_reason_name(result->reason),
        result->elapsed_ms,
        result->distance_count,
        result->yaw_cdeg,
        (result->ir_ok != 0U) ? result->sample.line_mask : 0U,
        (result->ir_ok != 0U) ? result->sample.error : 0);
}

static void task11_diff_pid_reset(straight_pid_t *pid)
{
    straight_pid_reset(pid);
    pid->kp = TASK11_DIFF_KP;
    pid->ki = 0;
    pid->kd = TASK11_DIFF_KD;
    pid->i_limit = 0;
    pid->corr_max = TASK11_DIFF_CORR_MAX;
    pid->integral = 0;
    pid->last_error = 0;
}

static void task11_drive_config(straight_drive_config_t *config,
    int32_t base_pwm,
    int32_t target_speed_diff)
{
    config->base_b_pwm = base_pwm;
    config->base_a_pwm = base_pwm;
    config->target_speed_diff = target_speed_diff;
    config->diff_ff_gain = TASK11_DIFF_FF_GAIN;
    config->distance_corr_divisor = 1;
    config->distance_corr_max = 0;
    config->correction_max = TASK11_DIFF_CORR_MAX;
    config->min_pwm = TASK11_LINE_MIN_PWM;
    config->max_pwm = TASK11_LINE_MAX_PWM;
}

static uint8_t task11_ir_mask_seen(const ir_tracking_sample_t *sample,
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

static uint8_t task11_left_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_right_clear)
{
    return task11_ir_mask_seen(sample,
        TASK11_IR_LEFT_EDGE_MASK,
        (require_right_clear != 0U) ? TASK11_IR_RIGHT_EDGE_MASK : 0U);
}

static uint8_t task11_right_edge_seen(const ir_tracking_sample_t *sample,
    uint8_t require_left_clear)
{
    return task11_ir_mask_seen(sample,
        TASK11_IR_RIGHT_EDGE_MASK,
        (require_left_clear != 0U) ? TASK11_IR_LEFT_EDGE_MASK : 0U);
}

static uint8_t task11_sensor_fast_turn(const char *tag,
    int16_t motor_b_pwm,
    int16_t motor_a_pwm,
    int16_t slow_motor_b_pwm,
    int16_t slow_motor_a_pwm,
    uint8_t stop_mask,
    uint8_t forbid_mask,
    int32_t stop_error_max)
{
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t line_stop_ready = 0U;
    uint8_t line_seen = 0U;
    uint8_t slow_mode = 0U;
    uint8_t center_ready = 0U;
    uint8_t wide_ready = 0U;
    uint8_t err_ready = 0U;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential(motor_b_pwm, motor_a_pwm);
    task11_ram_log_event(TASK11_RAM_EVENT_TURN_START,
        0U,
        g_task11_log_lap,
        g_task11_log_phase,
        0U,
        0,
        0,
        0,
        0,
        0U,
        &sample,
        0,
        0);
    task11_log_printf("%s start: sensor_fast_turn pwm=%d/%d slow=%d/%d stop_mask=0x%02X forbid=0x%02X err_max=%ld\r\n",
        tag,
        motor_b_pwm,
        motor_a_pwm,
        slow_motor_b_pwm,
        slow_motor_a_pwm,
        stop_mask,
        forbid_mask,
        stop_error_max);

    while (elapsed_ms < TASK11_FAST_TURN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        ir_ok = IRTracking_ReadSample(&sample);
        nav_ok = JY62_PeekNavigation(&nav);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        line_seen = ((ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (sample.active_count >= TASK11_IR_TURN_STOP_MIN_COUNT)) ? 1U : 0U;
        if ((line_seen != 0U) && (slow_mode == 0U)) {
            slow_mode = 1U;
            TB6612_SetDifferential(slow_motor_b_pwm, slow_motor_a_pwm);
        }
        center_ready = ((line_seen != 0U) &&
            ((sample.line_mask & stop_mask) != 0U) &&
            ((sample.line_mask & forbid_mask) == 0U)) ? 1U : 0U;
        wide_ready = ((line_seen != 0U) && (sample.line_mask == 0xFFU)) ? 1U : 0U;
        err_ready = ((line_seen != 0U) &&
            (abs_i32(sample.error) <= stop_error_max)) ? 1U : 0U;
        line_stop_ready = ((center_ready != 0U) || (wide_ready != 0U)) ? 1U : 0U;

        if (line_stop_ready != 0U) {
            stop_reason = (wide_ready != 0U) ? 4U : 1U;
            break;
        }

        if (report_elapsed_ms >= TASK11_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            task11_log_printf("%s t=%lu nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u seen=%u center=%u wide=%u err_ok=%u ready=%u\r\n",
                tag,
                elapsed_ms,
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
                motor_a_total,
                slow_mode,
                line_seen,
                center_ready,
                wide_ready,
                err_ready,
                line_stop_ready);
        }
    }

    if (stop_reason == 0U) {
        stop_reason = 2U;
    }

    TB6612_Brake();
    ir_ok = IRTracking_ReadSample(&sample);
    nav_ok = JY62_PeekNavigation(&nav);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    task11_ram_log_event(TASK11_RAM_EVENT_TURN_STOP,
        stop_reason,
        g_task11_log_lap,
        g_task11_log_phase,
        elapsed_ms,
        (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
    task11_log_printf("%s stop: reason=%s t=%lu nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld slow=%u\r\n",
        tag,
        (stop_reason == 1U) ? "line" : ((stop_reason == 4U) ? "wide" : ((stop_reason == 2U) ? "timeout" : "uart_stop")),
        elapsed_ms,
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
        motor_a_total,
        slow_mode);

    encoder_reset_distance_counts();
    return ((stop_reason == 1U) || (stop_reason == 4U)) ? 1U : 0U;
}

static uint8_t task11_advance_after_point(const char *tag, int32_t advance_count)
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
        task11_ram_log_event(TASK11_RAM_EVENT_ADVANCE_STOP,
            1U,
            g_task11_log_lap,
            g_task11_log_phase,
            0U,
            0,
            0,
            0,
            0,
            0U,
            &sample,
            0,
            0);
        task11_log_printf("%s skip: advance_count=%ld\r\n", tag, advance_count);
        return 1U;
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential((int16_t)TASK11_POINT_ADVANCE_PWM,
        (int16_t)TASK11_POINT_ADVANCE_PWM);
    task11_ram_log_event(TASK11_RAM_EVENT_ADVANCE_START,
        0U,
        g_task11_log_lap,
        g_task11_log_phase,
        0U,
        0,
        0,
        0,
        0,
        0U,
        &sample,
        0,
        0);
    task11_log_printf("%s start: advance_count=%ld pwm=%d\r\n",
        tag,
        advance_count,
        TASK11_POINT_ADVANCE_PWM);

    while (elapsed_ms < TASK11_POINT_ADVANCE_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        nav_ok = JY62_PeekNavigation(&nav);
        ir_ok = IRTracking_ReadSample(&sample);

        if (distance_count >= advance_count) {
            stop_reason = 1U;
            break;
        }

        if (report_elapsed_ms >= TASK11_FAST_TURN_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            task11_log_printf("%s t=%lu dist=%ld nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld\r\n",
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
    distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
    nav_ok = JY62_PeekNavigation(&nav);
    ir_ok = IRTracking_ReadSample(&sample);
    task11_ram_log_event(TASK11_RAM_EVENT_ADVANCE_STOP,
        stop_reason,
        g_task11_log_lap,
        g_task11_log_phase,
        elapsed_ms,
        distance_count,
        (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
        0,
        (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
        ir_ok,
        &sample,
        motor_b_total,
        motor_a_total);
    task11_log_printf("%s stop: reason=%s t=%lu dist=%ld nav=%u yaw=%ld gzlp=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld B=%ld A=%ld\r\n",
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

static uint8_t task11_align_to_yaw(const char *tag, int32_t target_cdeg)
{
    uint32_t elapsed_ms = 0;
    uint8_t stable_count = 0U;
    uint8_t stop_reason = 0U;
    int32_t yaw_cdeg = 0;
    int32_t gzlp_mdps = 0;
    int32_t error_cdeg = 0;
    uint8_t nav_ok;

    TB6612_Brake();
    delay_ms_with_st011(TASK11_POINT_SETTLE_MS);
    nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
    task11_log_printf("%s start: yaw=%ld target=%ld nav=%u\r\n",
        tag,
        yaw_cdeg,
        target_cdeg,
        nav_ok);

    while (elapsed_ms < TASK11_ALIGN_TIMEOUT_MS) {
        int32_t turn_pwm;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
        if (nav_ok == 0U) {
            stop_reason = 5U;
            break;
        }

        error_cdeg = normalize_cdeg(target_cdeg - yaw_cdeg);
        if (abs_i32(error_cdeg) <= TASK11_ALIGN_TOL_CDEG) {
            TB6612_Brake();
            if (abs_i32(gzlp_mdps) > TASK11_ALIGN_GZLP_TOL_MDPS) {
                stable_count = 0U;
                continue;
            }
            stable_count++;
            if (stable_count >= TASK11_ALIGN_STABLE_COUNT) {
                stop_reason = 1U;
                break;
            }
            continue;
        }

        stable_count = 0U;
        turn_pwm = (abs_i32(error_cdeg) <= TASK11_ALIGN_SLOW_ZONE_CDEG) ?
            TASK11_ALIGN_SLOW_PWM : TASK11_ALIGN_FAST_PWM;
        if (error_cdeg > 0) {
            TB6612_SetDifferential((int16_t)-turn_pwm, (int16_t)turn_pwm);
        } else {
            TB6612_SetDifferential((int16_t)turn_pwm, (int16_t)-turn_pwm);
        }
    }

    TB6612_Brake();
    if (stop_reason == 0U) {
        stop_reason = 2U;
    }
    (void)task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
    error_cdeg = normalize_cdeg(target_cdeg - yaw_cdeg);
    task11_log_printf("%s stop: reason=%s t=%lu yaw=%ld target=%ld err=%ld stable=%u gzlp=%ld\r\n",
        tag,
        task11_reason_name(stop_reason),
        elapsed_ms,
        yaw_cdeg,
        target_cdeg,
        error_cdeg,
        stable_count,
        gzlp_mdps);

    return (stop_reason == 1U) ? 1U : 0U;
}

static uint8_t task11_align_relative(const char *tag, int32_t delta_cdeg)
{
    int32_t yaw_cdeg;
    int32_t gzlp_mdps;
    uint8_t nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);

    if (nav_ok == 0U) {
        TB6612_Brake();
        task11_log_printf("%s abort: nav=0 delta=%ld\r\n", tag, delta_cdeg);
        return 0U;
    }

    return task11_align_to_yaw(tag, normalize_cdeg(yaw_cdeg + delta_cdeg));
}

static uint8_t task11_run_ir_line_segment(const char *tag,
    uint8_t arc_mode,
    int32_t expected_turn_dir,
    int32_t point_arm_count,
    int32_t force_count,
    int32_t point_yaw_arm_cdeg,
    task11_line_result_t *result)
{
    ir_tracking_sample_t sample = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_total;
    int32_t motor_a_total;
    int32_t filtered_error = 0;
    int32_t last_filtered_error = 0;
    int32_t last_turn = 0;
    int32_t yaw_start = 0;
    int32_t yaw_cdeg = 0;
    int32_t gzlp_mdps = 0;
    int32_t yaw_progress_cdeg = 0;
    uint8_t nav_ok;
    uint8_t exit_line_seen = 0U;
    uint8_t straight_point_confirm = 0U;
    uint8_t stop_reason = 0U;
    uint8_t ir_ok = 0U;

    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    nav_ok = task11_peek_yaw(&yaw_start, &gzlp_mdps);
    task11_log_printf("%s start: mode=%s arm=%ld force=%ld yaw_arm=%ld yaw0=%ld nav=%u base=%d\r\n",
        tag,
        (arc_mode != 0U) ? "arc" : "straight",
        point_arm_count,
        force_count,
        point_yaw_arm_cdeg,
        yaw_start,
        nav_ok,
        TASK11_LINE_BASE_PWM);

    while (elapsed_ms < TASK1_MAX_RUN_MS) {
        int32_t distance_count;
        int32_t raw_error = 0;
        int32_t derivative = 0;
        int32_t turn = 0;
        int32_t base_pwm = TASK11_LINE_BASE_PWM;
        int32_t left_pwm;
        int32_t right_pwm;
        uint8_t line_valid;
        uint8_t straight_point_candidate;
        uint8_t straight_point_ready;
        uint8_t arc_point_ready;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
        yaw_progress_cdeg = (nav_ok != 0U) ? abs_i32(normalize_cdeg(yaw_cdeg - yaw_start)) : 0;
        ir_ok = IRTracking_ReadSample(&sample);
        line_valid = ((ir_ok != 0U) && (sample.line_lost == 0U)) ? 1U : 0U;

        if ((distance_count >= point_arm_count) && (line_valid != 0U)) {
            exit_line_seen = 1U;
        }

        straight_point_candidate = ((arc_mode == 0U) &&
            (distance_count >= point_arm_count) &&
            (line_valid != 0U) &&
            ((abs_i32(sample.error) >= TASK11_STRAIGHT_POINT_ERROR_MIN) ||
             (sample.active_count >= TASK11_STRAIGHT_POINT_WIDE_COUNT))) ? 1U : 0U;
        if (straight_point_candidate != 0U) {
            if (straight_point_confirm < TASK11_STRAIGHT_POINT_CONFIRM_COUNT) {
                straight_point_confirm++;
            }
        } else {
            straight_point_confirm = 0U;
        }
        straight_point_ready = (straight_point_confirm >= TASK11_STRAIGHT_POINT_CONFIRM_COUNT) ? 1U : 0U;
        arc_point_ready = ((arc_mode != 0U) &&
            (distance_count >= point_arm_count) &&
            (yaw_progress_cdeg >= point_yaw_arm_cdeg) &&
            (exit_line_seen != 0U) &&
            ((line_valid == 0U) ||
             (sample.active_count >= TASK11_ARC_POINT_WIDE_COUNT))) ? 1U : 0U;

        if ((straight_point_ready != 0U) || (arc_point_ready != 0U)) {
            stop_reason = 1U;
            break;
        }

        if (distance_count >= force_count) {
            stop_reason = 2U;
            break;
        }

        if (line_valid != 0U) {
            raw_error = sample.error;
            filtered_error += (raw_error - filtered_error) / TASK11_LINE_ERROR_FILTER_DIVISOR;
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -TASK11_LINE_DERIV_LIMIT,
                TASK11_LINE_DERIV_LIMIT);
            last_filtered_error = filtered_error;
            turn = (filtered_error / TASK11_LINE_TURN_DIVISOR) +
                (derivative / TASK11_LINE_KD_DIVISOR);
            turn = clamp_i32(turn, -TASK11_LINE_TURN_LIMIT, TASK11_LINE_TURN_LIMIT);
            last_turn = turn;
        } else {
            base_pwm -= TASK11_LINE_LOST_BASE_DROP;
            if (last_turn != 0) {
                turn = clamp_i32(last_turn, -TASK11_LINE_LOST_TURN, TASK11_LINE_LOST_TURN);
            } else {
                turn = expected_turn_dir * TASK11_LINE_LOST_TURN;
            }
        }

        left_pwm = clamp_i32(base_pwm + turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        right_pwm = clamp_i32(base_pwm - turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (report_elapsed_ms >= TASK11_LINE_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            task11_log_printf("%s t=%lu dist=%ld yaw=%ld yprog=%ld arm=%u seen=%u ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld filt=%ld turn=%ld L=%ld R=%ld\r\n",
                tag,
                elapsed_ms,
                distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                (distance_count >= point_arm_count) ? 1U : 0U,
                exit_line_seen,
                ir_ok,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                filtered_error,
                turn,
                left_pwm,
                right_pwm);
        }
    }

    TB6612_Brake();
    if (stop_reason == 0U) {
        stop_reason = 4U;
    }
    (void)task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    result->reason = stop_reason;
    result->elapsed_ms = elapsed_ms;
    result->distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
    result->yaw_cdeg = yaw_cdeg;
    result->yaw_progress_cdeg = yaw_progress_cdeg;
    result->ir_ok = ir_ok;
    result->sample = sample;

    task11_log_printf("%s stop: reason=%s t=%lu dist=%ld yaw=%ld yprog=%ld ir=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld\r\n",
        tag,
        task11_reason_name(stop_reason),
        elapsed_ms,
        result->distance_count,
        yaw_cdeg,
        yaw_progress_cdeg,
        ir_ok,
        (ir_ok != 0U) ? sample.raw : 0xFFU,
        (ir_ok != 0U) ? sample.line_mask : 0U,
        (ir_ok != 0U) ? sample.active_count : 0U,
        (ir_ok != 0U) ? sample.line_lost : 1U,
        (ir_ok != 0U) ? sample.error : 0);

    return stop_reason;
}

static void run_task10_ab_zero_test(void)
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

static void run_task11_ir_map_test(void)
{
    task11_line_result_t result;
    ir_tracking_sample_t sample = {0};
    straight_pid_t diff_pid;
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
    uint8_t straight_point_confirm = 0U;
    uint8_t ir_ok = 0U;
    uint8_t stop_reason = 0U;

    TB6612_Brake();
    delay_ms_with_st011(TASK11_POINT_SETTLE_MS);
    IRTracking_Init();
    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    task11_diff_pid_reset(&diff_pid);
    nav_ok = task11_peek_yaw(&yaw_start, &gzlp_mdps);
    yaw_cdeg = yaw_start;
    task11_ram_log_reset();
    task11_ram_log_segment_reset(lap_count, phase, elapsed_ms, yaw_start);
    task11_ram_log_event(TASK11_RAM_EVENT_START,
        0U,
        lap_count,
        phase,
        elapsed_ms,
        0,
        yaw_cdeg,
        0,
        gzlp_mdps,
        0U,
        &sample,
        0,
        0);
    task11_log_printf("TASK11 start: continuous_ir laps=%u yaw=%ld nav=%u base=%d arc_base=%d cb_diff=%d/%d da_diff=%d/%d ff_gain=%d kp=%d kd=%d report=%d\r\n",
        TASK11_TARGET_LAPS,
        yaw_cdeg,
        nav_ok,
        TASK11_LINE_BASE_PWM,
        TASK11_ARC_BASE_PWM,
        TASK11_CB_ARC_ENTRY_TARGET_DIFF,
        TASK11_CB_ARC_CRUISE_TARGET_DIFF,
        TASK11_DA_ARC_ENTRY_TARGET_DIFF,
        TASK11_DA_ARC_CRUISE_TARGET_DIFF,
        TASK11_DIFF_FF_GAIN,
        TASK11_DIFF_KP,
        TASK11_DIFF_KD,
        TASK11_LINE_REPORT_PERIOD_MS);

    while ((elapsed_ms < TASK11_TOTAL_MAX_RUN_MS) && (lap_count < TASK11_TARGET_LAPS)) {
        int32_t total_distance_count;
        int32_t phase_distance_count;
        int32_t raw_error = 0;
        int32_t derivative = 0;
        int32_t base_pwm = TASK11_LINE_BASE_PWM;
        int32_t left_pwm;
        int32_t right_pwm;
        int32_t point_arm_count;
        int32_t force_count;
        int32_t expected_turn_dir;
        int32_t target_speed_diff;
        int32_t ir_turn = 0;
        const char *phase_name;
        const char *point_name;
        const char *force_name;
        uint8_t arc_mode;
        uint8_t line_valid;
        uint8_t straight_point_candidate;
        uint8_t edge_point_seen;
        uint8_t quick_turn_ok = 1U;
        uint8_t point_ready = 0U;

        if (phase == 0U) {
            phase_name = "AC";
            point_name = "C_LINE";
            force_name = "C_FORCE";
            arc_mode = 0U;
            expected_turn_dir = 0;
            point_arm_count = TASK11_AC_POINT_ARM_COUNT;
            force_count = TASK11_STRAIGHT_FORCE_COUNT;
        } else if (phase == 1U) {
            phase_name = "CB";
            point_name = "B_EXIT";
            force_name = "B_FORCE";
            arc_mode = 1U;
            expected_turn_dir = TASK3_ARC_TURN_LEFT;
            point_arm_count = TASK11_ARC_POINT_ARM_COUNT;
            force_count = TASK11_ARC_FORCE_COUNT;
        } else if (phase == 2U) {
            phase_name = "BD";
            point_name = "D_LINE";
            force_name = "D_FORCE";
            arc_mode = 0U;
            expected_turn_dir = 0;
            point_arm_count = TASK11_BD_POINT_ARM_COUNT;
            force_count = TASK11_STRAIGHT_FORCE_COUNT;
        } else {
            phase_name = "DA";
            point_name = "A_FINISH";
            force_name = "A_FORCE";
            arc_mode = 1U;
            expected_turn_dir = TASK3_ARC_TURN_RIGHT;
            point_arm_count = TASK11_ARC_POINT_ARM_COUNT;
            force_count = TASK11_ARC_FORCE_COUNT;
        }

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        total_distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        phase_distance_count = total_distance_count - phase_start_count;
        nav_ok = task11_peek_yaw(&yaw_cdeg, &gzlp_mdps);
        yaw_progress_cdeg = (nav_ok != 0U) ? abs_i32(normalize_cdeg(yaw_cdeg - yaw_start)) : 0;
        ir_ok = IRTracking_ReadSample(&sample);
        line_valid = ((ir_ok != 0U) && (sample.line_lost == 0U)) ? 1U : 0U;
        if (phase == 0U) {
            edge_point_seen = ((ir_ok != 0U) &&
                (task11_left_edge_seen(&sample, 1U) != 0U)) ? 1U : 0U;
        } else if (phase == 1U) {
            edge_point_seen = ((ir_ok != 0U) &&
                (task11_left_edge_seen(&sample, 0U) != 0U)) ? 1U : 0U;
        } else if (phase == 2U) {
            edge_point_seen = ((ir_ok != 0U) &&
                (task11_right_edge_seen(&sample, 0U) != 0U)) ? 1U : 0U;
        } else {
            edge_point_seen = ((ir_ok != 0U) &&
                (task11_right_edge_seen(&sample, 0U) != 0U)) ? 1U : 0U;
        }
        if (arc_mode != 0U) {
            base_pwm = TASK11_ARC_BASE_PWM;
            if (phase == 1U) {
                target_speed_diff = (phase_distance_count < TASK11_ARC_ENTRY_COUNT) ?
                    TASK11_CB_ARC_ENTRY_TARGET_DIFF :
                    TASK11_CB_ARC_CRUISE_TARGET_DIFF;
            } else {
                target_speed_diff = (phase_distance_count < TASK11_ARC_ENTRY_COUNT) ?
                    TASK11_DA_ARC_ENTRY_TARGET_DIFF :
                    TASK11_DA_ARC_CRUISE_TARGET_DIFF;
            }
        } else {
            base_pwm = TASK11_STRAIGHT_BASE_PWM;
            target_speed_diff = TASK11_STRAIGHT_TARGET_DIFF;
        }
        if (line_valid == 0U) {
            base_pwm -= TASK11_LINE_LOST_BASE_DROP;
        }
        task11_drive_config(&drive_config, base_pwm, target_speed_diff);
        straight_drive_update(&diff_pid,
            &drive_config,
            motor_b_delta,
            motor_a_delta,
            motor_b_total,
            motor_a_total,
            &drive);
        report_b_speed_sum += drive.motor_b_speed;
        report_a_speed_sum += drive.motor_a_speed;
        report_sample_count++;
        if (line_valid != 0U) {
            raw_error = sample.error;
            filtered_error += (raw_error - filtered_error) / TASK11_LINE_ERROR_FILTER_DIVISOR;
            derivative = clamp_i32(filtered_error - last_filtered_error,
                -TASK11_LINE_DERIV_LIMIT,
                TASK11_LINE_DERIV_LIMIT);
            last_filtered_error = filtered_error;
            ir_turn = (filtered_error / TASK11_LINE_TURN_DIVISOR) +
                (derivative / TASK11_LINE_KD_DIVISOR);
            ir_turn = clamp_i32(ir_turn,
                -TASK11_LINE_TURN_LIMIT,
                TASK11_LINE_TURN_LIMIT);
            last_turn = ir_turn;
        } else {
            if (last_turn != 0) {
                ir_turn = clamp_i32(last_turn,
                    -TASK11_LINE_LOST_TURN,
                    TASK11_LINE_LOST_TURN);
            } else {
                ir_turn = expected_turn_dir * TASK11_LINE_LOST_TURN;
            }
        }

        if (arc_mode == 0U) {
            straight_point_candidate = ((phase_distance_count >= point_arm_count) &&
                (edge_point_seen != 0U)) ? 1U : 0U;
            if (straight_point_candidate != 0U) {
                if (straight_point_confirm < TASK11_STRAIGHT_POINT_CONFIRM_COUNT) {
                    straight_point_confirm++;
                }
            } else {
                straight_point_confirm = 0U;
            }
            point_ready = (straight_point_confirm >= TASK11_STRAIGHT_POINT_CONFIRM_COUNT) ? 1U : 0U;
        } else {
            point_ready = ((phase_distance_count >= point_arm_count) &&
                (edge_point_seen != 0U)) ? 1U : 0U;
        }

        left_pwm = clamp_i32(drive.motor_b_pwm + ir_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        right_pwm = clamp_i32(drive.motor_a_pwm - ir_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        task11_ram_log_segment_sample(ir_ok,
            &sample,
            &drive,
            ir_turn,
            drive_config.target_speed_diff);
        if (task11_ram_window_should_log(lap_count,
            phase_distance_count,
            point_arm_count) != 0U) {
            task11_ram_log_window_sample(lap_count,
                phase,
                ir_ok,
                &sample,
                edge_point_seen,
                elapsed_ms,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                filtered_error,
                ir_turn,
                drive_config.target_speed_diff,
                &drive,
                left_pwm,
                right_pwm);
        }

        if (point_ready != 0U) {
            result.reason = 1U;
            result.elapsed_ms = elapsed_ms;
            result.distance_count = phase_distance_count;
            result.yaw_cdeg = yaw_cdeg;
            result.yaw_progress_cdeg = yaw_progress_cdeg;
            result.ir_ok = ir_ok;
            result.sample = sample;
            task11_ram_log_segment_finish(result.reason,
                elapsed_ms,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                ir_ok,
                &sample);
            task11_ram_log_event(TASK11_RAM_EVENT_POINT,
                result.reason,
                lap_count,
                phase,
                elapsed_ms,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                gzlp_mdps,
                ir_ok,
                &sample,
                motor_b_total,
                motor_a_total);
            task11_event_printf("TASK11_EVT_RT lap=%u seg=%s event=point t=%lu dist=%ld mask=0x%02X\r\n",
                lap_count,
                phase_name,
                elapsed_ms,
                phase_distance_count,
                (ir_ok != 0U) ? sample.line_mask : 0U);
            task11_print_point(point_name, &result);
            st011_start_pulse(TASK11_POINT_ALARM_MS);
            task11_log_printf("TASK11_POINT_DRIVE seg=%s edge=%u target_diff=%ld B_spd=%ld A_spd=%ld diff=%ld err=%ld P=%ld D=%ld ff=%ld fb=%ld corr=%ld ir_turn=%ld drive_pwm=%ld/%ld final_pwm=%ld/%ld\r\n",
                phase_name,
                edge_point_seen,
                drive_config.target_speed_diff,
                drive.motor_b_speed,
                drive.motor_a_speed,
                drive.speed_diff,
                drive.pid_error,
                drive.p_term,
                drive.d_term,
                drive.feedforward_correction,
                drive.feedback_correction,
                drive.correction,
                ir_turn,
                drive.motor_b_pwm,
                drive.motor_a_pwm,
                clamp_i32(drive.motor_b_pwm + ir_turn,
                    TASK11_LINE_MIN_PWM,
                    TASK11_LINE_MAX_PWM),
                clamp_i32(drive.motor_a_pwm - ir_turn,
                    TASK11_LINE_MIN_PWM,
                    TASK11_LINE_MAX_PWM));

            if (phase == 0U) {
                quick_turn_ok = task11_advance_after_point("TASK11_C_ADVANCE",
                    TASK11_POINT_ADVANCE_COUNT);
                if (quick_turn_ok == 0U) {
                    stop_reason = 2U;
                    break;
                }
                quick_turn_ok = task11_sensor_fast_turn("TASK11_C_LEFT_TURN",
                    TASK11_LEFT_TURN_B_PWM,
                    TASK11_LEFT_TURN_A_PWM,
                    TASK11_LEFT_TURN_SLOW_B_PWM,
                    TASK11_LEFT_TURN_SLOW_A_PWM,
                    TASK11_IR_CENTER_6_MASK,
                    TASK11_IR_CENTER_6_FORBID_MASK,
                    TASK11_TURN_CENTER6_ERROR_MAX);
            } else if (phase == 1U) {
                quick_turn_ok = task11_advance_after_point("TASK11_B_ADVANCE",
                    TASK11_ARC_POINT_ADVANCE_COUNT);
                if (quick_turn_ok == 0U) {
                    stop_reason = 2U;
                    break;
                }
                quick_turn_ok = task11_sensor_fast_turn("TASK11_B_LEFT_TURN",
                    TASK11_LEFT_TURN_B_PWM,
                    TASK11_LEFT_TURN_A_PWM,
                    TASK11_LEFT_TURN_SLOW_B_PWM,
                    TASK11_LEFT_TURN_SLOW_A_PWM,
                    TASK11_IR_CENTER_4_MASK,
                    TASK11_IR_CENTER_4_FORBID_MASK,
                    TASK11_TURN_CENTER4_ERROR_MAX);
            } else if (phase == 2U) {
                quick_turn_ok = task11_advance_after_point("TASK11_D_ADVANCE",
                    TASK11_POINT_ADVANCE_COUNT);
                if (quick_turn_ok == 0U) {
                    stop_reason = 2U;
                    break;
                }
                quick_turn_ok = task11_sensor_fast_turn("TASK11_D_RIGHT_TURN",
                    TASK11_RIGHT_TURN_B_PWM,
                    TASK11_RIGHT_TURN_A_PWM,
                    TASK11_RIGHT_TURN_SLOW_B_PWM,
                    TASK11_RIGHT_TURN_SLOW_A_PWM,
                    TASK11_IR_CENTER_6_MASK,
                    TASK11_IR_CENTER_6_FORBID_MASK,
                    TASK11_TURN_CENTER6_ERROR_MAX);
            } else if ((uint8_t)(lap_count + 1U) < TASK11_TARGET_LAPS) {
                quick_turn_ok = task11_advance_after_point("TASK11_A_ADVANCE",
                    TASK11_ARC_POINT_ADVANCE_COUNT);
                if (quick_turn_ok == 0U) {
                    stop_reason = 2U;
                    break;
                }
                quick_turn_ok = task11_sensor_fast_turn("TASK11_A_RIGHT_TURN",
                    TASK11_RIGHT_TURN_B_PWM,
                    TASK11_RIGHT_TURN_A_PWM,
                    TASK11_RIGHT_TURN_SLOW_B_PWM,
                    TASK11_RIGHT_TURN_SLOW_A_PWM,
                    TASK11_IR_CENTER_4_MASK,
                    TASK11_IR_CENTER_4_FORBID_MASK,
                    TASK11_TURN_CENTER4_ERROR_MAX);
            }
            if (quick_turn_ok == 0U) {
                stop_reason = 2U;
                break;
            }

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
            phase_start_count = 0;
            (void)task11_peek_yaw(&yaw_start, &gzlp_mdps);
            straight_point_confirm = 0U;
            filtered_error = 0;
            last_filtered_error = 0;
            last_turn = 0;
            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_sample_count = 0;
            task11_diff_pid_reset(&diff_pid);
            task11_ram_log_segment_reset(lap_count, phase, elapsed_ms, yaw_start);
            continue;
        } else if (phase_distance_count >= force_count) {
            result.reason = 2U;
            result.elapsed_ms = elapsed_ms;
            result.distance_count = phase_distance_count;
            result.yaw_cdeg = yaw_cdeg;
            result.yaw_progress_cdeg = yaw_progress_cdeg;
            result.ir_ok = ir_ok;
            result.sample = sample;
            task11_ram_log_segment_finish(result.reason,
                elapsed_ms,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                ir_ok,
                &sample);
            task11_ram_log_event(TASK11_RAM_EVENT_FORCE,
                result.reason,
                lap_count,
                phase,
                elapsed_ms,
                phase_distance_count,
                yaw_cdeg,
                yaw_progress_cdeg,
                gzlp_mdps,
                ir_ok,
                &sample,
                motor_b_total,
                motor_a_total);
            task11_event_printf("TASK11_EVT_RT lap=%u seg=%s event=force t=%lu dist=%ld mask=0x%02X\r\n",
                lap_count,
                phase_name,
                elapsed_ms,
                phase_distance_count,
                (ir_ok != 0U) ? sample.line_mask : 0U);
            task11_print_point(force_name, &result);
            st011_start_pulse(TASK11_POINT_ALARM_MS);
            task11_log_printf("TASK11_POINT_DRIVE seg=%s edge=%u target_diff=%ld B_spd=%ld A_spd=%ld diff=%ld err=%ld P=%ld D=%ld ff=%ld fb=%ld corr=%ld ir_turn=%ld drive_pwm=%ld/%ld final_pwm=%ld/%ld\r\n",
                phase_name,
                edge_point_seen,
                drive_config.target_speed_diff,
                drive.motor_b_speed,
                drive.motor_a_speed,
                drive.speed_diff,
                drive.pid_error,
                drive.p_term,
                drive.d_term,
                drive.feedforward_correction,
                drive.feedback_correction,
                drive.correction,
                ir_turn,
                drive.motor_b_pwm,
                drive.motor_a_pwm,
                clamp_i32(drive.motor_b_pwm + ir_turn,
                    TASK11_LINE_MIN_PWM,
                    TASK11_LINE_MAX_PWM),
                clamp_i32(drive.motor_a_pwm - ir_turn,
                    TASK11_LINE_MIN_PWM,
                    TASK11_LINE_MAX_PWM));

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
            straight_point_confirm = 0U;
            filtered_error = 0;
            last_filtered_error = 0;
            last_turn = 0;
            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_sample_count = 0;
            task11_diff_pid_reset(&diff_pid);
            task11_ram_log_segment_reset(lap_count, phase, elapsed_ms, yaw_start);
            continue;
        }

        left_pwm = clamp_i32(drive.motor_b_pwm + ir_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        right_pwm = clamp_i32(drive.motor_a_pwm - ir_turn,
            TASK11_LINE_MIN_PWM,
            TASK11_LINE_MAX_PWM);
        TB6612_SetDifferential((int16_t)left_pwm, (int16_t)right_pwm);

        if (report_elapsed_ms >= TASK11_LINE_REPORT_PERIOD_MS) {
            int32_t b_speed_avg = (report_sample_count != 0U) ?
                (report_b_speed_sum / (int32_t)report_sample_count) : drive.motor_b_speed;
            int32_t a_speed_avg = (report_sample_count != 0U) ?
                (report_a_speed_sum / (int32_t)report_sample_count) : drive.motor_a_speed;
            int32_t diff_avg = b_speed_avg - a_speed_avg;

            report_elapsed_ms = 0;
            report_b_speed_sum = 0;
            report_a_speed_sum = 0;
            report_sample_count = 0;
            task11_log_printf("TASK11_DATA lap=%u seg=%s phase=%u t=%lu dist=%ld edge=%u yprog=%ld yaw=%ld nav=%u gzlp=%ld raw=0x%02X mask=0x%02X cnt=%u lost=%u err=%ld filt=%ld der=%ld ir_turn=%ld base=%ld target_diff=%ld B_cnt=%ld A_cnt=%ld B_total=%ld A_total=%ld B_spd=%ld A_spd=%ld diff=%ld B_avg=%ld A_avg=%ld diff_avg=%ld pd_err=%ld P=%ld D=%ld ff=%ld fb=%ld corr=%ld pwm=%ld/%ld\r\n",
                lap_count,
                phase_name,
                phase,
                elapsed_ms,
                phase_distance_count,
                edge_point_seen,
                yaw_progress_cdeg,
                yaw_cdeg,
                nav_ok,
                gzlp_mdps,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                (ir_ok != 0U) ? sample.error : 0,
                filtered_error,
                derivative,
                ir_turn,
                base_pwm,
                drive_config.target_speed_diff,
                motor_b_delta,
                motor_a_delta,
                motor_b_total,
                motor_a_total,
                drive.motor_b_speed,
                drive.motor_a_speed,
                drive.speed_diff,
                b_speed_avg,
                a_speed_avg,
                diff_avg,
                drive.pid_error,
                drive.p_term,
                drive.d_term,
                drive.feedforward_correction,
                drive.feedback_correction,
                drive.correction,
                left_pwm,
                right_pwm);
        }
    }

    TB6612_Brake();
    if (g_st011_pulse_remaining_ms != 0U) {
        delay_ms_with_st011(TASK11_POINT_ALARM_MS);
    }
    encoder_reset_distance_counts();
    if (stop_reason == 0U) {
        stop_reason = (lap_count >= TASK11_TARGET_LAPS) ? 1U :
            ((elapsed_ms >= TASK11_TOTAL_MAX_RUN_MS) ? 4U : 2U);
    }
    task11_ram_log_event(TASK11_RAM_EVENT_COMPLETE,
        stop_reason,
        lap_count,
        phase,
        elapsed_ms,
        0,
        yaw_cdeg,
        0,
        gzlp_mdps,
        ir_ok,
        &sample,
        0,
        0);
    task11_event_printf("TASK11_EVT_RT event=complete reason=%s t=%lu lap=%u phase=%u\r\n",
        task11_reason_name(stop_reason),
        elapsed_ms,
        lap_count,
        phase);
    task11_log_printf("TASK11 complete: continuous_ir reason=%s t=%lu lap=%u phase=%u\r\n",
        task11_reason_name(stop_reason),
        elapsed_ms,
        lap_count,
        phase);
    task11_ram_log_dump();
}

static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: A26/UART0 01=task1, A24/UART0 02=task2, B24/UART0 03=task3, A22/UART0 04=task4, UART0 05=PID test, 06=C turn, 07=PD, 10=AB zero, 11=IR map, 00=stop\r\n");

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
        } else if (task_id == TASK_ID_7) {
            TB6612_Brake();
            run_motor_pd_stream();
        } else if (task_id == TASK_ID_10) {
            TB6612_Brake();
            run_task10_ab_zero_test();
        } else if (task_id == TASK_ID_11) {
            TB6612_Brake();
            run_task11_ir_map_test();
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
