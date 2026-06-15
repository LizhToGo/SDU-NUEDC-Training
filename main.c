#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"
#include "app_config.h"
#include "app_control.h"
#include "app_motion_utils.h"
#include "app_services.h"
#include "app_straight.h"
#include "app_task_ids.h"
#include "app_debug_modes.h"
#include "line_fast_turn.h"
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
static uint8_t g_task6_c_turn_requested;

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

typedef struct {
    const char *tag;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int16_t slow_motor_b_pwm;
    int16_t slow_motor_a_pwm;
    uint8_t stop_mask;
    uint8_t forbid_mask;
    int32_t stop_error_max;
    uint8_t yaw_stop_enable;
    int32_t yaw_stop_target_cdeg;
} sensor_fast_turn_config_t;

typedef struct {
    const char *tag;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int16_t slow_motor_b_pwm;
    int16_t slow_motor_a_pwm;
    int32_t yaw_stop_target_cdeg;
} gyro_turn_config_t;

#if TASK11_UART_LOG_ENABLE
#define race_log_printf(...) lc_printf(__VA_ARGS__)
#else
#define race_log_printf(...) ((void)0)
#endif

#if TASK11_REALTIME_EVENT_LOG_ENABLE
#define race_event_printf(...) lc_printf(__VA_ARGS__)
#else
#define race_event_printf(...) ((void)0)
#endif

static void jy62_print_navigation_line(const char *mode, uint32_t elapsed_ms)
{
#if ENABLE_JY62_NAV
    jy62_navigation_t nav = {0};
    uint32_t frame_delta = JY62_GetNavigation(&nav);

    if ((g_jy62_zero_ready == 0U) && (nav.valid != 0U)) {
        JY62_SetYawZeroToCurrent();
        g_jy62_zero_ready = 1U;
        frame_delta = JY62_GetNavigation(&nav);
    }

    lc_printf("JY62 mode=%s t=%lu df=%lu ok=%u flags=0x%02X yaw_cdeg=%ld rel_cdeg=%ld gz_mdps=%ld gyro_z_filtered_mdps=%ld rx=%lu head=%lu frames=%lu err=%lu/%lu/%lu\r\n",
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
    jy62_navigation_t nav = {0};

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

#include "heading_straight.h"

#include "task2_ram_log.h"

static void race_diff_pid_reset(straight_pid_t *pid);
static void race_drive_config(straight_drive_config_t *config,
    int32_t base_pwm,
    int32_t target_speed_diff);
static uint8_t run_task2_cd_exit_angle_straight(const char *tag);
static uint8_t run_task2_da_race_arc(const char *tag);
static int32_t race_heading_turn_from_error(int32_t heading_error_cdeg,
    int32_t gyro_z_filtered_mdps,
    int32_t corr_divisor,
    int32_t gyro_damp_divisor,
    int32_t corr_max);
static int32_t race_arc_expected_yaw_cdeg(int32_t phase_distance_count,
    int32_t phase_turn_dir);
static uint8_t race_advance_after_point(const char *tag, int32_t advance_count);
static uint8_t race_peek_yaw(int32_t *yaw_cdeg, int32_t *gyro_z_filtered_mdps);
static uint8_t race_gyro_turn_to_yaw(
    const gyro_turn_config_t *config);

static uint8_t task2_post_exit_arc_to_yaw(const char *tag,
    int32_t yaw_start_cdeg,
    int32_t turn_sign)
{
    jy62_navigation_t nav = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t yaw_cdeg = 0;
    int32_t target_cdeg;
    int32_t error_cdeg = 0;
    int32_t last_error_cdeg = 0;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    uint8_t nav_ok;
    uint8_t error_valid = 0U;
    uint8_t stop_reason = 0U;

    turn_sign = (turn_sign < 0) ? -1 : 1;
    target_cdeg = normalize_cdeg(yaw_start_cdeg +
        (turn_sign * TASK2_ARC_ALIGN_TARGET_CDEG));
    if (turn_sign < 0) {
        motor_b_pwm = TASK2_ARC_ALIGN_OUTER_PWM;
        motor_a_pwm = TASK2_ARC_ALIGN_INNER_PWM;
    } else {
        motor_b_pwm = TASK2_ARC_ALIGN_INNER_PWM;
        motor_a_pwm = TASK2_ARC_ALIGN_OUTER_PWM;
    }

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    TB6612_SetDifferential(motor_b_pwm, motor_a_pwm);
    race_log_printf("%s start: yaw0=%ld target=%ld sign=%ld pwm=%d/%d\r\n",
        tag,
        yaw_start_cdeg,
        target_cdeg,
        turn_sign,
        motor_b_pwm,
        motor_a_pwm);

    while (elapsed_ms < TASK2_ARC_ALIGN_TIMEOUT_MS) {
        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

        nav_ok = JY62_PeekNavigation(&nav);
        if (nav_ok == 0U) {
            stop_reason = 5U;
            break;
        }

        yaw_cdeg = nav.yaw_relative_cdeg;
        error_cdeg = normalize_cdeg(target_cdeg - yaw_cdeg);
        if (abs_i32(error_cdeg) <= TASK2_ARC_ALIGN_TOL_CDEG) {
            stop_reason = 1U;
            break;
        }
        if ((error_valid != 0U) &&
            (((last_error_cdeg < 0) && (error_cdeg > 0)) ||
             ((last_error_cdeg > 0) && (error_cdeg < 0)))) {
            stop_reason = 1U;
            break;
        }
        last_error_cdeg = error_cdeg;
        error_valid = 1U;

        if (report_elapsed_ms >= TASK2_ARC_REPORT_PERIOD_MS) {
            report_elapsed_ms = 0;
            encoder_get_total_counts(&motor_b_total, &motor_a_total);
            race_log_printf("%s t=%lu yaw=%ld target=%ld err=%ld yprog=%ld gzlp=%ld B=%ld A=%ld\r\n",
                tag,
                elapsed_ms,
                yaw_cdeg,
                target_cdeg,
                error_cdeg,
                abs_i32(normalize_cdeg(yaw_cdeg - yaw_start_cdeg)),
                nav.gyro_z_filtered_mdps,
                motor_b_total,
                motor_a_total);
        }
    }

    if (stop_reason != 1U) {
        TB6612_Brake();
    }
    if (stop_reason == 0U) {
        stop_reason = 2U;
    }
    (void)JY62_PeekNavigation(&nav);
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    race_log_printf("%s stop: reason=%s t=%lu yaw=%ld target=%ld err=%ld yprog=%ld B=%ld A=%ld\r\n",
        tag,
        (stop_reason == 1U) ? "yaw" : ((stop_reason == 2U) ? "timeout" : ((stop_reason == 3U) ? "uart_stop" : "nav_invalid")),
        elapsed_ms,
        nav.yaw_relative_cdeg,
        target_cdeg,
        normalize_cdeg(target_cdeg - nav.yaw_relative_cdeg),
        abs_i32(normalize_cdeg(nav.yaw_relative_cdeg - yaw_start_cdeg)),
        motor_b_total,
        motor_a_total);

    return (stop_reason == 1U) ? 1U : 0U;
}

#include "arc_segment.h"

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
    if (abs_i32(yaw_error_cdeg) <= TASK5_YAW_DEADBAND_CDEG) {
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

static uint8_t run_straight_to_line_segment(
    const straight_line_segment_config_t *config)
{
    const char *tag = config->tag;
    uint8_t zero_heading = config->zero_heading;
    uint32_t start_alarm_ms = config->start_alarm_ms;
    uint32_t stop_alarm_ms = config->stop_alarm_ms;
    int32_t line_arm_count = config->line_arm_count;
    int32_t force_stop_count = config->force_stop_count;
    uint8_t stop_min_ir_count = config->stop_min_ir_count;
    uint8_t yaw_corr_enable = config->yaw_corr_enable;
    uint8_t entry_brake_enable = config->entry_brake_enable;
    uint8_t fixed_yaw_target_enable = config->fixed_yaw_target_enable;
    int32_t fixed_yaw_target_cdeg = config->fixed_yaw_target_cdeg;
    straight_pid_t pid;
    straight_drive_config_t drive_config;
    straight_drive_output_t drive = {0};
    ir_tracking_sample_t sample = {0};
    jy62_navigation_t nav = {0};
    jy62_navigation_t nav_start = {0};
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint32_t jy62_report_elapsed_ms = 0;
    int32_t motor_b_delta = 0;
    int32_t motor_a_delta = 0;
    int32_t motor_b_total = 0;
    int32_t motor_a_total = 0;
    int32_t distance_count = 0;
    int32_t yaw_correction = 0;
    int32_t correction = 0;
    uint8_t ir_ok = 0U;
    uint8_t nav_ok = 0U;
    uint8_t nav_start_ok = 0U;
    uint8_t stop_nav_ok = 0U;
    uint8_t stop_reason = 0U;
    uint8_t task2_ram_mode;
    uint32_t report_period_ms;
    int32_t yaw_start_cdeg = 0;
    int32_t yaw_target_cdeg = 0;
    int32_t feedforward_correction;
    int32_t target_b_base_pwm;
    int32_t target_a_base_pwm;

    if (entry_brake_enable != 0U) {
        TB6612_Brake();
    }
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
    straight_drive_config_pid_test(&drive_config);
    target_b_base_pwm = drive_config.base_b_pwm;
    target_a_base_pwm = drive_config.base_a_pwm;
    straight_pid_reset(&pid);
    straight_pid_set_limits(&pid, PID_TEST_I_LIMIT, PID_TEST_CORR_MAX);
    feedforward_correction = straight_drive_feedforward(&drive_config);
#if ENABLE_JY62_NAV
    nav_start_ok = JY62_PeekNavigation(&nav_start);
    if (nav_start_ok != 0U) {
        yaw_start_cdeg = nav_start.yaw_relative_cdeg;
    }
#endif
    yaw_target_cdeg = (fixed_yaw_target_enable != 0U) ?
        normalize_cdeg(fixed_yaw_target_cdeg) : yaw_start_cdeg;

    encoder_reset_distance_counts();
    encoder_enable_interrupts();
    task2_ram_mode = task2_ram_enabled_for_tag(tag);
    report_period_ms = (task2_ram_mode != 0U) ?
        TASK2_STRAIGHT_RAM_LOG_PERIOD_MS : TASK1_REPORT_PERIOD_MS;
    if (task2_ram_mode != 0U) {
        task2_ram_log_event(tag,
            TASK2_EVT_START,
            0U,
            0U,
            0,
            yaw_start_cdeg,
            abs_i32(normalize_cdeg(yaw_start_cdeg - yaw_target_cdeg)),
            0U,
            &sample,
            nav_start_ok,
            0U,
            0,
            0);
    } else {
        lc_printf("%s start: ctrl=task5 zero=%u ycorr=%u brake=%u rpt=%lu yaw0=%ld target=%ld fixed=%u B_base=%ld A_base=%ld ramp=%d/%d/%dms target_diff=%ld ff_gain=%ld ff_corr=%ld d_div=%ld d_max=%ld i_limit=%d corr_max=%ld arm=%ld force=%ld stop_min=%u nav0=%u\r\n",
            tag,
            zero_heading,
            yaw_corr_enable,
            entry_brake_enable,
            report_period_ms,
            yaw_start_cdeg,
            yaw_target_cdeg,
            fixed_yaw_target_enable,
            target_b_base_pwm,
            target_a_base_pwm,
            TASK1_RAMP_B_START_PWM,
            TASK1_RAMP_A_START_PWM,
            TASK1_START_RAMP_MS,
            drive_config.target_speed_diff,
            drive_config.diff_ff_gain,
            feedforward_correction,
            drive_config.distance_corr_divisor,
            drive_config.distance_corr_max,
            PID_TEST_I_LIMIT,
            drive_config.correction_max,
            line_arm_count,
            force_stop_count,
            stop_min_ir_count,
            nav_start_ok);
    }
    TB6612_SetDifferential((int16_t)TASK1_RAMP_B_START_PWM,
        (int16_t)TASK1_RAMP_A_START_PWM);

    while (elapsed_ms < TASK1_MAX_RUN_MS) {
        uint8_t ir_armed;

        delay_ms_with_st011(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;
        jy62_report_elapsed_ms += CONTROL_PERIOD_MS;

        if (task_uart_stop_requested() != 0U) {
            stop_reason = 3U;
            break;
        }

#if ENABLE_JY62_NAV
        (void)JY62_GetNavigation(&nav);
        nav_ok = nav.valid;
#else
        nav_ok = 0U;
#endif
        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);
        drive_config.base_b_pwm = ramp_i32(TASK1_RAMP_B_START_PWM,
            target_b_base_pwm,
            elapsed_ms,
            TASK1_START_RAMP_MS);
        drive_config.base_a_pwm = ramp_i32(TASK1_RAMP_A_START_PWM,
            target_a_base_pwm,
            elapsed_ms,
            TASK1_START_RAMP_MS);
        straight_drive_update(&pid,
            &drive_config,
            motor_b_delta,
            motor_a_delta,
            motor_b_total,
            motor_a_total,
            &drive);
        if (yaw_corr_enable != 0U) {
            yaw_correction = (fixed_yaw_target_enable != 0U) ?
                task2_fixed_yaw_correction(nav_ok, &nav, yaw_target_cdeg) :
                task5_yaw_correction(nav_ok, &nav, yaw_target_cdeg);
        } else {
            yaw_correction = 0;
        }
        correction = clamp_i32(drive.correction + yaw_correction,
            -drive_config.correction_max,
            drive_config.correction_max);
        drive.motor_b_pwm = clamp_i32(drive_config.base_b_pwm - correction,
            drive_config.min_pwm,
            drive_config.max_pwm);
        drive.motor_a_pwm = clamp_i32(drive_config.base_a_pwm + correction,
            drive_config.min_pwm,
            drive_config.max_pwm);

        distance_count = drive.distance_count;
        ir_armed = (distance_count >= line_arm_count) ? 1U : 0U;
        ir_ok = IRTracking_ReadSample(&sample);
        if ((ir_armed != 0U) &&
            (ir_ok != 0U) &&
            (sample.line_lost == 0U) &&
            (sample.active_count >= stop_min_ir_count)) {
            stop_reason = 1U;
            break;
        }
        if (distance_count >= force_stop_count) {
            stop_reason = 2U;
            break;
        }

        TB6612_SetDifferential((int16_t)drive.motor_b_pwm,
            (int16_t)drive.motor_a_pwm);

        if (report_elapsed_ms >= report_period_ms) {
            report_elapsed_ms = 0;
            if (task2_ram_mode != 0U) {
                int32_t phase_yaw_cdeg = (nav_ok != 0U) ?
                    normalize_cdeg(nav.yaw_relative_cdeg - yaw_start_cdeg) : 0;
                int32_t heading_error_cdeg = (nav_ok != 0U) ?
                    normalize_cdeg(nav.yaw_relative_cdeg - yaw_target_cdeg) : 0;
                task2_ram_log_sample(tag,
                    elapsed_ms,
                    distance_count,
                    (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
                    phase_yaw_cdeg,
                    abs_i32(phase_yaw_cdeg),
                    yaw_target_cdeg,
                    heading_error_cdeg,
                    (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
                    ir_ok,
                    &sample,
                    nav_ok,
                    ir_armed,
                    motor_b_total,
                    motor_a_total,
                    &drive,
                    correction,
                    0,
                    yaw_correction,
                    correction,
                    drive_config.target_speed_diff,
                    drive.motor_b_pwm,
                    drive.motor_a_pwm);
            } else {
                lc_printf("%s t=%lu dist=%ld arm=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u yaw=%ld gzlp=%ld ycorr=%ld B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_tgt=%ld v_err=%ld P=%ld I=%ld D=%ld ff=%ld fb=%ld corr=%ld B_pwm=%ld A_pwm=%ld\r\n",
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
                    (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
                    (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0,
                    yaw_correction,
                    motor_b_total,
                    motor_a_total,
                    drive.distance_error,
                    drive.distance_correction,
                    drive.motor_b_speed,
                    drive.motor_a_speed,
                    drive_config.target_speed_diff,
                    drive.pid_error,
                    drive.p_term,
                    drive.i_term,
                    drive.d_term,
                    drive.feedforward_correction,
                    drive.feedback_correction,
                    correction,
                    drive.motor_b_pwm,
                    drive.motor_a_pwm);
            }
        }

        if ((task2_ram_mode == 0U) &&
            (jy62_report_elapsed_ms >= JY62_TASK_REPORT_PERIOD_MS)) {
            jy62_report_elapsed_ms = 0;
            jy62_print_navigation_line(tag, elapsed_ms);
        }
    }

    if ((stop_alarm_ms != 0U) || (stop_reason != 1U)) {
        TB6612_Brake();
    }
    encoder_get_total_counts(&motor_b_total, &motor_a_total);
    distance_count = motion_distance_count(motor_b_total, motor_a_total);
    stop_nav_ok = JY62_PeekNavigation(&nav);
    if (task2_ram_mode != 0U) {
        int32_t stop_heading_error_cdeg = (stop_nav_ok != 0U) ?
            normalize_cdeg(nav.yaw_relative_cdeg - yaw_target_cdeg) : 0;
        task2_ram_log_event(tag,
            (stop_reason == 1U) ? TASK2_EVT_POINT : TASK2_EVT_STOP,
            stop_reason,
            elapsed_ms,
            distance_count,
            (stop_nav_ok != 0U) ? nav.yaw_relative_cdeg : 0,
            abs_i32(stop_heading_error_cdeg),
            ir_ok,
            &sample,
            stop_nav_ok,
            (stop_reason == 1U) ? 1U : 0U,
            motor_b_total,
            motor_a_total);
    } else {
        lc_printf("%s stop: reason=%s t=%lu dist=%ld arm=%ld force=%ld raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u rel_cdeg=%ld B_total=%ld A_total=%ld\r\n",
            tag,
            (stop_reason == 1U) ? "line" : ((stop_reason == 2U) ? "force" : ((stop_reason == 3U) ? "uart_stop" : "timeout")),
            elapsed_ms,
            distance_count,
            line_arm_count,
            force_stop_count,
            (ir_ok != 0U) ? sample.raw : 0xFFU,
            (ir_ok != 0U) ? sample.line_mask : 0U,
            (ir_ok != 0U) ? sample.active_count : 0U,
            (ir_ok != 0U) ? sample.line_lost : 1U,
            ir_ok,
            stop_nav_ok,
            nav.yaw_relative_cdeg,
            motor_b_total,
            motor_a_total);
    }
    if (stop_alarm_ms != 0U) {
        st011_pulse(stop_alarm_ms);
    }

    return stop_reason;
}

static void run_task1_ab(void)
{
    const straight_line_segment_config_t config = {
        .tag = "TASK1_AB",
        .zero_heading = 1U,
        .start_alarm_ms = TASK1_START_ALARM_MS,
        .stop_alarm_ms = TASK1_FINISH_ALARM_MS,
        .line_arm_count = TASK1_B_LINE_ARM_COUNT,
        .force_stop_count = TASK1_FORCE_STOP_COUNT,
        .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
        .yaw_corr_enable = 1U,
        .entry_brake_enable = 1U,
        .fixed_yaw_target_enable = 0U,
        .fixed_yaw_target_cdeg = 0
    };

    (void)run_straight_to_line_segment(&config);
}

static void run_task2_abcd(void)
{
    uint8_t reason;

    task2_ram_log_reset();
    task2_ram_log_event("TASK2_AB",
        TASK2_EVT_START,
        0U,
        0U,
        0,
        0,
        0,
        0U,
        0,
        0U,
        0U,
        0,
        0);

    {
        const straight_line_segment_config_t config = {
            .tag = "TASK2_AB",
            .zero_heading = 0U,
            .start_alarm_ms = TASK1_START_ALARM_MS,
            .stop_alarm_ms = 0U,
            .line_arm_count = TASK1_B_LINE_ARM_COUNT,
            .force_stop_count = TASK1_FORCE_STOP_COUNT,
            .stop_min_ir_count = TASK1_STOP_MIN_IR_COUNT,
            .yaw_corr_enable = 1U,
            .entry_brake_enable = 1U,
            .fixed_yaw_target_enable = 0U,
            .fixed_yaw_target_cdeg = 0
        };

        reason = run_straight_to_line_segment(&config);
    }
    if (reason != 1U) {
        task2_ram_log_event("TASK2_AB",
            TASK2_EVT_ABORT,
            reason,
            0U,
            0,
            0,
            0,
            0U,
            0,
            0U,
            0U,
            0,
            0);
        st011_finish_pending_pulse();
        task2_ram_log_dump();
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    reason = run_task2_bc_race_arc_debug("TASK2_BC");
    if (reason != 1U) {
        task2_ram_log_event("TASK2_BC",
            TASK2_EVT_ABORT,
            reason,
            0U,
            0,
            0,
            0,
            0U,
            0,
            0U,
            0U,
            0,
            0);
        st011_finish_pending_pulse();
        task2_ram_log_dump();
        return;
    }

    reason = run_task2_cd_exit_angle_straight("TASK2_CD");
    if (reason != 1U) {
        task2_ram_log_event("TASK2_CD",
            TASK2_EVT_ABORT,
            reason,
            0U,
            0,
            0,
            0,
            0U,
            0,
            0U,
            0U,
            0,
            0);
        st011_finish_pending_pulse();
        task2_ram_log_dump();
        return;
    }
    st011_start_pulse(TASK2_POINT_ALARM_MS);

    task2_ram_log_event("TASK2_CD",
        TASK2_EVT_COMPLETE,
        reason,
        0U,
        0,
        0,
        0,
        0U,
        0,
        0U,
        0U,
        0,
        0);

    reason = run_task2_da_race_arc("TASK2_DA");
    if (reason != 1U) {
        task2_ram_log_event("TASK2_DA",
            TASK2_EVT_ABORT,
            reason,
            0U,
            0,
            0,
            0,
            0U,
            0,
            0U,
            0U,
            0,
            0);
        st011_finish_pending_pulse();
        task2_ram_log_dump();
        return;
    }

    TB6612_Brake();
    task2_ram_log_event("TASK2_DA",
        TASK2_EVT_COMPLETE,
        reason,
        0U,
        0,
        0,
        0,
        0U,
        0,
        0U,
        0U,
        0,
        0);
    st011_finish_pending_pulse();
    task2_ram_log_dump();
}

static uint8_t task3_arc_stop_is_success(uint8_t stop_reason)
{
    return ((stop_reason == 1U) || (stop_reason == 3U)) ? 1U : 0U;
}

static void run_task6_ac_c_turn_test(void)
{
    uint8_t reason;

    lc_printf("TASK6 start: UART 06, run task3 AC only, then fast left turn at C target=%d pwm=%d/%d\r\n",
        TASK6_C_TURN_TARGET_CDEG,
        TASK6_C_TURN_B_PWM,
        TASK6_C_TURN_A_PWM);

    g_task6_c_turn_requested = 1U;
    {
        const heading_straight_segment_config_t config = {
            .tag = "TASK6_AC",
            .zero_heading = 0U,
            .heading_target_cdeg = TASK3_AC_HEADING_TARGET_CDEG,
            .heading_only = 1U,
            .fast_correction = 0U,
            .line_search_protect = 2U,
            .start_alarm_ms = TASK1_START_ALARM_MS,
            .stop_alarm_ms = 0U
        };

        reason = run_heading_straight_to_line_segment(&config);
    }
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

#include "race_laps.h"



static void run_task_dispatcher(void)
{
    task_id_t task_id;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: buttons A26/A24/B24/A22 or UART0 HEX bytes 01..07,10; 00=stop while running; ASCII t01..t10 still ok\r\n");

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
            run_race_laps(1U);
        } else if (task_id == TASK_ID_4) {
            run_race_laps(4U);
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
