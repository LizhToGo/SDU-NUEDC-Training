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
#include "bsp_encoder.h"

/*
 * Acceptance-mode firmware:
 * 1. Initialize SysConfig, JY62, encoders, and TB6612.
 * 2. Enter the blocking task dispatcher.
 * 3. Accept only Task1..Task4 from buttons or UART0 commands 01..04.
 */

/* Set once the firmware has established a JY62 relative-yaw zero. */
static uint8_t g_jy62_zero_ready;

/**
 * @brief IR-assisted fast-turn command used by race point actions.
 *
 * The type remains in main.c because race_primitives.h is included header-only
 * and uses this configuration before the concrete race lap code is included.
 */
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
    uint32_t control_period_ms;
} sensor_fast_turn_config_t;

/**
 * @brief Gyro-only turn command used after exiting B/A arc segments.
 */
typedef struct {
    const char *tag;
    int16_t motor_b_pwm;
    int16_t motor_a_pwm;
    int16_t slow_motor_b_pwm;
    int16_t slow_motor_a_pwm;
    int32_t yaw_stop_target_cdeg;
    uint8_t predictive_stop_enable;
    int32_t predictive_stop_ms;
    int32_t predictive_stop_min_gz_mdps;
    uint32_t control_period_ms;
} gyro_turn_config_t;

#if RACE_UART_LOG_ENABLE
#define race_log_printf(...) lc_printf(__VA_ARGS__)
#else
#define race_log_printf(...) ((void)0)
#endif

/**
 * @brief Print one JY62 navigation diagnostic line.
 */
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

/**
 * @brief Set the current JY62 yaw as the relative zero when navigation is valid.
 *
 * @return 1 when zeroing succeeds, otherwise 0.
 */
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

/**
 * @brief Choose the end-of-line search direction for Task2 straight exits.
 */
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

#include "race/task2_ram_log.h"

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
static uint8_t race_advance_after_point_with_heading(const char *tag,
    int32_t advance_count,
    int32_t target_cdeg);
static uint8_t race_peek_yaw(int32_t *yaw_cdeg, int32_t *gyro_z_filtered_mdps);
static uint8_t race_gyro_turn_to_yaw(
    const gyro_turn_config_t *config);

/**
 * @brief Rotate after a Task2 arc exit until the expected heading is reached.
 */
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

#include "turn/arc_segment.h"
#include "straight/straight_line.h"
#include "race/race_laps.h"
#include "tasks/task_sequences.h"
#include "tasks/task_dispatcher.h"

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

    run_task_dispatcher();

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
