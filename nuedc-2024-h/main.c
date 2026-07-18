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
 * 验收版固件主流程：
 * 1. 初始化 SysConfig、JY62、编码器和 TB6612。
 * 2. 进入阻塞式任务调度器。
 * 3. 只接受按键或 UART0 命令 01..04 启动任务一到任务四。
 */

/* JY62 相对航向零点建立后置位，避免重复置零。 */
static uint8_t g_jy62_zero_ready;

/**
 * @brief 竞速点位动作使用的红外辅助快速转向配置。
 *
 * 该类型保留在 main.c 中，供头文件形式的竞速原语提前使用。
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
 * @brief B 点、A 点出弧后使用的陀螺仪定航向转向配置。
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
 * @brief 打印一行 JY62 导航诊断信息。
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
 * @brief JY62 导航有效时，把当前航向设为相对零点。
 *
 * @return 置零成功返回 1，否则返回 0。
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

static void race_diff_pid_reset(straight_pid_t *pid);
static void race_drive_config(straight_drive_config_t *config,
    int32_t base_pwm,
    int32_t target_speed_diff);
static uint8_t run_task2_cd_exit_angle_straight(const char *tag);
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
