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

static void st011_pulse(uint32_t pulse_ms)
{
    st011_set_active(1U);
    delay_ms(pulse_ms);
    st011_set_active(0U);
}

static uint8_t task_button_is_pressed(void)
{
    return ((DL_GPIO_readPins(KEYS_PORT, KEYS_KEY2_PIN) & KEYS_KEY2_PIN) == 0U) ? 1U : 0U;
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

static void wait_task_button_press(void)
{
    uint32_t elapsed_ms = 0;
    uint32_t jy62_elapsed_ms = 0;

    while (1) {
        if (task_button_is_pressed() != 0U) {
            delay_ms(TASK_BUTTON_DEBOUNCE_MS);
            if (task_button_is_pressed() != 0U) {
                while (task_button_is_pressed() != 0U) {
                    delay_ms(TASK_BUTTON_IDLE_MS);
                }
                delay_ms(TASK_BUTTON_DEBOUNCE_MS);
                return;
            }
        }

        TB6612_Brake();
        delay_ms(TASK_BUTTON_IDLE_MS);
        elapsed_ms += TASK_BUTTON_IDLE_MS;
        jy62_elapsed_ms += TASK_BUTTON_IDLE_MS;

        if (jy62_elapsed_ms >= JY62_IDLE_REPORT_PERIOD_MS) {
            jy62_elapsed_ms = 0;
            jy62_print_navigation_line("idle", elapsed_ms);
        }
    }
}

static void run_task1_ab(void)
{
    straight_pid_t pid;
    heading_filter_t heading_filter;
    ir_tracking_sample_t sample;
    jy62_navigation_t nav;
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    uint32_t jy62_report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    uint8_t ir_ok = 0U;
    uint8_t stop_reason = 0U;
    uint8_t stop_nav_ok;

    TB6612_Brake();
    st011_pulse(TASK1_START_ALARM_MS);
    delay_ms(TASK1_START_SETTLE_MS);
    (void)jy62_zero_to_current("task1_zero", TASK1_START_SETTLE_MS);
    delay_ms(TASK1_AFTER_ZERO_DELAY_MS);

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
        int32_t heading_error;
        int32_t heading_filtered;
        int32_t heading_gain;
        int32_t heading_gyro_z;
        int32_t heading_correction;
        int32_t correction;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;
        uint8_t nav_ok;
        uint8_t ir_armed;
        uint8_t heading_priority;
        uint8_t heading_wobble;

        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;
        jy62_report_elapsed_ms += CONTROL_PERIOD_MS;

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        encoder_get_total_counts(&motor_b_total, &motor_a_total);

        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);
        distance_count = (abs_i32(motor_b_total) + abs_i32(motor_a_total)) / 2;
        ir_armed = (distance_count >= TASK1_B_LINE_ARM_COUNT) ? 1U : 0U;
        nav_ok = JY62_PeekNavigation(&nav);
        heading_raw = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
        heading_gyro_z = (nav_ok != 0U) ? nav.gyro_z_filtered_mdps : 0;
        heading_filtered = 0;
        heading_gain = 0;
        heading_wobble = 0U;
        heading_error = (nav_ok != 0U) ?
            heading_filter_update(&heading_filter,
                heading_raw,
                heading_gyro_z,
                &heading_filtered,
                &heading_gain,
                &heading_wobble) : 0;
        heading_correction = clamp_i32((heading_error * TASK1_HEADING_CORR_SIGN) / TASK1_HEADING_CORR_DIVISOR,
            -TASK1_HEADING_CORR_MAX, TASK1_HEADING_CORR_MAX);

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
            lc_printf("TASK1 t=%lu dist=%ld arm=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u h_raw=%ld h_flt=%ld h_use=%ld h_gain=%ld h_wob=%u gzlp=%ld h_corr=%ld hp=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
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
            jy62_print_navigation_line("task1", elapsed_ms);
        }
    }

    TB6612_Brake();
    encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
    encoder_get_total_counts(&motor_b_delta, &motor_a_delta);
    stop_nav_ok = JY62_PeekNavigation(&nav);
    lc_printf("TASK1 stop: reason=%s t=%lu dist=%ld arm=%d force=%d raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u rel_cdeg=%ld B_total=%ld A_total=%ld\r\n",
        (stop_reason == 1U) ? "line" : ((stop_reason == 2U) ? "force" : "timeout"),
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
    st011_pulse(TASK1_FINISH_ALARM_MS);
}

static void run_task_dispatcher(void)
{
    uint32_t task_count = 0;

    st011_set_active(0U);
    TB6612_Brake();
    lc_printf("TASK ready: press PB21/KEY2. press 1=task1 A->B straight PID\r\n");

    while (1) {
        wait_task_button_press();
        task_count++;

        if (task_count == 1U) {
            run_task1_ab();
        } else {
            TB6612_Brake();
            st011_pulse(TASK1_START_ALARM_MS);
            lc_printf("TASK%lu not implemented yet\r\n", task_count);
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
