#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "app_config.h"

/*
 * 这个模块当前以头文件方式集成，避免修改 CCS 自动生成的 Makefile。
 * 它只在 main.c 中 include 一次，因此这里的 static 状态和函数不会重复定义。
 */

/* 编码器计数在 GPIO 中断里更新，所以主循环读取时必须声明为 volatile。 */
static volatile int32_t g_motor_a_encoder_count;
static volatile int32_t g_motor_b_encoder_count;
static volatile uint8_t g_motor_a_encoder_state;
static volatile uint8_t g_motor_b_encoder_state;

/* 读取编码器 A/B 两相，压缩成 2 bit 状态：A 相为 bit1，B 相为 bit0。 */
static uint8_t encoder_read_state(uint32_t pin_a, uint32_t pin_b)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT, pin_a | pin_b);
    uint8_t state = 0U;

    if ((pins & pin_a) != 0U) {
        state |= 0x02U;
    }

    if ((pins & pin_b) != 0U) {
        state |= 0x01U;
    }

    return state;
}

/*
 * 解码一次正交编码器状态跳变。
 * 下标为 previous_state << 2 | current_state，
 * 所有 2 bit 到 2 bit 的跳变都会映射为 -1、0 或 +1。
 */
static int8_t encoder_decode_delta(uint8_t previous, uint8_t current)
{
    static const int8_t transition_table[16] = {
        0,  1, -1,  0,
       -1,  0,  0,  1,
        1,  0,  0, -1,
        0, -1,  1,  0
    };

    return transition_table[((previous & 0x03U) << 2) | (current & 0x03U)];
}

/* B 电机编码器发生 GPIO 中断时调用。 */
static void encoder_update_motor_b(void)
{
    uint8_t current = encoder_read_state(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);
    int8_t delta = encoder_decode_delta(g_motor_b_encoder_state, current);

    g_motor_b_encoder_state = current;
    g_motor_b_encoder_count += ((int32_t)delta * ENCODER_MOTOR_B_FORWARD_SIGN);
}

/* A 电机编码器发生 GPIO 中断时调用。 */
static void encoder_update_motor_a(void)
{
    uint8_t current = encoder_read_state(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
    int8_t delta = encoder_decode_delta(g_motor_a_encoder_state, current);

    g_motor_a_encoder_state = current;
    g_motor_a_encoder_count += ((int32_t)delta * ENCODER_MOTOR_A_FORWARD_SIGN);
}

/* 在开启 GPIO 中断前读取编码器初始状态。 */
static void encoder_init_runtime(void)
{
    g_motor_a_encoder_count = 0;
    g_motor_b_encoder_count = 0;
    g_motor_b_encoder_state = encoder_read_state(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);
    g_motor_a_encoder_state = encoder_read_state(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN |
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN);
}

/* 编码器初始状态有效后，开启 GPIOA 分组中断。 */
static void encoder_enable_interrupts(void)
{
    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN |
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN);

    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

/*
 * 返回距离上一次调用以来的编码器计数增量。
 * 读取 volatile 计数器时会短暂关中断，避免读到一半被中断打断。
 */
static void encoder_get_delta_counts(int32_t *motor_b_delta, int32_t *motor_a_delta)
{
    static int32_t last_motor_b_count;
    static int32_t last_motor_a_count;
    int32_t motor_b_count;
    int32_t motor_a_count;

    __disable_irq();
    motor_b_count = g_motor_b_encoder_count;
    motor_a_count = g_motor_a_encoder_count;
    __enable_irq();

    *motor_b_delta = motor_b_count - last_motor_b_count;
    *motor_a_delta = motor_a_count - last_motor_a_count;
    last_motor_b_count = motor_b_count;
    last_motor_a_count = motor_a_count;
}

static void encoder_get_total_counts(int32_t *motor_b_total, int32_t *motor_a_total)
{
    __disable_irq();
    *motor_b_total = g_motor_b_encoder_count;
    *motor_a_total = g_motor_a_encoder_count;
    __enable_irq();
}

static void encoder_reset_distance_counts(void)
{
    int32_t dummy_b;
    int32_t dummy_a;

    __disable_irq();
    g_motor_b_encoder_count = 0;
    g_motor_a_encoder_count = 0;
    g_motor_b_encoder_state = encoder_read_state(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);
    g_motor_a_encoder_state = encoder_read_state(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
    __enable_irq();

    encoder_get_delta_counts(&dummy_b, &dummy_a);
}

/* 在固定时间窗口内测量编码器变化量，用于可选的编码器自检。 */
static int32_t encoder_measure_for_ms(uint32_t ms, int32_t *motor_b_delta, int32_t *motor_a_delta)
{
    uint32_t elapsed_ms = 0;
    int32_t sample_b;
    int32_t sample_a;

    encoder_get_delta_counts(&sample_b, &sample_a);

    while (elapsed_ms < ms) {
        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
    }

    encoder_get_delta_counts(motor_b_delta, motor_a_delta);
    return ((*motor_b_delta < 0) ? -*motor_b_delta : *motor_b_delta) +
           ((*motor_a_delta < 0) ? -*motor_a_delta : *motor_a_delta);
}

/*
 * 可选接线自检。
 * 每次只转一个电机，检查对应编码器是否有计数。
 */
static uint8_t encoder_motor_self_test(void)
{
    int32_t motor_b_delta;
    int32_t motor_a_delta;
    int32_t motor_b_abs;
    int32_t motor_a_abs;
    uint8_t ok = 1U;

    lc_printf("Encoder self-test: B motor\r\n");
    TB6612_SetDifferential(ENCODER_TEST_PWM, 0);
    encoder_measure_for_ms(ENCODER_TEST_MS, &motor_b_delta, &motor_a_delta);
    TB6612_Brake();
    delay_ms(300);

    motor_b_abs = (motor_b_delta < 0) ? -motor_b_delta : motor_b_delta;
    motor_a_abs = (motor_a_delta < 0) ? -motor_a_delta : motor_a_delta;
    lc_printf("B motor test count: B=%ld A=%ld\r\n", motor_b_delta, motor_a_delta);

    if (motor_b_abs < ENCODER_MIN_PULSE) {
        lc_printf("ERROR: B motor encoder has no pulse. Check PA14/PA15, PWMB, BIN1/BIN2, and motor B wiring.\r\n");
        ok = 0U;
    }

    if (motor_a_abs > motor_b_abs) {
        lc_printf("ERROR: B motor test counted more on A motor encoder. Check encoder channel definitions or wiring.\r\n");
        ok = 0U;
    }

    lc_printf("Encoder self-test: A motor\r\n");
    TB6612_SetDifferential(0, ENCODER_TEST_PWM);
    encoder_measure_for_ms(ENCODER_TEST_MS, &motor_b_delta, &motor_a_delta);
    TB6612_Brake();
    delay_ms(300);

    motor_b_abs = (motor_b_delta < 0) ? -motor_b_delta : motor_b_delta;
    motor_a_abs = (motor_a_delta < 0) ? -motor_a_delta : motor_a_delta;
    lc_printf("A motor test count: B=%ld A=%ld\r\n", motor_b_delta, motor_a_delta);

    if (motor_a_abs < ENCODER_MIN_PULSE) {
        lc_printf("ERROR: A motor encoder has no pulse. Check PA16/PA17, PWMA, AIN1/AIN2, and motor A wiring.\r\n");
        ok = 0U;
    }

    if (motor_b_abs > motor_a_abs) {
        lc_printf("ERROR: A motor test counted more on B motor encoder. Check encoder channel definitions or wiring.\r\n");
        ok = 0U;
    }

    TB6612_Brake();
    return ok;
}

#endif /* BSP_ENCODER_H */
