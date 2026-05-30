#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "bsp_ir_tracking.h"
#include "bsp_jy62.h"

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

/* 直行 PID 的控制周期。每隔 20 ms 读取一次编码器增量并更新 PWM。 */
#define CONTROL_PERIOD_MS (20)

/* 左轮 B 电机、右轮 A 电机的基础 PWM。A 轮略大是为了补偿实车左右差异。 */
#define STRAIGHT_B_BASE_PWM (410)
#define STRAIGHT_A_BASE_PWM (450)

/* PID 数据打印周期。串口打印太频繁会拖慢主循环，所以每 100 ms 打印一次。 */
#define PID_REPORT_PERIOD_MS (100)

/*
 * 八路红外循迹模块串口打印测试开关。
 *
 * 1：只测试红外模块，I2C 读数后从 UART0 打印，电机不会跑。
 * 0：关闭红外测试，恢复后面的编码器直行 PID 跑车程序。
 */
#define ENABLE_IR_TRACKING_UART_TEST (0)

/* 红外模块测试打印周期。100 ms 适合人眼观察，也不会让串口输出太密。 */
#define IR_TRACKING_TEST_PERIOD_MS   (100)

#define ENABLE_CONTEST_TASKS         (1)
#define ENABLE_LINE_FOLLOW_TEST      (0)
#define ENABLE_JY62_NAV             (1)
#define JY62_BOOT_ZERO_DELAY_MS      (300)
#define JY62_IDLE_REPORT_PERIOD_MS   (500)
#define JY62_TASK_REPORT_PERIOD_MS   (500)
#define LINE_FOLLOW_PERIOD_MS        (20)
#define LINE_FOLLOW_REPORT_PERIOD_MS (300)
#define LINE_FOLLOW_BASE_PWM         (480)
#define LINE_FOLLOW_MIN_PWM          (0)
#define LINE_FOLLOW_MAX_PWM          (820)
#define LINE_FOLLOW_TURN_DIVISOR     (11)
#define LINE_FOLLOW_TURN_LIMIT       (420)
#define LINE_FOLLOW_TURN_SIGN        (1)

/* 调试阶段限制 PWM 范围，避免小车突然加速过高。 */
#define STRAIGHT_MIN_PWM  (0)
#define STRAIGHT_MAX_PWM  (520)

#define TASK_BUTTON_DEBOUNCE_MS       (30)
#define TASK_BUTTON_IDLE_MS           (10)
#define TASK1_START_ALARM_MS          (120)
#define TASK1_FINISH_ALARM_MS         (120)
#define TASK1_START_SETTLE_MS         (250)
#define TASK1_AFTER_ZERO_DELAY_MS     (100)
#define TASK1_START_RAMP_MS           (400)
#define TASK1_RAMP_B_START_PWM        (370)
#define TASK1_RAMP_A_START_PWM        (410)
#define TASK1_REPORT_PERIOD_MS        (100)
#define TASK1_MAX_RUN_MS              (15000)
/* Start looking for the B-point black line after the car is close enough to B. */
#define TASK1_B_LINE_ARM_COUNT        (6000)
/* If the B-point line is missed, keep going a little farther and then stop. */
#define TASK1_FORCE_STOP_COUNT        (9500)
#define TASK1_STOP_MIN_IR_COUNT       (1)

/* ST011 trigger is treated as active-low: idle high, short low pulse to notify. */
#define ST011_ACTIVE_LOW              (1)

/* 编码器自检参数。只有小车架空时才建议把 ENABLE_ENCODER_SELF_TEST 改为 1。 */
#define ENCODER_TEST_PWM  (260)
#define ENCODER_TEST_MS   (500)
#define ENCODER_MIN_PULSE (2)
#define ENABLE_ENCODER_SELF_TEST (0)

/*
 * 直行 PID 使用整数定点计算，避免在 Cortex-M0+ 上引入浮点开销。
 * 实际输出修正量 = (KP*误差 + KI*积分 + KD*微分) / STRAIGHT_PID_SCALE。
 */
#define STRAIGHT_PID_SCALE (20)
#define STRAIGHT_PID_KP    (22)
#define STRAIGHT_PID_KI    (2)
#define STRAIGHT_PID_KD    (6)

/* 积分限幅和修正量限幅，用来降低积分饱和和突然大幅修正。 */
#define STRAIGHT_I_LIMIT   (180)
#define STRAIGHT_CORR_MAX  (80)

#define TASK1_DISTANCE_CORR_DIVISOR (28)
#define TASK1_DISTANCE_CORR_MAX     (45)
#define TASK1_HEADING_CORR_DIVISOR  (35)
#define TASK1_HEADING_CORR_MAX      (25)
#define TASK1_HEADING_CORR_SIGN     (-1)
#define TASK1_HEADING_PRIORITY_CDEG (250)
#define TASK1_HEADING_PRIORITY_MAX_VERR (18)
#define TASK1_HEADING_PRIORITY_MAX_DERR (240)

/*
 * 编码器命名跟随 TB6612 电机通道：
 * A 电机编码器使用 PA16/PA17，B 电机编码器使用 PA14/PA15。
 */
#define ENCODER_MOTOR_A_FORWARD_SIGN (-1)
#define ENCODER_MOTOR_B_FORWARD_SIGN (1)

typedef struct {
    /* PID 三个系数都使用整数；最终统一除以 STRAIGHT_PID_SCALE。 */
    int32_t kp;
    int32_t ki;
    int32_t kd;

    /* integral 保存历史误差累积，last_error 用来计算微分项。 */
    int32_t integral;
    int32_t last_error;
} straight_pid_t;

/* 编码器计数在 GPIO 中断里更新，所以主循环读取时必须声明为 volatile。 */
static volatile int32_t g_motor_a_encoder_count;
static volatile int32_t g_motor_b_encoder_count;
static volatile uint8_t g_motor_a_encoder_state;
static volatile uint8_t g_motor_b_encoder_state;
static uint8_t g_jy62_zero_ready;

/* 限幅函数，用于 PWM、积分限幅和 PID 修正量限幅。 */
static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int32_t ramp_i32(int32_t start_value, int32_t target_value, uint32_t elapsed_ms, uint32_t ramp_ms)
{
    if ((ramp_ms == 0U) || (elapsed_ms >= ramp_ms)) {
        return target_value;
    }

    return start_value + (((target_value - start_value) * (int32_t)elapsed_ms) / (int32_t)ramp_ms);
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

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

    /*
     * ENCODER_MOTOR_B_FORWARD_SIGN 用来统一“前进时计数为正”的方向。
     * 如果以后换了编码器 A/B 相顺序，可以优先改这个符号宏。
     */
    g_motor_b_encoder_count += ((int32_t)delta * ENCODER_MOTOR_B_FORWARD_SIGN);
}

/* A 电机编码器发生 GPIO 中断时调用。 */
static void encoder_update_motor_a(void)
{
    uint8_t current = encoder_read_state(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
    int8_t delta = encoder_decode_delta(g_motor_a_encoder_state, current);

    g_motor_a_encoder_state = current;

    /* A 电机实测前进方向与解码表方向相反，所以当前符号为 -1。 */
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

    /* GPIOA 分组中断打开后，GROUP1_IRQHandler 才会持续更新编码器计数。 */
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

    /* 这里不做控制，只等待指定时间，让编码器累计一段可观测的脉冲。 */
    while (elapsed_ms < ms) {
        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
    }

    encoder_get_delta_counts(motor_b_delta, motor_a_delta);
    return abs_i32(*motor_b_delta) + abs_i32(*motor_a_delta);
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

    /*
     * 自检的判断逻辑：只让 B 电机转动时，B 编码器应该明显有计数，
     * A 编码器不应该比 B 编码器计数更多；A 电机同理。
     */
    lc_printf("Encoder self-test: B motor\r\n");
    TB6612_SetDifferential(ENCODER_TEST_PWM, 0);
    encoder_measure_for_ms(ENCODER_TEST_MS, &motor_b_delta, &motor_a_delta);
    TB6612_Brake();
    delay_ms(300);

    motor_b_abs = abs_i32(motor_b_delta);
    motor_a_abs = abs_i32(motor_a_delta);
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

    motor_b_abs = abs_i32(motor_b_delta);
    motor_a_abs = abs_i32(motor_a_delta);
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

/* 进入直行闭环控制前，重置 PID 运行状态。 */
static void straight_pid_reset(straight_pid_t *pid)
{
    pid->kp = STRAIGHT_PID_KP;
    pid->ki = STRAIGHT_PID_KI;
    pid->kd = STRAIGHT_PID_KD;
    pid->integral = 0;
    pid->last_error = 0;
}

/*
 * PID 目标：让 B 电机速度等于 A 电机速度。
 * correction 为正表示 B 轮更快，因此会降低 B 轮 PWM、提高 A 轮 PWM。
 */
static int32_t straight_pid_update(straight_pid_t *pid,
    int32_t motor_b_speed,
    int32_t motor_a_speed,
    int32_t *error_out,
    int32_t *p_out,
    int32_t *i_out,
    int32_t *d_out)
{
    int32_t error = motor_b_speed - motor_a_speed;
    int32_t derivative = error - pid->last_error;
    int32_t p_term;
    int32_t i_term;
    int32_t d_term;
    int32_t output;

    /*
     * err = B_spd - A_spd：
     * err > 0 表示左轮 B 比右轮 A 快，需要降低 B、提高 A；
     * err < 0 表示右轮 A 比左轮 B 快，需要提高 B、降低 A。
     */
    pid->integral = clamp_i32(pid->integral + error, -STRAIGHT_I_LIMIT, STRAIGHT_I_LIMIT);
    pid->last_error = error;

    p_term = pid->kp * error;
    i_term = pid->ki * pid->integral;
    d_term = pid->kd * derivative;
    output = p_term + i_term + d_term;
    output /= STRAIGHT_PID_SCALE;

    *error_out = error;
    *p_out = p_term;
    *i_out = i_term;
    *d_out = d_term;

    return clamp_i32(output, -STRAIGHT_CORR_MAX, STRAIGHT_CORR_MAX);
}

/* 主电机闭环任务。该函数会一直运行，并每 100 ms 打印一次 PID 数据。 */
static void run_motor_pid_stream(void)
{
    straight_pid_t pid;
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;

    straight_pid_reset(&pid);

    /* 先清掉第一次读取的历史增量，避免启动瞬间把旧计数当作速度。 */
    encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
    TB6612_SetDifferential(STRAIGHT_B_BASE_PWM, STRAIGHT_A_BASE_PWM);
    lc_printf("PID motor stream start: B_base=%d A_base=%d period=%dms report=%dms\r\n",
        STRAIGHT_B_BASE_PWM, STRAIGHT_A_BASE_PWM, CONTROL_PERIOD_MS, PID_REPORT_PERIOD_MS);
    encoder_enable_interrupts();
    lc_printf("PID encoder IRQ enabled, B=PA14/PA15 A=PA16/PA17\r\n");

    while (1) {
        int32_t motor_b_speed;
        int32_t motor_a_speed;
        int32_t error;
        int32_t p_term;
        int32_t i_term;
        int32_t d_term;
        int32_t correction;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;

        delay_ms(CONTROL_PERIOD_MS);
        elapsed_ms += CONTROL_PERIOD_MS;
        report_elapsed_ms += CONTROL_PERIOD_MS;

        /* 用 20 ms 时间窗口内的编码器增量作为简化速度估计。 */
        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);

        correction = straight_pid_update(&pid,
            motor_b_speed, motor_a_speed,
            &error, &p_term, &i_term, &d_term);

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
            report_elapsed_ms = 0;
            lc_printf("PID t=%lu B_cnt=%ld A_cnt=%ld B_spd=%ld A_spd=%ld err=%ld P=%ld I=%ld D=%ld corr=%ld B_pwm=%ld A_pwm=%ld\r\n",
                elapsed_ms,
                motor_b_delta, motor_a_delta,
                motor_b_speed, motor_a_speed,
                error, p_term, i_term, d_term,
                correction, motor_b_pwm, motor_a_pwm);
        }
    }
}

/* 八路红外循迹模块测试：I2C 读取模块数据，再通过调试串口打印。 */
static int32_t line_follow_calculate_turn(int32_t error)
{
    int32_t turn = (error * LINE_FOLLOW_TURN_SIGN) / LINE_FOLLOW_TURN_DIVISOR;

    return clamp_i32(turn, -LINE_FOLLOW_TURN_LIMIT, LINE_FOLLOW_TURN_LIMIT);
}

static void run_line_follow_test(void)
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

static void run_task1_ab(void)
{
    straight_pid_t pid;
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
        int32_t heading_error;
        int32_t heading_correction;
        int32_t correction;
        int32_t base_b_pwm;
        int32_t base_a_pwm;
        int32_t motor_b_pwm;
        int32_t motor_a_pwm;
        uint8_t nav_ok;
        uint8_t ir_armed;
        uint8_t heading_priority;

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
        heading_error = (nav_ok != 0U) ? nav.yaw_relative_cdeg : 0;
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
            lc_printf("TASK1 t=%lu dist=%ld arm=%u raw=0x%02X mask=0x%02X cnt=%u lost=%u ir=%u nav=%u h_err=%ld h_corr=%ld hp=%u B_total=%ld A_total=%ld d_err=%ld d_corr=%ld B_spd=%ld A_spd=%ld v_err=%ld P=%ld I=%ld D=%ld v_corr=%ld bal=%ld corr=%ld base=%ld/%ld B_pwm=%ld A_pwm=%ld\r\n",
                elapsed_ms,
                distance_count,
                ir_armed,
                (ir_ok != 0U) ? sample.raw : 0xFFU,
                (ir_ok != 0U) ? sample.line_mask : 0U,
                (ir_ok != 0U) ? sample.active_count : 0U,
                (ir_ok != 0U) ? sample.line_lost : 1U,
                ir_ok,
                nav_ok,
                heading_error,
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

static void run_ir_tracking_uart_test(void)
{
    ir_tracking_sample_t sample;
    uint32_t elapsed_ms = 0;

    IRTracking_Init();

    /* 红外测试只验证传感器读数，电机保持刹车，避免调试时小车移动。 */
    TB6612_Brake();

    lc_printf("\r\nIR tracking UART test start\r\n");
    lc_printf("I2C: addr=0x%02X reg=0x%02X SDA=PB3 SCL=PB2 period=%dms\r\n",
        IR_TRACKING_I2C_ADDR, IR_TRACKING_DATA_REG, IR_TRACKING_TEST_PERIOD_MS);
    lc_printf("raw bit7..bit0 = X1..X8, mask bit0..bit7 = X1..X8 black line\r\n");
    lc_printf("error: left negative, center zero, right positive\r\n");

    while (1) {
        /*
         * IRTracking_ReadSample() 完成一次完整采样：
         * I2C 读取原始字节 -> 转换黑线掩码 -> 计算位置误差。
         */
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
