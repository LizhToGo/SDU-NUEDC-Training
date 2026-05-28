#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"

#define CONTROL_PERIOD_MS (20)
#define STRAIGHT_B_BASE_PWM (225)
#define STRAIGHT_A_BASE_PWM (235)
#define PID_REPORT_PERIOD_MS (100)
#define STRAIGHT_MIN_PWM  (0)
#define STRAIGHT_MAX_PWM  (420)
#define ENCODER_TEST_PWM  (260)
#define ENCODER_TEST_MS   (500)
#define ENCODER_MIN_PULSE (2)
#define ENABLE_ENCODER_SELF_TEST (0)

#define STRAIGHT_PID_SCALE (20)
#define STRAIGHT_PID_KP    (16)
#define STRAIGHT_PID_KI    (1)
#define STRAIGHT_PID_KD    (4)
#define STRAIGHT_I_LIMIT   (120)
#define STRAIGHT_CORR_MAX  (60)

/*
 * Encoder names follow TB6612 motor channels:
 * motor A encoder uses PA16/PA17, motor B encoder uses PA14/PA15.
 */
#define ENCODER_MOTOR_A_FORWARD_SIGN (-1)
#define ENCODER_MOTOR_B_FORWARD_SIGN (1)

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t integral;
    int32_t last_error;
} straight_pid_t;

static volatile int32_t g_motor_a_encoder_count;
static volatile int32_t g_motor_b_encoder_count;
static volatile uint8_t g_motor_a_encoder_state;
static volatile uint8_t g_motor_b_encoder_state;

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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

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

static void encoder_update_motor_b(void)
{
    uint8_t current = encoder_read_state(ENCODER_MOTOR_B_A_PIN, ENCODER_MOTOR_B_B_PIN);
    int8_t delta = encoder_decode_delta(g_motor_b_encoder_state, current);

    g_motor_b_encoder_state = current;
    g_motor_b_encoder_count += ((int32_t)delta * ENCODER_MOTOR_B_FORWARD_SIGN);
}

static void encoder_update_motor_a(void)
{
    uint8_t current = encoder_read_state(ENCODER_MOTOR_A_A_PIN, ENCODER_MOTOR_A_B_PIN);
    int8_t delta = encoder_decode_delta(g_motor_a_encoder_state, current);

    g_motor_a_encoder_state = current;
    g_motor_a_encoder_count += ((int32_t)delta * ENCODER_MOTOR_A_FORWARD_SIGN);
}

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

static void encoder_enable_interrupts(void)
{
    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_MOTOR_B_A_PIN | ENCODER_MOTOR_B_B_PIN |
        ENCODER_MOTOR_A_A_PIN | ENCODER_MOTOR_A_B_PIN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

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
    return abs_i32(*motor_b_delta) + abs_i32(*motor_a_delta);
}

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

static void straight_pid_reset(straight_pid_t *pid)
{
    pid->kp = STRAIGHT_PID_KP;
    pid->ki = STRAIGHT_PID_KI;
    pid->kd = STRAIGHT_PID_KD;
    pid->integral = 0;
    pid->last_error = 0;
}

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

static void run_motor_pid_stream(void)
{
    straight_pid_t pid;
    uint32_t elapsed_ms = 0;
    uint32_t report_elapsed_ms = 0;
    int32_t motor_b_delta;
    int32_t motor_a_delta;

    straight_pid_reset(&pid);
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

        encoder_get_delta_counts(&motor_b_delta, &motor_a_delta);
        motor_b_speed = abs_i32(motor_b_delta);
        motor_a_speed = abs_i32(motor_a_delta);

        correction = straight_pid_update(&pid,
            motor_b_speed, motor_a_speed,
            &error, &p_term, &i_term, &d_term);
        motor_b_pwm = clamp_i32(STRAIGHT_B_BASE_PWM - correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);
        motor_a_pwm = clamp_i32(STRAIGHT_A_BASE_PWM + correction,
            STRAIGHT_MIN_PWM, STRAIGHT_MAX_PWM);

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

int main(void)
{
    SYSCFG_DL_init();
    lc_printf("\r\nBOOT: UART OK, straight PID firmware\r\n");
    delay_ms(200);

    encoder_init_runtime();
    lc_printf("BOOT: encoder state loaded, B=PA14/PA15 A=PA16/PA17\r\n");
    delay_ms(200);

    TB6612_Init();
    lc_printf("BOOT: TB6612 ready, A motor and B motor enabled\r\n");
    delay_ms(1000);

    if ((ENABLE_ENCODER_SELF_TEST != 0) && (encoder_motor_self_test() == 0U)) {
        lc_printf("Self-test failed. Fix wiring/direction before PID run.\r\n");
        while (1) {
            TB6612_Brake();
            delay_ms(1000);
        }
    }

    run_motor_pid_stream();

    while (1) {
    }
}

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
