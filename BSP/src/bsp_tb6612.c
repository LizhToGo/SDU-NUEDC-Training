#include "bsp_tb6612.h"

/* 将带符号速度命令转换为限幅后的 PWM 比较值。 */
static uint32_t TB6612_AbsClamp(int16_t speed)
{
    int32_t value = speed;

    if (value < 0) {
        value = -value;
    }

    if (value > (int32_t)TB6612_PWM_MAX) {
        value = (int32_t)TB6612_PWM_MAX;
    }

    return (uint32_t)value;
}

/* A 电机使用 PWM 通道 C0，B 电机使用 PWM 通道 C1。 */
static void TB6612_SetPwm(tb6612_motor_t motor, uint32_t pwm)
{
    if (pwm > TB6612_PWM_MAX) {
        pwm = TB6612_PWM_MAX;
    }

    if (motor == TB6612_MOTOR_A) {
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm, GPIO_PWM_0_C0_IDX);
    } else {
        DL_TimerA_setCaptureCompareValue(PWM_0_INST, pwm, GPIO_PWM_0_C1_IDX);
    }
}

/* 根据正反转方向设置 TB6612 某一路半桥的方向引脚电平。 */
static void TB6612_SetDirection(tb6612_motor_t motor, uint8_t forward)
{
    uint8_t in1;
    uint8_t in2;

    if (motor == TB6612_MOTOR_A) {
        in1 = (uint8_t)TB6612_A_FORWARD_IN1;
        in2 = (uint8_t)TB6612_A_FORWARD_IN2;

        if (!forward) {
            in1 = !in1;
            in2 = !in2;
        }

        AIN1_OUT(in1);
        AIN2_OUT(in2);
    } else {
        in1 = (uint8_t)TB6612_B_FORWARD_IN1;
        in2 = (uint8_t)TB6612_B_FORWARD_IN2;

        if (!forward) {
            in1 = !in1;
            in2 = !in2;
        }

        BIN1_OUT(in1);
        BIN2_OUT(in2);
    }
}

void TB6612_Init(void)
{
    /* 先进入待机状态，再切到明确的刹车状态。 */
    TB6612_Disable();
    TB6612_Brake();
}

void TB6612_Enable(void)
{
    STBY_OUT(1);
}

void TB6612_Disable(void)
{
    TB6612_SetPwm(TB6612_MOTOR_A, 0);
    TB6612_SetPwm(TB6612_MOTOR_B, 0);
    STBY_OUT(0);
}

void TB6612_Brake(void)
{
    TB6612_SetPwm(TB6612_MOTOR_A, TB6612_PWM_MAX);
    TB6612_SetPwm(TB6612_MOTOR_B, TB6612_PWM_MAX);

    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
    TB6612_Enable();
}

void TB6612_Coast(void)
{
    TB6612_SetPwm(TB6612_MOTOR_A, 0);
    TB6612_SetPwm(TB6612_MOTOR_B, 0);

    AIN1_OUT(0);
    AIN2_OUT(0);
    BIN1_OUT(0);
    BIN2_OUT(0);
    TB6612_Enable();
}

void TB6612_SetMotor(tb6612_motor_t motor, int16_t speed)
{
    uint32_t pwm = TB6612_AbsClamp(speed);

    if ((motor != TB6612_MOTOR_A) && (motor != TB6612_MOTOR_B)) {
        lc_printf("\r\nTB6612_SetMotor parameter error\r\n");
        return;
    }

    /* 速度为 0 时让该电机滑行停止，而不是主动刹车。 */
    if (pwm == 0U) {
        if (motor == TB6612_MOTOR_A) {
            AIN1_OUT(0);
            AIN2_OUT(0);
        } else {
            BIN1_OUT(0);
            BIN2_OUT(0);
        }

        TB6612_SetPwm(motor, 0);
        TB6612_Enable();
        return;
    }

    if (speed >= 0) {
        TB6612_SetDirection(motor, 1);
    } else {
        TB6612_SetDirection(motor, 0);
    }

    TB6612_Enable();
    TB6612_SetPwm(motor, pwm);
}

void TB6612_SetDifferential(int16_t left_speed, int16_t right_speed)
{
    /* 当前接线：B 电机是左轮，A 电机是右轮。 */
    TB6612_SetMotor(TB6612_MOTOR_B, left_speed);
    TB6612_SetMotor(TB6612_MOTOR_A, right_speed);
}
