#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include <stdint.h>
#include "board.h"

/*
 * Current SysConfig PWM period is 1000 and both PWM outputs are inverted.
 * Therefore compare values 0..999 can be used like a duty command here.
 */
#define TB6612_PWM_MAX (999U)

/*
 * Current wiring from 暂行接线图.md:
 * Real-car self-test: PWMA=PA0 right wheel, PWMB=PA1 left wheel,
 * AIN1=PB6, AIN2=PB7, BIN1=PB8, BIN2=PB9, STBY=PB1.
 *
 * Default forward levels follow the WHEELTEC TB6612 demo:
 * motor A forward: AIN1=0, AIN2=1
 * motor B forward: BIN1=0, BIN2=1
 *
 * If your car runs backward, either swap the motor wires or change these
 * four values.
 */
#ifndef TB6612_A_FORWARD_IN1
#define TB6612_A_FORWARD_IN1 (0U)
#endif

#ifndef TB6612_A_FORWARD_IN2
#define TB6612_A_FORWARD_IN2 (1U)
#endif

#ifndef TB6612_B_FORWARD_IN1
#define TB6612_B_FORWARD_IN1 (0U)
#endif

#ifndef TB6612_B_FORWARD_IN2
#define TB6612_B_FORWARD_IN2 (1U)
#endif

#define AIN1_OUT(X) ((X) ? (DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN)))
#define AIN2_OUT(X) ((X) ? (DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN)))
#define BIN1_OUT(X) ((X) ? (DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN)))
#define BIN2_OUT(X) ((X) ? (DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN)))
#define STBY_OUT(X) ((X) ? (DL_GPIO_setPins(TB6612_PORT, TB6612_STBY_PIN)) : (DL_GPIO_clearPins(TB6612_PORT, TB6612_STBY_PIN)))

typedef enum {
    TB6612_MOTOR_A = 0,
    TB6612_MOTOR_B = 1
} tb6612_motor_t;

void TB6612_Init(void);
void TB6612_Enable(void);
void TB6612_Disable(void);
void TB6612_Brake(void);
void TB6612_Coast(void);
void TB6612_SetMotor(tb6612_motor_t motor, int16_t speed);
void TB6612_SetDifferential(int16_t left_speed, int16_t right_speed);

/* Legacy LCKFB-style APIs kept for old examples. dir=1 forward, dir=0 reverse. */
void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif /* _BSP_TB6612_H */
