#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include <stdint.h>
#include "board.h"

/*
 * 当前 SysConfig 中 PWM 周期为 1000，且两路 PWM 输出均为反相。
 * 因此这里可以把 0..999 的比较值近似当作占空比命令使用。
 */
#define TB6612_PWM_MAX (999U)

/*
 * 当前接线参考《暂行接线图.md》：
 * 实车自测结果：PWMA=PA0 右轮，PWMB=PA1 左轮，
 * AIN1=PB6，AIN2=PB7，BIN1=PB8，BIN2=PB9，STBY=PB1。
 *
 * 默认前进电平参考 WHEELTEC TB6612 例程：
 * A 电机前进：AIN1=0，AIN2=1
 * B 电机前进：BIN1=0，BIN2=1
 *
 * 如果小车前进方向反了，可以交换电机线，或者修改下面四个宏。
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

#endif /* 结束 _BSP_TB6612_H 头文件保护 */
