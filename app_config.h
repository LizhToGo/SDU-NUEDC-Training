#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 主控制循环周期。 */
#define CONTROL_PERIOD_MS (20)

/* 当前接线：左轮是 B 电机，右轮是 A 电机。 */
#define STRAIGHT_B_BASE_PWM (485)
#define STRAIGHT_A_BASE_PWM (525)
#define STRAIGHT_MIN_PWM    (0)
#define STRAIGHT_MAX_PWM    (650)

#define PID_REPORT_PERIOD_MS (100)

/* 功能开关。 */
#define ENABLE_IR_TRACKING_UART_TEST (0)
#define ENABLE_CONTEST_TASKS         (1)
#define ENABLE_LINE_FOLLOW_TEST      (0)
#define ENABLE_JY62_NAV              (1)
#define ENABLE_ENCODER_SELF_TEST     (0)

/* JY62 陀螺仪的置零和状态打印周期。 */
#define JY62_BOOT_ZERO_DELAY_MS    (300)
#define JY62_IDLE_REPORT_PERIOD_MS (500)
#define JY62_TASK_REPORT_PERIOD_MS (500)

/* 红外模块串口调试模式。 */
#define IR_TRACKING_TEST_PERIOD_MS (100)

/* 纯红外循迹调试模式。 */
#define LINE_FOLLOW_PERIOD_MS        (20)
#define LINE_FOLLOW_REPORT_PERIOD_MS (300)
#define LINE_FOLLOW_BASE_PWM         (480)
#define LINE_FOLLOW_MIN_PWM          (0)
#define LINE_FOLLOW_MAX_PWM          (820)
#define LINE_FOLLOW_TURN_DIVISOR     (11)
#define LINE_FOLLOW_TURN_LIMIT       (420)
#define LINE_FOLLOW_TURN_SIGN        (1)

/* 任务按键和任务一时序参数。 */
#define TASK_BUTTON_DEBOUNCE_MS (30)
#define TASK_BUTTON_IDLE_MS     (10)
#define TASK1_START_ALARM_MS    (120)
#define TASK1_FINISH_ALARM_MS   (120)
#define TASK1_START_SETTLE_MS   (250)
#define TASK1_AFTER_ZERO_DELAY_MS (100)
#define TASK1_START_RAMP_MS       (400)
#define TASK1_RAMP_B_START_PWM    (445)
#define TASK1_RAMP_A_START_PWM    (485)
#define TASK1_REPORT_PERIOD_MS    (100)
#define TASK1_MAX_RUN_MS          (15000)
#define TASK1_B_LINE_ARM_COUNT    (6000)
#define TASK1_FORCE_STOP_COUNT    (9500)
#define TASK1_STOP_MIN_IR_COUNT   (1)

/* ST011 声光模块低电平触发：空闲为高电平，短暂拉低表示提示。 */
#define ST011_ACTIVE_LOW (1)

/* 可选编码器自检参数；只有小车架空时才建议开启自检。 */
#define ENCODER_TEST_PWM  (260)
#define ENCODER_TEST_MS   (500)
#define ENCODER_MIN_PULSE (2)

/* 直行 PID，使用整数定点计算：output = (KP*err + KI*integral + KD*d_err) / SCALE。 */
#define STRAIGHT_PID_SCALE (20)
#define STRAIGHT_PID_KP    (22)
#define STRAIGHT_PID_KI    (2)
#define STRAIGHT_PID_KD    (6)
#define STRAIGHT_I_LIMIT   (180)
#define STRAIGHT_CORR_MAX  (80)

/* 任务一的编码器距离修正和 JY62 航向修正参数。 */
#define TASK1_DISTANCE_CORR_DIVISOR (28)
#define TASK1_DISTANCE_CORR_MAX     (45)
#define TASK1_HEADING_CORR_DIVISOR  (29)
#define TASK1_HEADING_CORR_MAX      (25)
#define TASK1_HEADING_CORR_SIGN     (-1)
#define TASK1_HEADING_FILTER_DIVISOR        (5)
#define TASK1_HEADING_WOBBLE_FILTER_DIVISOR (16)
#define TASK1_HEADING_DEADBAND_CDEG         (60)
#define TASK1_HEADING_GYRO_GATE_START_MDPS  (6000)
#define TASK1_HEADING_GYRO_GATE_END_MDPS    (18000)
#define TASK1_HEADING_PRIORITY_CDEG (250)
#define TASK1_HEADING_PRIORITY_MAX_VERR (18)
#define TASK1_HEADING_PRIORITY_MAX_DERR (240)

/* 编码器方向符号：统一让前进方向计数为正。 */
#define ENCODER_MOTOR_A_FORWARD_SIGN (-1)
#define ENCODER_MOTOR_B_FORWARD_SIGN (1)

#endif /* APP_CONFIG_H */
