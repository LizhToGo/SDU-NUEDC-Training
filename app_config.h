/**
 * @file app_config.h
 * @brief 验收版自动行驶小车的集中调参入口。
 *
 * 本文件只放需要实车调试的常量：控制周期、传感器开关、四个验收
 * 任务的距离/航向/PWM 参数，以及任务三/四共享的竞速控制参数。
 * 修改参数时建议一次只改一小组，并在提交说明中记录实车现象。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ==========================================================================
 * 一、全局基础参数
 * ========================================================================== */

/* 默认主控制周期，单位 ms；任务一、任务二和任务三默认使用该周期。 */
#define CONTROL_PERIOD_MS (20)
/* 任务四高速模式使用更短控制周期，提高转向和航向修正响应。 */
#define RACE_TASK4_CONTROL_PERIOD_MS (10)

/* 当前接线约定：左轮为 B 电机，右轮为 A 电机。 */
#define STRAIGHT_B_BASE_PWM (628)
#define STRAIGHT_A_BASE_PWM (633)
/* 直线段 PWM 限幅；下限为 0 表示不允许直线闭环主动反转。 */
#define STRAIGHT_MIN_PWM    (0)
#define STRAIGHT_MAX_PWM    (870)

/* 直线基础航向辅助：小误差死区、比例除数、限幅和陀螺仪阻尼。 */
#define STRAIGHT_YAW_DEADBAND_CDEG     (20)
#define STRAIGHT_YAW_CORR_DIVISOR      (30)
#define STRAIGHT_YAW_CORR_MAX          (18)
#define STRAIGHT_YAW_GYRO_DAMP_DIVISOR (2600)

/* 生产任务是否启用 JY62 航向导航；置 0 可退化为纯编码器/红外控制。 */
#define ENABLE_JY62_NAV (1)

/* JY62 上电稳定后再置零；运行中按该周期输出导航诊断。 */
#define JY62_BOOT_ZERO_DELAY_MS    (300)
#define JY62_TASK_REPORT_PERIOD_MS (500)

/* 任务选择按键防抖和空闲轮询周期。 */
#define TASK_BUTTON_DEBOUNCE_MS (30)
#define TASK_BUTTON_IDLE_MS     (10)

/* ST011 声光模块低电平触发：空闲高电平，短暂拉低表示提示。 */
#define ST011_ACTIVE_LOW (1)

/* 编码器自检参数；只有车轮架空时才建议启用自检流程。 */
#define ENCODER_TEST_PWM  (260)
#define ENCODER_TEST_MS   (500)
#define ENCODER_MIN_PULSE (2)

/* 赛道几何换算：每厘米对应的平均编码器计数。 */
#define COUNTS_PER_CM (66)

/* 直线差速 PID：output = (KP*err + KI*I + KD*D) / SCALE。 */
#define STRAIGHT_PID_SCALE (20)
#define STRAIGHT_PID_KP    (22)
#define STRAIGHT_PID_KI    (2)
#define STRAIGHT_PID_KD    (6)
#define STRAIGHT_I_LIMIT   (180)
#define STRAIGHT_CORR_MAX  (140)

/* 默认目标轮速差，定义为 B_spd - A_spd；直线段通常保持 0。 */
#define STRAIGHT_TARGET_SPEED_DIFF (0)

/* 历史命名保留：直线和竞速共用的较宽限幅差速闭环参数。 */
#define PID_TEST_TARGET_SPEED_DIFF     (0)
#define PID_TEST_I_LIMIT               (600)
#define PID_TEST_CORR_MAX              (180)
#define PID_TEST_DIFF_FF_GAIN          (3)
#define PID_TEST_DISTANCE_CORR_DIVISOR (9)
#define PID_TEST_DISTANCE_CORR_MAX     (45)

/* 历史命名保留：竞速差速控制使用的 PD 参数。 */
#define PD_TEST_TARGET_SPEED_DIFF      (20)
#define PD_TEST_KP                     (22)
#define PD_TEST_KD                     (6)
#define PD_TEST_CORR_MAX               (180)
#define PD_TEST_DIFF_FF_GAIN           (3)
#define PD_TEST_DISTANCE_CORR_DIVISOR  (12)
#define PD_TEST_DISTANCE_CORR_MAX      (0)

/* ==========================================================================
 * 二、任务一：A -> B 直线
 * ========================================================================== */

/* 起点和终点声光提示时长；起步后等待航向稳定再进入闭环。 */
#define TASK1_START_ALARM_MS       (120)
#define TASK1_FINISH_ALARM_MS      (120)
#define TASK1_START_SETTLE_MS      (250)
#define TASK1_AFTER_ZERO_DELAY_MS  (100)

/* 起步斜坡，降低车体瞬时冲击，避免刚启动时偏航过大。 */
#define TASK1_START_RAMP_MS    (400)
#define TASK1_RAMP_B_START_PWM (560)
#define TASK1_RAMP_A_START_PWM (600)

/* 任务一串口诊断周期、最长运行时间、到线使能距离和保护距离。 */
#define TASK1_REPORT_PERIOD_MS    (100)
#define TASK1_MAX_RUN_MS          (15000)
#define TASK1_B_LINE_ARM_COUNT    (6000)
#define TASK1_FORCE_STOP_COUNT    (9500)
#define TASK1_STOP_MIN_IR_COUNT   (1)

/* 接近 B 点后的保守速度参数，保留给直线末端降速策略使用。 */
#define TASK1_APPROACH_SLOW_COUNT (5000)
#define TASK1_APPROACH_B_BASE_PWM (620)
#define TASK1_APPROACH_A_BASE_PWM (630)

/* 任务一编码器距离修正和 JY62 航向修正。 */
#define TASK1_DISTANCE_CORR_DIVISOR        (16)
#define TASK1_DISTANCE_CORR_MAX            (45)
#define TASK1_HEADING_CORR_DIVISOR         (12)
#define TASK1_HEADING_CORR_MAX             (90)
#define TASK1_HEADING_CORR_SIGN            (-1)
#define TASK1_HEADING_FILTER_DIVISOR       (5)
#define TASK1_HEADING_WOBBLE_FILTER_DIVISOR (8)
#define TASK1_HEADING_DEADBAND_CDEG        (60)
#define TASK1_HEADING_GYRO_GATE_START_MDPS (30000)
#define TASK1_HEADING_GYRO_GATE_END_MDPS   (80000)
#define TASK1_HEADING_PRIORITY_CDEG        (250)
#define TASK1_HEADING_PRIORITY_MAX_VERR    (18)
#define TASK1_HEADING_PRIORITY_MAX_DERR    (240)

/* ==========================================================================
 * 三、任务二：A -> B -> C -> D -> A
 * ========================================================================== */

/* 点位声光提示。 */
#define TASK2_POINT_ALARM_MS (120)

/* CD 直线出弧后的固定航向目标和航向修正强度。 */
#define TASK2_CD_STRAIGHT_TARGET_CDEG       (18000)
#define TASK2_CD_HEADING_CORR_DIVISOR       (5)
#define TASK2_CD_HEADING_CORR_MAX           (160)
#define TASK2_CD_HEADING_GYRO_DAMP_DIVISOR  (700)

/* CD 直线到线使能距离：避开 C 点出弯后的残留线形，再开始寻找 D 点。 */
#define TASK2_STRAIGHT_SEARCH_START_COUNT (5600)

/* BC/DA 弧线直接复用任务三/四竞速弧线控制，具体速度、出弧和保护参数见 TASK3_* / RACE_*。 */

/* ==========================================================================
 * 四、任务三：A -> C -> B -> D -> A，单圈竞速
 * ========================================================================== */

/* 任务三点位声光和弧线几何。 */
#define TASK3_POINT_ALARM_MS        (120)
#define TASK3_SENSOR_TO_AXIS_MM     (175)
#define TASK3_ARC_RADIUS_CM         (40)
#define TASK3_ARC_LENGTH_COUNT \
    ((31416L * TASK3_ARC_RADIUS_CM * COUNTS_PER_CM) / 10000L)
#define TASK3_ARC_EXIT_IGNORE_COUNT (TASK3_ARC_LENGTH_COUNT / 2L)
#define TASK3_ARC_EXIT_EXTRA_COUNT \
    (((TASK3_SENSOR_TO_AXIS_MM * COUNTS_PER_CM) + 5L) / 10L)
#define TASK3_ARC_FINISH_COUNT      (TASK3_ARC_LENGTH_COUNT + TASK3_ARC_EXIT_EXTRA_COUNT)
#define TASK3_ARC_FORCE_STOP_COUNT  (TASK3_ARC_FINISH_COUNT + 1800L)
#define TASK3_ARC_TURN_LEFT         (-1)
#define TASK3_ARC_TURN_RIGHT        (1)

/* 任务三历史直线航向和直线末端找线参数，保留兼容旧调参记录。 */
#define TASK3_AC_HEADING_TARGET_CDEG       (-3660)
#define TASK3_BD_HEADING_TARGET_CDEG       (-13920)
#define TASK3_STRAIGHT_STOP_MASK           (0xFFU)
#define TASK3_STRAIGHT_STOP_MIN_IR_COUNT   (1)
#define TASK3_STRAIGHT_CORR_MAX            (155)
#define TASK3_BD_HEADING_CORR_DIVISOR      (5)
#define TASK3_BD_HEADING_CORR_MAX          (170)
#define TASK3_STRAIGHT_LINE_ARM_COUNT      (6100)
#define TASK3_STRAIGHT_FORCE_STOP_COUNT    (11500)
#define TASK3_STRAIGHT_SEARCH_START_COUNT  (6100)
#define TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT (6500)
#define TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS   (70)
#define TASK3_STRAIGHT_SEARCH_CORR_DIVISOR (38)
#define TASK3_STRAIGHT_SEARCH_CORR_MAX     (145)
#define TASK3_STRAIGHT_SEARCH_SOFT_CORR    (85)
#define TASK3_STRAIGHT_SEARCH_SWEEP_CORR   (145)
#define TASK3_STRAIGHT_SEARCH_BASE_DROP    (60)

/* 任务三弧线基础 PWM、入弧策略和弧线诊断周期。 */
#define TASK3_ARC_B_BASE_PWM          (450)
#define TASK3_ARC_A_BASE_PWM          (480)
#define TASK3_ARC_MIN_PWM             (0)
#define TASK3_ARC_MAX_PWM             (620)
#define TASK3_ARC_ENTRY_COUNT         (3400)
#define TASK3_ARC_ENTRY_TURN          (130)
#define TASK3_ARC_ENTRY_BASE_DROP     (40)
#define TASK3_ARC_WIDE_LINE_MIN_COUNT (6)
#define TASK3_ARC_MAX_RUN_MS          (12000)
#define TASK3_ARC_REPORT_PERIOD_MS    (100)

/* 任务三当前竞速主流程使用的起跑对齐和直线航向。 */
#define RACE_TASK3_START_ALIGN_ENABLE (1)
#define RACE_TASK3_START_RIGHT_TURN_B_PWM      (RACE_EXIT_RIGHT_TURN_B_PWM)
#define RACE_TASK3_START_RIGHT_TURN_A_PWM      (RACE_EXIT_RIGHT_TURN_A_PWM)
#define RACE_TASK3_START_RIGHT_TURN_SLOW_B_PWM (RACE_EXIT_RIGHT_TURN_SLOW_B_PWM)
#define RACE_TASK3_START_RIGHT_TURN_SLOW_A_PWM (RACE_EXIT_RIGHT_TURN_SLOW_A_PWM)
#define RACE_TASK3_AC_HEADING_TARGET_CDEG (-3400)
#define RACE_TASK3_BD_HEADING_TARGET_CDEG (-18000 + 3550)
#define RACE_TASK3_AC_FORCE_TURN_COUNT    (7800)

/* ==========================================================================
 * 五、任务四：任务三路线四圈竞速
 * ========================================================================== */

/* 任务四圈数固定为 4；沿用任务三路线和阶段状态机。 */
#define TASK4_LAP_COUNT (4)

/* 任务四历史兼容字段，当前实际航向以 RACE_TASK4_* 为准。 */
#define TASK4_AC_HEADING_TARGET_CDEG TASK3_AC_HEADING_TARGET_CDEG
#define TASK4_BD_HEADING_TARGET_CDEG TASK3_BD_HEADING_TARGET_CDEG
#define TASK4_AC_LINE_SEARCH_PROTECT (2)
#define TASK4_AC_START_TURN_COUNT    (3400)
#define TASK4_AC_START_BOOST_MIN_ERR_CDEG    (1200)
#define TASK4_AC_START_HEADING_CORR_DIVISOR  (3)
#define TASK4_AC_START_HEADING_CORR_MAX      (300)
#define TASK4_AC_START_CORR_MAX              (285)

/* 任务四高速直线、弧线和起步斜坡参数。 */
#define RACE_TASK4_LINE_MAX_PWM             (950)
#define RACE_TASK4_STRAIGHT_BASE_PWM        (950)
#define RACE_TASK4_STRAIGHT_TARGET_DIFF     (0)
#define RACE_TASK4_FIRST_AC_RAMP_START_PWM  (640)
#define RACE_TASK4_FIRST_AC_RAMP_COUNT      (1000)
#define RACE_TASK4_ENTRY_DECEL_START_COUNT  (RACE_AC_POINT_ARM_COUNT - 500L)
#define RACE_TASK4_ENTRY_DECEL_RAMP_COUNT   (350L)
#define RACE_TASK4_ARC_BASE_PWM             (700)
#define RACE_TASK4_EXIT_DECEL_START_COUNT   (TASK3_ARC_LENGTH_COUNT - 2300L)
#define RACE_TASK4_EXIT_DECEL_RAMP_COUNT    (350L)

/* 任务四起跑对齐和场地图形航向。 */
#define RACE_TASK4_START_ALIGN_ENABLE (1)
#define RACE_TASK4_START_RIGHT_TURN_B_PWM      (RACE_EXIT_RIGHT_TURN_B_PWM)
#define RACE_TASK4_START_RIGHT_TURN_A_PWM      (RACE_EXIT_RIGHT_TURN_A_PWM)
#define RACE_TASK4_START_RIGHT_TURN_SLOW_B_PWM (RACE_EXIT_RIGHT_TURN_SLOW_B_PWM)
#define RACE_TASK4_START_RIGHT_TURN_SLOW_A_PWM (RACE_EXIT_RIGHT_TURN_SLOW_A_PWM)
#define RACE_TASK4_AC_HEADING_TARGET_CDEG (-3380)
#define RACE_TASK4_BD_HEADING_TARGET_CDEG (-18000 + 3270)

/* 任务四点位、强制入弯、预测停转和点后航向保持。 */
#define RACE_TASK4_POINT_ADVANCE_COUNT          (RACE_POINT_ADVANCE_COUNT)
#define RACE_TASK4_BD_POINT_ARM_COUNT           (6600)
#define RACE_TASK4_AC_FORCE_TURN_COUNT          (7720)
#define RACE_TASK4_FIRST_AC_FORCE_TURN_COUNT    (8000)
#define RACE_TASK4_BD_FORCE_TURN_COUNT          (7500)
#define RACE_TASK4_EXIT_TURN_PREDICT_ENABLE     (1)
#define RACE_TASK4_EXIT_TURN_PREDICT_MS         (20)
#define RACE_TASK4_EXIT_TURN_PREDICT_ARM_CDEG   (RACE_TURN_YAW_SLOW_ZONE_CDEG)
#define RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS \
    (RACE_TURN_YAW_STOP_GZLP_TOL_MDPS)
#define RACE_TASK4_ADVANCE_GYRO_ENABLE          (1)
#define RACE_TASK4_B_ADVANCE_HEADING_TARGET_CDEG (18000)
#define RACE_TASK4_A_ADVANCE_HEADING_TARGET_CDEG (0)
#define RACE_TASK4_ADVANCE_HEADING_CORR_DIVISOR  (14)
#define RACE_TASK4_ADVANCE_HEADING_CORR_MAX      (60)
#define RACE_TASK4_ADVANCE_GYRO_DAMP_DIVISOR     (2800)

/* 任务四高速入弯和保护入弯 PWM。 */
#define RACE_TASK4_ENTRY_LEFT_TURN_B_PWM       (-460)
#define RACE_TASK4_ENTRY_LEFT_TURN_A_PWM       (680)
#define RACE_TASK4_ENTRY_LEFT_TURN_SLOW_B_PWM  (-100)
#define RACE_TASK4_ENTRY_LEFT_TURN_SLOW_A_PWM  (420)
#define RACE_TASK4_ENTRY_RIGHT_TURN_B_PWM      (700)
#define RACE_TASK4_ENTRY_RIGHT_TURN_A_PWM      (-440)
#define RACE_TASK4_ENTRY_RIGHT_TURN_SLOW_B_PWM (420)
#define RACE_TASK4_ENTRY_RIGHT_TURN_SLOW_A_PWM (-100)
#define RACE_TASK4_FORCE_LEFT_TURN_B_PWM       (-330)
#define RACE_TASK4_FORCE_LEFT_TURN_A_PWM       (760)
#define RACE_TASK4_FORCE_LEFT_TURN_SLOW_B_PWM  (120)
#define RACE_TASK4_FORCE_LEFT_TURN_SLOW_A_PWM  (500)
#define RACE_TASK4_FORCE_RIGHT_TURN_B_PWM      (760)
#define RACE_TASK4_FORCE_RIGHT_TURN_A_PWM      (-130)
#define RACE_TASK4_FORCE_RIGHT_TURN_SLOW_B_PWM (500)
#define RACE_TASK4_FORCE_RIGHT_TURN_SLOW_A_PWM (120)

/* ==========================================================================
 * 六、任务三/任务四共享竞速控制
 * ========================================================================== */

/* 实时文本日志开关；高速实车默认关闭，避免影响控制周期。 */
#define RACE_UART_LOG_ENABLE (0)

/* 竞速点位声光、起步稳定时间和总超时。 */
#define RACE_POINT_ALARM_MS        (80)
#define RACE_START_ALARM_MS        (RACE_POINT_ALARM_MS)
#define RACE_POINT_SETTLE_MS       (120)
#define RACE_TOTAL_MAX_RUN_MS      (240000)
#define RACE_LINE_REPORT_PERIOD_MS (200)

/* 红外循迹基础控制参数：误差滤波、转向比例/微分、丢线补偿和限幅。 */
#define RACE_LINE_BASE_PWM             (560)
#define RACE_LINE_MIN_PWM              (0)
#define RACE_LINE_MAX_PWM              (860)
#define RACE_LINE_TURN_DIVISOR         (9)
#define RACE_LINE_KD_DIVISOR           (9)
#define RACE_LINE_DERIV_LIMIT          (700)
#define RACE_LINE_TURN_LIMIT           (240)
#define RACE_LINE_ERROR_FILTER_DIVISOR (3)
#define RACE_LINE_LOST_BASE_DROP       (60)
#define RACE_LINE_LOST_TURN            (150)

/* 竞速直线段基础 PWM、目标轮速差和航向/红外辅助修正。 */
#define RACE_STRAIGHT_BASE_PWM              (600)
#define RACE_STRAIGHT_TARGET_DIFF           (0)
#define RACE_STRAIGHT_GYRO_NAV_ENABLE       (1)
#define RACE_STRAIGHT_IR_ASSIST_ENABLE      (0)
#define RACE_STRAIGHT_IR_ASSIST_DIVISOR     (3)
#define RACE_STRAIGHT_HEADING_CORR_DIVISOR  (8)
#define RACE_STRAIGHT_HEADING_CORR_MAX      (140)
#define RACE_STRAIGHT_GYRO_DAMP_DIVISOR     (1600)

/* 竞速弧线基础 PWM、左右弧目标轮速差和航向辅助。 */
#define RACE_ARC_BASE_PWM              (540)
#define RACE_CB_ARC_ENTRY_TARGET_DIFF  (-48)
#define RACE_CB_ARC_CRUISE_TARGET_DIFF (-48)
#define RACE_DA_ARC_ENTRY_TARGET_DIFF  (46)
#define RACE_DA_ARC_CRUISE_TARGET_DIFF (46)
#define RACE_ARC_YAW_NAV_ENABLE        (1)
#define RACE_ARC_YAW_CORR_DIVISOR      (220)
#define RACE_ARC_YAW_CORR_MAX          (45)
#define RACE_ARC_GYRO_DAMP_DIVISOR     (4400)

/* 竞速差速闭环参数，复用上方 PD_TEST_* 的历史调参值。 */
#define RACE_DIFF_KP       (PD_TEST_KP)
#define RACE_DIFF_KD       (PD_TEST_KD)
#define RACE_DIFF_FF_GAIN  (2)
#define RACE_DIFF_CORR_MAX (PD_TEST_CORR_MAX)

/* 红外点位和转向停车掩码。 */
#define RACE_IR_LEFT_EDGE_MASK        (0x03U)
#define RACE_IR_RIGHT_EDGE_MASK       (0xC0U)
#define RACE_IR_CENTER_6_MASK         (0x3CU)
#define RACE_IR_CENTER_6_FORBID_MASK  (0x81U)
#define RACE_IR_CENTER_4_MASK         (0x3CU)
#define RACE_IR_CENTER_4_FORBID_MASK  (0xC3U)
#define RACE_IR_TURN_STOP_MIN_COUNT   (1)

/* 普通入弯 PWM：左弯用于 C 点，右弯用于 D 点。 */
#define RACE_LEFT_TURN_B_PWM       (-220)
#define RACE_LEFT_TURN_A_PWM       (620)
#define RACE_LEFT_TURN_SLOW_B_PWM  (80)
#define RACE_LEFT_TURN_SLOW_A_PWM  (360)
#define RACE_RIGHT_TURN_B_PWM      (620)
#define RACE_RIGHT_TURN_A_PWM      (20)
#define RACE_RIGHT_TURN_SLOW_B_PWM (360)
#define RACE_RIGHT_TURN_SLOW_A_PWM (80)

/* B/A 出弧后的陀螺仪转向 PWM。 */
#define RACE_EXIT_LEFT_TURN_B_PWM       (140)
#define RACE_EXIT_LEFT_TURN_A_PWM       (700)
#define RACE_EXIT_LEFT_TURN_SLOW_B_PWM  (60)
#define RACE_EXIT_LEFT_TURN_SLOW_A_PWM  (300)
#define RACE_EXIT_RIGHT_TURN_B_PWM      (640)
#define RACE_EXIT_RIGHT_TURN_A_PWM      (140)
#define RACE_EXIT_RIGHT_TURN_SLOW_B_PWM (300)
#define RACE_EXIT_RIGHT_TURN_SLOW_A_PWM (60)

/* 快速转向通用超时、降速和航向停车参数。 */
#define RACE_FAST_TURN_TIMEOUT_MS          (1000)
#define RACE_FAST_TURN_REPORT_PERIOD_MS    (40)
#define RACE_FAST_TURN_GYRO_SLOW_ENABLE    (1)
#define RACE_FAST_TURN_GYRO_SLOW_CDEG      (2600)
#define RACE_EXIT_TURN_YAW_STOP_ENABLE     (1)
#define RACE_TURN_YAW_STOP_TOL_CDEG        (260)
#define RACE_TURN_YAW_SLOW_ZONE_CDEG       (1000)
#define RACE_TURN_CROSS_ARM_CDEG           (RACE_TURN_YAW_SLOW_ZONE_CDEG)
#define RACE_TURN_YAW_STOP_GZLP_TOL_MDPS   (14000)
#define RACE_TURN_CENTER6_ERROR_MAX        (1500)

/* 点位后短距离前进，用于把车身从交叉点推入下一段。 */
#define RACE_POINT_ADVANCE_COUNT      (400)
#define RACE_ARC_POINT_ADVANCE_COUNT  (800)
#define RACE_POINT_ADVANCE_PWM        (340)
#define RACE_POINT_ADVANCE_TIMEOUT_MS (800)

/* 点位识别距离、强制入弯距离和弧线出口保护。 */
#define RACE_ARC_EXIT_IGNORE_MS          (1000)
#define RACE_AC_POINT_ARM_COUNT          (6500)
#define RACE_BD_POINT_ARM_COUNT          (7300)
#define RACE_BD_FORCE_TURN_COUNT         (7400)
#define RACE_FORCE_ENTRY_TURN_CDEG       (3500)
#define RACE_FORCE_FIND_LINE_COUNT       (1800)
#define RACE_FORCE_FIND_LINE_TIMEOUT_MS  (1200)
#define RACE_FORCE_FIND_LINE_PWM         (RACE_POINT_ADVANCE_PWM)
#define RACE_STRAIGHT_FORCE_COUNT        (12800)
#define RACE_STRAIGHT_POINT_CONFIRM_COUNT (1)
#define RACE_ARC_POINT_YAW_ARM_CDEG      (14000)
#define RACE_GYRO_TURN_TIMEOUT_MS        (1200)

/* ==========================================================================
 * 七、编码器方向
 * ========================================================================== */

/* 编码器方向符号：统一让小车前进时计数为正。 */
#define ENCODER_MOTOR_A_FORWARD_SIGN (-1)
#define ENCODER_MOTOR_B_FORWARD_SIGN (1)

#endif /* APP_CONFIG_H */
