# app_config 宏定义说明

`app_config.h` 是当前验收版的集中调参入口。固件只保留任务一、任务二、任务三、任务四；历史调试入口相关宏已删除。

## 全局控制

| 宏 | 含义 |
|---|---|
| `CONTROL_PERIOD_MS` | 默认控制周期 |
| `RACE_TASK4_CONTROL_PERIOD_MS` | 任务四控制周期 |
| `ENABLE_JY62_NAV` | 是否启用 JY62 导航 |

## 直线基础参数

| 宏 | 含义 |
|---|---|
| `STRAIGHT_B_BASE_PWM` / `STRAIGHT_A_BASE_PWM` | B/A 轮基础 PWM |
| `STRAIGHT_MIN_PWM` / `STRAIGHT_MAX_PWM` | 直线 PWM 限幅 |
| `STRAIGHT_PID_*` | 直线差速 PID |
| `STRAIGHT_TARGET_SPEED_DIFF` | 默认 B-A 速度差目标 |
| `STRAIGHT_YAW_*` | 直线 yaw 辅助修正 |

`PID_TEST_*` 和 `PD_TEST_*` 是历史命名的共享差速闭环参数，当前不再对应 UART 调试任务。

## 任务一

| 宏 | 含义 |
|---|---|
| `TASK1_START_ALARM_MS` / `TASK1_FINISH_ALARM_MS` | 起终点声光 |
| `TASK1_START_RAMP_MS` | 起步斜坡 |
| `TASK1_B_LINE_ARM_COUNT` | B 点线检测使能距离 |
| `TASK1_FORCE_STOP_COUNT` | 距离保护 |
| `TASK1_DISTANCE_CORR_*` | 编码器距离修正 |
| `TASK1_HEADING_*` | JY62 航向修正 |

## 任务二

| 宏 | 含义 |
|---|---|
| `TASK2_POINT_ALARM_MS` | 点位声光 |
| `TASK2_RAM_LOG_ENABLE` | 任务二 RAM 日志 |
| `TASK2_CD_STRAIGHT_TARGET_CDEG` | CD 直线目标航向 |
| `TASK2_AB_*` | AB 直线参数 |
| `TASK2_STRAIGHT_SEARCH_*` | 直线末端找线 |
| `TASK2_ARC_*` | BC/DA 弧线几何、对齐和限时 |

## 任务三

| 宏 | 含义 |
|---|---|
| `TASK3_POINT_ALARM_MS` | 点位声光 |
| `TASK3_ARC_*` | 弧线几何和基础弧线参数 |
| `TASK3_STRAIGHT_*` | 任务三历史兼容直线参数 |
| `RACE_TASK3_AC_HEADING_TARGET_CDEG` | 当前任务三 AC 目标航向 |
| `RACE_TASK3_BD_HEADING_TARGET_CDEG` | 当前任务三 BD 目标航向 |
| `RACE_TASK3_AC_FORCE_TURN_COUNT` | 当前任务三 AC 强制入弯距离 |

## 任务四

| 宏 | 含义 |
|---|---|
| `TASK4_LAP_COUNT` | 圈数，当前为 4 |
| `TASK4_AC_*` / `TASK4_BD_*` | 历史兼容航向和起步参数 |
| `RACE_TASK4_STRAIGHT_BASE_PWM` | 任务四直线基础 PWM |
| `RACE_TASK4_LINE_MAX_PWM` | 任务四直线 PWM 上限 |
| `RACE_TASK4_ARC_BASE_PWM` | 任务四弧线基础 PWM |
| `RACE_TASK4_AC_HEADING_TARGET_CDEG` | 任务四 AC 目标航向 |
| `RACE_TASK4_BD_HEADING_TARGET_CDEG` | 任务四 BD 目标航向 |
| `RACE_TASK4_AC_FORCE_TURN_COUNT` | 任务四 AC 强制入弯距离 |
| `RACE_TASK4_FIRST_AC_FORCE_TURN_COUNT` | 任务四第一圈 AC 强制入弯距离 |
| `RACE_TASK4_BD_POINT_ARM_COUNT` | 任务四 BD 点位检测使能距离 |
| `RACE_TASK4_BD_FORCE_TURN_COUNT` | 任务四 BD 强制入弯距离 |
| `RACE_TASK4_EXIT_TURN_PREDICT_*` | 出弯预测停转 |

## 任务三/四竞速参数

| 宏 | 含义 |
|---|---|
| `RACE_UART_LOG_ENABLE` | 实时文本日志 |
| `RACE_RAM_LOG_ENABLE` | RAM dump |
| `RACE_RAM_*` | RAM 日志容量和窗口 |
| `RACE_DUMP_*` | RAM dump 输出节奏 |
| `RACE_POINT_ALARM_MS` / `RACE_START_ALARM_MS` | 点位和起步声光 |
| `RACE_LINE_*` | 红外弧线巡线控制 |
| `RACE_STRAIGHT_*` | 直线段基础控制 |
| `RACE_ARC_*` | 弧线段基础控制 |
| `RACE_DIFF_*` | 竞速差速闭环 |
| `RACE_IR_*` | 红外入弯/停转掩码 |
| `RACE_LEFT_TURN_*` / `RACE_RIGHT_TURN_*` | 普通入弯转向 PWM |
| `RACE_TASK4_ENTRY_*` / `RACE_TASK4_FORCE_*` | 任务四入弯和强制入弯 PWM |
| `RACE_EXIT_*` | B/A 出弯转向 PWM |
| `RACE_FAST_TURN_*` | 快速入弯通用参数 |
| `RACE_POINT_ADVANCE_*` | 点位后前进交接 |
| `RACE_FORCE_*` | 强制入弯和找线 |

## 编码器方向

| 宏 | 含义 |
|---|---|
| `ENCODER_MOTOR_A_FORWARD_SIGN` | A 轮前进计数符号 |
| `ENCODER_MOTOR_B_FORWARD_SIGN` | B 轮前进计数符号 |

## 调参注意

- 验收前优先保持任务一/二/三稳定参数。
- 任务四提速优先改 `RACE_TASK4_*`。
- 修改距离、航向、PWM 后建议保留 RAM 日志作为回退依据。
- 不要重新加入旧调试任务入口。
