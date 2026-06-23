# app_config 宏定义说明

`app_config.h` 是当前验收版的集中调参入口。文件已按“全局基础、任务一、任务二、任务三、任务四、任务三/四共享竞速、编码器方向”分组；RAM 缓存日志相关宏已经删除。

## 全局基础参数

| 宏 | 含义 |
|---|---|
| `CONTROL_PERIOD_MS` | 默认控制周期 |
| `RACE_TASK4_CONTROL_PERIOD_MS` | 任务四高速控制周期 |
| `ENABLE_JY62_NAV` | 是否启用 JY62 导航 |
| `JY62_*` | JY62 上电置零等待和状态打印周期 |
| `TASK_BUTTON_*` | 任务按键防抖与空闲轮询周期 |
| `ST011_ACTIVE_LOW` | 声光模块触发极性 |
| `ENCODER_TEST_*` | 可选编码器自检参数 |
| `COUNTS_PER_CM` | 编码器距离换算比例 |
| `STRAIGHT_*` | 全局直线基础 PWM、航向辅助和直线 PID |
| `PID_TEST_*` / `PD_TEST_*` | 历史命名保留的共享差速闭环参数 |

## 任务一：A -> B 直线

| 宏 | 含义 |
|---|---|
| `TASK1_START_ALARM_MS` / `TASK1_FINISH_ALARM_MS` | 起点和终点声光提示 |
| `TASK1_START_SETTLE_MS` / `TASK1_AFTER_ZERO_DELAY_MS` | 起步置零前后的稳定等待 |
| `TASK1_START_RAMP_MS` / `TASK1_RAMP_*_START_PWM` | 起步斜坡 |
| `TASK1_REPORT_PERIOD_MS` | 直线段串口诊断周期 |
| `TASK1_B_LINE_ARM_COUNT` | B 点到线检测使能距离 |
| `TASK1_FORCE_STOP_COUNT` | 任务一距离保护 |
| `TASK1_STOP_MIN_IR_COUNT` | 终点线最少命中探头数 |
| `TASK1_APPROACH_*` | 接近终点的保守速度参数 |
| `TASK1_DISTANCE_CORR_*` | 编码器距离修正 |
| `TASK1_HEADING_*` | JY62 航向修正、滤波、死区和晃动门控 |

## 任务二：A -> B -> C -> D -> A

| 宏 | 含义 |
|---|---|
| `TASK2_POINT_ALARM_MS` | 任务二点位声光提示 |
| `TASK2_CD_STRAIGHT_TARGET_CDEG` | CD 直线固定航向目标 |
| `TASK2_CD_HEADING_*` | CD 固定航向修正 |
| `TASK2_STRAIGHT_SEARCH_START_COUNT` | CD 直线到线使能距离，用于避开 C 点出弯后的残留线形 |
| `TASK3_*` / `RACE_*` | 任务二 BC/DA 弧线复用任务三/四竞速弧线控制参数 |

## 任务三：单圈竞速

| 宏 | 含义 |
|---|---|
| `TASK3_POINT_ALARM_MS` | 任务三点位声光提示 |
| `TASK3_ARC_*` | 任务三弧线几何、方向、入弧和弧线诊断周期 |
| `TASK3_STRAIGHT_*` | 任务三历史兼容直线参数 |
| `RACE_TASK3_START_*` | 任务三起跑对齐动作 |
| `RACE_TASK3_AC_HEADING_TARGET_CDEG` | 当前任务三 AC 目标航向 |
| `RACE_TASK3_BD_HEADING_TARGET_CDEG` | 当前任务三 BD 目标航向 |
| `RACE_TASK3_AC_FORCE_TURN_COUNT` | 任务三 AC 强制入弯距离 |

任务三使用稳定参数；除非明确联调任务三，不要跟随任务四高速调参一起修改。

## 任务四：四圈竞速

| 宏 | 含义 |
|---|---|
| `TASK4_LAP_COUNT` | 圈数，当前为 4 |
| `TASK4_*` | 历史兼容字段，当前实际控制优先看 `RACE_TASK4_*` |
| `RACE_TASK4_STRAIGHT_BASE_PWM` | 任务四直线基础 PWM |
| `RACE_TASK4_LINE_MAX_PWM` | 任务四直线 PWM 上限 |
| `RACE_TASK4_FIRST_AC_RAMP_*` | 任务四第一圈 AC 起步斜坡 |
| `RACE_TASK4_ENTRY_DECEL_*` | 任务四直线入弯前减速 |
| `RACE_TASK4_ARC_BASE_PWM` | 任务四弧线基础 PWM |
| `RACE_TASK4_EXIT_DECEL_*` | 任务四弧线出口前减速 |
| `RACE_TASK4_START_*` | 任务四起跑对齐动作 |
| `RACE_TASK4_AC_HEADING_TARGET_CDEG` | 任务四 AC 目标航向 |
| `RACE_TASK4_BD_HEADING_TARGET_CDEG` | 任务四 BD 目标航向 |
| `RACE_TASK4_*FORCE*_COUNT` | 任务四强制入弯距离 |
| `RACE_TASK4_BD_POINT_ARM_COUNT` | 任务四 BD 点位检测使能距离 |
| `RACE_TASK4_EXIT_TURN_PREDICT_*` | B/A 出弯预测停转 |
| `RACE_TASK4_ADVANCE_*` | 弧线出口后短距离前进的航向保持 |
| `RACE_TASK4_ENTRY_*` / `RACE_TASK4_FORCE_*` | 任务四入弯和保护入弯 PWM |

任务四提速优先修改 `RACE_TASK4_*`，避免顺手改变任务三稳定表现。

## 任务三/四共享竞速控制

| 宏 | 含义 |
|---|---|
| `RACE_UART_LOG_ENABLE` | 任务三/四实时文本日志开关，默认关闭 |
| `RACE_POINT_ALARM_MS` / `RACE_START_ALARM_MS` | 竞速点位和起跑声光 |
| `RACE_POINT_SETTLE_MS` / `RACE_TOTAL_MAX_RUN_MS` | 起步稳定等待和总超时 |
| `RACE_LINE_*` | 红外循迹误差滤波、转向、丢线补偿和 PWM 限幅 |
| `RACE_STRAIGHT_*` | 竞速直线基础控制和航向辅助 |
| `RACE_ARC_*` | 竞速弧线基础控制和航向辅助 |
| `RACE_DIFF_*` | 竞速差速闭环参数 |
| `RACE_IR_*` | 红外点位和转向停车掩码 |
| `RACE_LEFT_TURN_*` / `RACE_RIGHT_TURN_*` | 普通入弯转向 PWM |
| `RACE_EXIT_*` | B/A 出弯陀螺仪转向 PWM |
| `RACE_FAST_TURN_*` | 快速转向通用超时、降速和航向停车参数 |
| `RACE_POINT_ADVANCE_*` | 点位后短距离前进 |
| `RACE_*POINT_ARM_COUNT` / `RACE_*FORCE*_COUNT` | 点位使能距离和强制入弯距离 |
| `RACE_FORCE_*` | 强制入弯和找线 |
| `RACE_GYRO_TURN_TIMEOUT_MS` | 陀螺仪定航向转向超时 |

## 编码器方向

| 宏 | 含义 |
|---|---|
| `ENCODER_MOTOR_A_FORWARD_SIGN` | A 轮前进计数符号 |
| `ENCODER_MOTOR_B_FORWARD_SIGN` | B 轮前进计数符号 |

## 调参注意

- 验收前优先保持任务一/二/三稳定参数。
- 任务四提速优先改 `RACE_TASK4_*`。
- 修改距离、航向、PWM 后，建议记录串口输出、视频、手工计时和成功率，方便回退。
- `RACE_UART_LOG_ENABLE` 只建议短时打开定位问题，高速正式跑车时保持关闭。
- 不要重新加入旧调试任务入口。
