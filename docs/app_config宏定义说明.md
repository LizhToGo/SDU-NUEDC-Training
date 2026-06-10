# app_config.h 宏定义说明

本文档说明 `app_config.h` 中参数的实际用途、有效性和整理建议。判断依据不只看源码文本引用，还要结合当前编译配置、任务调度入口、函数调用链和参数是否实际参与控制逻辑。

说明：

| 标记 | 含义 |
| --- | --- |
| 正式任务生效 | 当前 `ENABLE_CONTEST_TASKS=1` 时，任务 1/2/3/4 的实际调用链会用到 |
| 调试任务生效 | 任务 5/6/7/10/11 或测试模式会用到，但不属于正式任务 1/2/3/4 主路径 |
| 条件编译生效 | 只有对应 `ENABLE_*`、`TASK*_LOG_ENABLE`、`TASK*_UART_LOG_ENABLE` 打开时才影响行为 |
| 派生链生效 | 宏本身不在任务中直接出现，但用于计算另一个实际生效参数 |
| 被旧代码引用但当前任务确认不生效（2026-06-08 codegraph 验证） | 宏出现在源码中，但全部位于已确认不可达的旧函数中（0 callers） |
| 当前完全无源码引用 | 当前源码和派生链都没有使用，建议删除或迁移到历史说明文档 |

当前默认路径为：`main()` 在 `ENABLE_CONTEST_TASKS=1` 下进入 `run_task_dispatcher()`。当前实际入口为：任务 1 走 `run_task1_ab()`，任务 2 走 `run_task2_abcd()`，任务 3 走 `run_task11_ir_map_test_laps(1U)`，任务 4 走 `run_task11_ir_map_test_laps(4U)`；任务 5/6/7/10/11 属于调试或专项验证入口。

## 总体情况

2026-06-10 第一轮清理后，`app_config.h` 当前包含约 284 个配置宏。本轮已删除 11 个确认不可达旧函数，并删除 84 个当前源码零引用宏。本文后续若仍出现“可删除/待删除”的历史表述，应以当前源码和本节为准；后续整理时建议继续按调用链复核，而不是只看文本引用。

| 区域 | 作用 | 当前建议 |
| --- | --- | --- |
| 控制周期和功能开关 | 选择主模式、调试模式和导航功能 | 保留，但建议放文件顶部 |
| JY62/IR/编码器/ST011 | 硬件和调试参数 | 保留，硬件相关参数建议单独分区 |
| 直行 PID/PD | 轮速闭环和调试参数 | 保留，建议移入 `config_motion.h` |
| Task1/2/3/4/6/11 | 任务路线和经验参数 | 保留有效项，删除无引用项 |
| RAM log/UART log | 数据记录容量、周期和开关 | 条件有效，建议移入 `config_log.h` |

## 清理前的完全无源码引用宏

以下宏是第一轮清理前记录的无源码引用项。2026-06-10 已按当前源码重新计算并完成删除，本节保留为清理依据，不代表这些宏仍存在于 `app_config.h`。

| 宏 | 当前值 | 说明 | 建议 |
| --- | --- | --- | --- |
| `JY62_IDLE_REPORT_PERIOD_MS` | `500` | 空闲状态 JY62 打印周期，当前没有空闲打印逻辑引用 | 删除或重新接入 dispatcher 空闲打印 |
| `TASK2_CD_HEADING_TARGET_CDEG` | `17950` | Task2 CD 航向目标，当前疑似被 `TASK2_CD_STRAIGHT_TARGET_CDEG` 替代 | 删除前确认实车是否还需要 |
| `TASK2_ARC_BASE_PWM` | `485` | 早期 Task2 弧线基础 PWM | 可删除或合并到当前弧线配置 |
| `TASK2_ARC_MIN_BASE_PWM` | `325` | 早期 Task2 弧线最低基础 PWM | 可删除 |
| `TASK2_ARC_KP_DIVISOR` | `18` | 早期 Task2 弧线红外 P 分母 | 可删除 |
| `TASK2_ARC_KD_DIVISOR` | `6` | 早期 Task2 弧线红外 D 分母 | 可删除 |
| `TASK2_ARC_TURN_SLEW_STEP` | `45` | 早期转向斜率限制 | 可删除 |
| `TASK2_ARC_SLOW_ERROR_DIVISOR` | `18` | 早期慢速区误差降速参数 | 可删除 |
| `TASK2_ARC_SLOW_DERIV_DIVISOR` | `10` | 早期慢速区导数降速参数 | 可删除 |
| `TASK2_ARC_TURN_LIMIT` | `260` | 早期 Task2 弧线转向限幅 | 可删除 |
| `TASK2_ARC_YAW_DONE_CDEG` | `17500` | 早期弧线完成角度 | 可删除 |
| `TASK2_BC_FINISH_COUNT` | `TASK2_ARC_FINISH_COUNT - TASK2_BC_FINISH_TRIM_COUNT` | BC 完成距离 | 可删除 |
| `TASK2_BC_YAW_DONE_CDEG` | `16600` | BC 完成角度 | 可删除 |
| `TASK2_ARC_MIN_DISTANCE_COUNT` | `TASK2_ARC_EXIT_ARM_COUNT` | 弧线最小距离 | 可删除 |
| `TASK2_ARC_LOST_STOP_COUNT` | `3` | 丢线停止计数 | 可删除 |
| `TASK2_ARC_ALIGN_STABLE_COUNT` | `4` | 弧线后对准稳定计数 | 可删除，当前对准逻辑没有用稳定计数 |
| `TASK3_ARC_MIN_DISTANCE_COUNT` | `TASK3_ARC_LENGTH_COUNT * 3 / 5` | Task3 弧线最小距离 | 可删除 |
| `TASK3_BAC_THEORY_CDEG` | `3866` | 几何角度理论值 | 可移到文档 |
| `TASK3_AC_TANGENT_CORR_CDEG` | `4339` | AC 切线修正角 | 可移到文档 |
| `TASK3_BD_RELATIVE_TURN_CDEG` | `6600` | BD 相对转角 | 可移到文档 |
| `TASK11_STRAIGHT_POINT_ARM_COUNT` | `8600` | Task11 直线点位 arm count，当前使用 AC/BD 独立参数 | 可删除 |
| `TASK11_C_ENTRY_TURN_CDEG` | `3200` | C 点入弯转角，当前无引用 | 可删除或接入 Task11 转向流程 |
| `TASK11_D_ENTRY_TURN_CDEG` | `-3200` | D 点入弯转角，当前无引用 | 可删除或接入 Task11 转向流程 |

## 仅在派生表达式中使用的宏

以下宏没有在 `main.c`、`app_*.h`、`BSP/`、`Board/` 中直接引用，但仍被 `app_config.h` 内其它宏使用。它们不应按“无引用宏”直接删除，除非同步改写依赖它们的派生公式。

| 宏 | 当前值 | 依赖关系 | 建议 |
| --- | --- | --- | --- |
| `TASK2_ARC_EXIT_EXTRA_COUNT` | `((TASK2_ARC_SENSOR_TO_AXIS_MM * COUNTS_PER_CM + 5) / 10)` | 用于 `TASK2_ARC_EXIT_ARM_COUNT`、`TASK2_ARC_FINISH_COUNT` | 保留，说明其为传感器到车轴距离补偿 |
| `TASK2_ARC_FINISH_COUNT` | `TASK2_ARC_LENGTH_COUNT + TASK2_ARC_EXIT_EXTRA_COUNT` | 用于 `TASK2_ARC_FORCE_STOP_COUNT`，也被当前无引用的 `TASK2_BC_FINISH_COUNT` 使用 | 保留或将公式内联后再删除 |
| `TASK2_BC_FINISH_TRIM_COUNT` | `650` | 只用于当前无引用的 `TASK2_BC_FINISH_COUNT` | 与 `TASK2_BC_FINISH_COUNT` 一起复核 |
| `TASK3_ARC_RADIUS_CM` | `40` | 用于 `TASK3_ARC_LENGTH_COUNT` | 保留，属于 Task3 弧线几何基础参数 |
| `TASK3_ARC_EXIT_EXTRA_COUNT` | `((TASK3_SENSOR_TO_AXIS_MM * COUNTS_PER_CM + 5) / 10)` | 用于 `TASK3_ARC_FINISH_COUNT` | 保留，说明其为出口补偿 |

这一类宏更适合标记为“派生中间量”，而不是删除。若后续想进一步减小配置噪声，可以把几何公式集中到一个小节，并把中间量的注释写清楚。

## 清理前被旧代码引用但当前任务确认不生效的参数（2026-06-08 codegraph 验证）

以下参数是第一轮清理前的记录。2026-06-10 已删除对应旧函数，并按当前源码删除了 84 个零引用宏。本节保留为历史依据，不代表这些宏仍存在于 `app_config.h`。

注：此节原文档标记为“疑似不生效”，2026-06-08 经 codegraph 调用链验证后升级为“确认不生效”。验证依据见 `项目整理建议.md` 中的调用链分析。

| 参数或参数组 | 当前主要出现位置 | 当前判断 | 建议 |
| --- | --- | --- | --- |
| `TASK2_ARC_RADIUS_MM`、`TASK2_ARC_WHEEL_BASE_MM`、`TASK2_ARC_CENTER_SPEED_COUNT`、`TASK2_ARC_TARGET_DIFF_MAX`、`TASK2_ARC_DIFF_FF_GAIN_X10` | `task2_arc_model_target_diff()`、`run_arc_line_follow_segment()` | 被旧 Task2 弧线模型引用，**确认不生效** | Task2 已确认改走 `run_task2_task11_arc_segment()`，**可删除** |
| `TASK2_ARC_PID_I_LIMIT`、`TASK2_ARC_PID_CORR_MAX`、`TASK2_ARC_B_BASE_PWM`、`TASK2_ARC_A_BASE_PWM`、`TASK2_ARC_MIN_PWM`、`TASK2_ARC_MAX_PWM` | `run_arc_line_follow_segment()` | 被旧 Task2 弧线控制引用，**确认不生效** | `run_arc_line_follow_segment()` 已确认 0 callers，**可删除** |
| `TASK2_ARC_ERROR_DEADBAND`、`TASK2_ARC_ERROR_FILTER_DIVISOR`、`TASK2_ARC_DERIVATIVE_LIMIT`、`TASK2_ARC_IR_KP_DIVISOR`、`TASK2_ARC_IR_KD_DIVISOR`、`TASK2_ARC_IR_CORR_MAX` | `run_arc_line_follow_segment()` | 被旧 Task2 弧线红外辅助引用，**确认不生效** | 旧弧线函数已确认不可达，**可删除** |
| `TASK2_ARC_YAW_CORR_DIVISOR`、`TASK2_ARC_YAW_CORR_MAX`、`TASK2_ARC_HEAD_EXIT_CDEG`、`TASK2_ARC_HEAD_EXIT_TOL_CDEG`、`TASK2_ARC_ALIGN_TARGET_CDEG` | `run_arc_line_follow_segment()`、`task2_arc_yaw_*()` | 被旧 Task2 弧线航向/出口判断引用，**确认不生效** | 旧函数已确认不可达，**可删除** |
| `TASK3_ALIGN_TOL_CDEG`、`TASK3_ALIGN_STABLE_COUNT`、`TASK3_ALIGN_TIMEOUT_MS`、`TASK3_ALIGN_FAST_PWM`、`TASK3_ALIGN_SLOW_PWM`、`TASK3_ALIGN_SLOW_ZONE_CDEG` | `run_task3_yaw_align_segment()` | 被旧 Task3 航向对准段引用，**确认不生效** | Task3/Task4 已确认不再调用该对准段，**可删除** |
| 旧 `TASK3_ARC_*` 红外弧线参数 | `run_task3_arc_line_follow_segment()` | 被旧 Task3 红外弧线段引用，**确认不生效** | Task3/Task4 已确认改走 Task11 风格弧线段，**可删除** |
| `TASK11_ALIGN_TOL_CDEG`、`TASK11_ALIGN_STABLE_COUNT`、`TASK11_ALIGN_TIMEOUT_MS`、`TASK11_ALIGN_FAST_PWM`、`TASK11_ALIGN_SLOW_PWM`、`TASK11_ALIGN_SLOW_ZONE_CDEG`、`TASK11_ALIGN_GZLP_TOL_MDPS` | `task11_align_to_yaw()`、`task11_align_relative()` | 被旧 Task11 对准接口引用，**确认不生效** | Task11 已确认不再做该对准流程，**可删除** |
| `TASK11_LINE_*` 纯红外线段参数 | `task11_run_ir_line_segment()` | 被旧 Task11 纯红外线段引用，**确认不生效** | Task11 已确认走差速/航向融合路径，**可删除** |

上表中的“确认不生效”是基于 codegraph 验证的结论：这些参数被旧代码引用，而旧代码已确认为 0 callers。删除旧函数后，这些参数将变成零引用宏，可一并删除。

## 功能开关

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `ENABLE_IR_TRACKING_UART_TEST` | `0` | 条件有效 | 进入红外模块串口打印测试模式 |
| `ENABLE_CONTEST_TASKS` | `1` | 有效 | 进入正式任务调度器，是当前默认主模式 |
| `ENABLE_LINE_FOLLOW_TEST` | `0` | 条件有效 | 进入纯红外循迹测试模式 |
| `ENABLE_JY62_NAV` | `1` | 有效 | 编译 JY62 初始化、导航读取和 UART1 中断 |
| `ENABLE_ENCODER_SELF_TEST` | `0` | 条件有效 | 在非正式任务模式下启用编码器/电机自检 |

当前主流程由 `ENABLE_CONTEST_TASKS` 控制。若该宏为 1，`main()` 会调用 `run_task_dispatcher()`；其它测试模式不会进入。

## 通用控制周期

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `CONTROL_PERIOD_MS` | `20` | 有效 | 主控制循环周期。直行、弧线、Task11 线段、调试任务和编码器速度周期都依赖它 |

该参数会同时影响控制响应、编码器速度差分、日志周期累加和停止检测频率，不建议随意修改。

## 直行基础 PWM

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `STRAIGHT_B_BASE_PWM` | `628` | 有效 | B 电机，即当前左轮，直行基础 PWM |
| `STRAIGHT_A_BASE_PWM` | `633` | 有效 | A 电机，即当前右轮，直行基础 PWM |
| `STRAIGHT_MIN_PWM` | `0` | 有效 | 直行闭环输出最小 PWM |
| `STRAIGHT_MAX_PWM` | `870` | 有效 | 直行闭环输出最大 PWM |

注释中说明“左轮是 B 电机，右轮是 A 电机”。这几个值影响 `app_straight.h` 的 PID/PD 调试配置，也在 `main.c` 任务段中直接使用。

## Task5 / PID 调试日志

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `PID_REPORT_PERIOD_MS` | `100` | 条件有效 | 轮速 PID/PD 调试串口打印周期 |
| `TASK5_RAM_LOG_ENABLE` | `0` | 条件有效 | 打开 Task5 RAM log 编译路径 |
| `TASK5_RAM_LOG_CAPACITY` | `384` | 条件有效 | Task5 RAM log 样本容量 |
| `TASK5_RAM_LOG_PERIOD_MS` | `40` | 条件有效 | Task5 RAM log 采样周期 |
| `TASK5_DUMP_LINE_DELAY_MS` | `100` | 条件有效 | Dump 每行之间的延时，避免串口拥塞 |
| `TASK5_YAW_CORR_ENABLE` | `1` | 有效/条件有效 | 允许 Task5 直行调试加入航向修正 |
| `TASK5_YAW_DEADBAND_CDEG` | `20` | 有效/条件有效 | Task5 航向误差死区，单位 centi-degree |
| `TASK5_YAW_CORR_DIVISOR` | `30` | 有效/条件有效 | Task5 航向误差转修正量的分母 |
| `TASK5_YAW_CORR_MAX` | `18` | 有效/条件有效 | Task5 航向修正限幅 |
| `TASK5_YAW_GYRO_DAMP_DIVISOR` | `2600` | 有效/条件有效 | 根据陀螺 Z 轴角速度抑制修正 |

即使 `TASK5_RAM_LOG_ENABLE` 为 0，Task5 航向修正参数仍可能通过 `app_debug_modes.h` 的 `task5_yaw_correction()` 影响直行段。

## JY62 参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `JY62_BOOT_ZERO_DELAY_MS` | `300` | 有效 | 上电后等待 JY62 稳定并置零前的延时 |
| `JY62_IDLE_REPORT_PERIOD_MS` | `500` | 当前无引用 | 预留的空闲状态打印周期 |
| `JY62_TASK_REPORT_PERIOD_MS` | `500` | 有效 | 任务运行中 JY62 状态打印周期 |

`ENABLE_JY62_NAV=1` 时，`main()` 会初始化 JY62 并开启 UART1 中断。

## 红外与纯循迹调试

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `IR_TRACKING_TEST_PERIOD_MS` | `100` | 条件有效 | 红外 UART 打印测试周期 |
| `LINE_FOLLOW_PERIOD_MS` | `20` | 条件有效 | 纯红外循迹调试控制周期 |
| `LINE_FOLLOW_REPORT_PERIOD_MS` | `300` | 条件有效 | 纯红外循迹调试打印周期 |
| `LINE_FOLLOW_BASE_PWM` | `480` | 条件有效 | 纯红外循迹基础 PWM |
| `LINE_FOLLOW_MIN_PWM` | `0` | 条件有效 | 纯红外循迹最小 PWM |
| `LINE_FOLLOW_MAX_PWM` | `820` | 条件有效 | 纯红外循迹最大 PWM |
| `LINE_FOLLOW_TURN_DIVISOR` | `11` | 条件有效 | 红外误差转向分母 |
| `LINE_FOLLOW_TURN_LIMIT` | `420` | 条件有效 | 纯红外循迹转向限幅 |
| `LINE_FOLLOW_TURN_SIGN` | `1` | 条件有效 | 红外循迹转向方向符号 |

这些宏主要服务 `ENABLE_LINE_FOLLOW_TEST` 和 `app_debug_modes.h`。

## 任务按键与 Task1 参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK_BUTTON_DEBOUNCE_MS` | `30` | 有效 | 实体任务按键去抖时间 |
| `TASK_BUTTON_IDLE_MS` | `10` | 有效 | 等待任务命令时的轮询间隔 |
| `TASK1_START_ALARM_MS` | `120` | 有效 | Task1/部分任务启动提示脉冲 |
| `TASK1_FINISH_ALARM_MS` | `120` | 有效 | Task1/部分段完成提示脉冲 |
| `TASK1_START_SETTLE_MS` | `250` | 有效 | 启动提示后等待车体稳定 |
| `TASK1_AFTER_ZERO_DELAY_MS` | `100` | 有效 | JY62 置零后的额外等待 |
| `TASK1_START_RAMP_MS` | `400` | 有效 | 起步 PWM 从 ramp 值过渡到目标值的时间 |
| `TASK1_RAMP_B_START_PWM` | `560` | 有效 | B 电机起步 ramp PWM |
| `TASK1_RAMP_A_START_PWM` | `600` | 有效 | A 电机起步 ramp PWM |
| `TASK1_REPORT_PERIOD_MS` | `100` | 有效 | Task1/直行段打印周期 |
| `TASK1_MAX_RUN_MS` | `15000` | 有效 | 直行段最大运行时间 |
| `TASK1_B_LINE_ARM_COUNT` | `6000` | 有效 | 到达该距离后才允许红外线触发停车 |
| `TASK1_APPROACH_SLOW_COUNT` | `5000` | 有效 | 接近终点时降速距离阈值 |
| `TASK1_APPROACH_B_BASE_PWM` | `620` | 有效 | 接近终点时 B 电机基础 PWM |
| `TASK1_APPROACH_A_BASE_PWM` | `630` | 有效 | 接近终点时 A 电机基础 PWM |
| `TASK1_FORCE_STOP_COUNT` | `9500` | 有效 | 直行强制停车距离 |
| `TASK1_STOP_MIN_IR_COUNT` | `1` | 有效 | 触发停车所需最小黑线探头数 |

## Task1 航向和距离修正

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK1_DISTANCE_CORR_DIVISOR` | `16` | 有效 | 左右编码器距离差转修正量分母 |
| `TASK1_DISTANCE_CORR_MAX` | `45` | 有效 | 距离差修正限幅 |
| `TASK1_HEADING_CORR_DIVISOR` | `12` | 有效 | 航向误差转修正量分母 |
| `TASK1_HEADING_CORR_MAX` | `90` | 有效 | 航向修正限幅 |
| `TASK1_HEADING_CORR_SIGN` | `-1` | 有效 | 航向修正方向符号 |
| `TASK1_HEADING_FILTER_DIVISOR` | `5` | 有效 | 航向一阶低通分母 |
| `TASK1_HEADING_WOBBLE_FILTER_DIVISOR` | `8` | 有效 | 晃动时更慢低通分母 |
| `TASK1_HEADING_DEADBAND_CDEG` | `60` | 有效 | 航向误差死区 |
| `TASK1_HEADING_GYRO_GATE_START_MDPS` | `30000` | 有效 | 开始降低航向修正增益的角速度阈值 |
| `TASK1_HEADING_GYRO_GATE_END_MDPS` | `80000` | 有效 | 完全压制航向修正的角速度阈值 |
| `TASK1_HEADING_PRIORITY_CDEG` | `250` | 有效 | 航向优先控制阈值 |
| `TASK1_HEADING_PRIORITY_MAX_VERR` | `18` | 有效 | 航向优先时允许的最大速度误差 |
| `TASK1_HEADING_PRIORITY_MAX_DERR` | `240` | 有效 | 航向优先时允许的最大距离误差 |

这些宏和 `app_control.h` 的 `heading_filter_update()`、直行段控制逻辑关联较强。

## Task2 参数

Task2 当前混合了 A-B 直行、B-C 弧线、C-D 直行、D-A 弧线，以及 Task2 RAM log。

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK2_POINT_ALARM_MS` | `120` | 有效 | Task2 点位提示脉冲 |
| `TASK2_RAM_LOG_ENABLE` | `1` | 条件有效 | 打开 Task2 RAM log |
| `TASK2_RAM_SAMPLE_CAPACITY` | `192` | 条件有效 | Task2 样本日志容量 |
| `TASK2_RAM_EVENT_CAPACITY` | `20` | 条件有效 | Task2 事件日志容量 |
| `TASK2_DUMP_LINE_DELAY_MS` | `40` | 条件有效 | Task2 dump 行间延时 |
| `TASK2_STRAIGHT_RAM_LOG_PERIOD_MS` | `40` | 条件有效 | Task2 直行段 RAM log 周期 |
| `TASK2_CD_STRAIGHT_TARGET_CDEG` | `18150` | 有效 | Task2 CD 直行固定航向目标 |
| `TASK2_CD_HEADING_CORR_DIVISOR` | `5` | 有效 | CD 航向修正分母 |
| `TASK2_CD_HEADING_CORR_MAX` | `160` | 有效 | CD 航向修正限幅 |
| `TASK2_CD_HEADING_GYRO_DAMP_DIVISOR` | `700` | 有效 | CD 航向修正角速度阻尼 |
| `COUNTS_PER_CM` | `66` | 有效 | 编码器计数到厘米的换算系数，Task2/Task3 派生距离依赖 |
| `TASK2_ARC_RADIUS_CM` | `40` | 有效 | Task2 弧线半径，参与弧长计算 |
| `TASK2_ARC_RADIUS_MM` | `400` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线模型半径毫米值，**可删除** |
| `TASK2_ARC_WHEEL_BASE_MM` | `126` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线模型车轮间距，**可删除** |
| `TASK2_ARC_SENSOR_TO_AXIS_MM` | `175` | 有效 | 红外传感器到车轴距离 |
| `TASK2_ARC_LENGTH_COUNT` | 派生 | 有效 | 半圆弧理论编码器计数 |
| `TASK2_ARC_EXIT_ARM_COUNT` | 派生 | 有效 | 弧线出口判定开始距离 |
| `TASK2_ARC_FORCE_STOP_COUNT` | 派生 | 有效 | 弧线强制停止距离 |
| `TASK2_ARC_CENTER_SPEED_COUNT` | `90` | 被旧代码引用但当前任务确认不生效 | 旧弧线模型中心速度计数，**可删除** |
| `TASK2_ARC_TARGET_DIFF_MAX` | `48` | 被旧代码引用但当前任务确认不生效 | 旧弧线目标差速上限，**可删除** |
| `TASK2_ARC_DIFF_FF_GAIN_X10` | `31` | 被旧代码引用但当前任务确认不生效 | 旧弧线差速前馈增益，**可删除** |
| `TASK2_ARC_PID_I_LIMIT` | `700` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线 PID 积分限幅，**可删除** |
| `TASK2_ARC_PID_CORR_MAX` | `190` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线 PID 修正限幅，**可删除** |
| `TASK2_ARC_B_BASE_PWM` | `485` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线 B 电机基础 PWM，**可删除** |
| `TASK2_ARC_A_BASE_PWM` | `553` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线 A 电机基础 PWM，**可删除** |
| `TASK2_ARC_MIN_PWM` | `0` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线最小 PWM，**可删除** |
| `TASK2_ARC_MAX_PWM` | `650` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线最大 PWM，**可删除** |
| `TASK2_ARC_ERROR_DEADBAND` | `180` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线红外误差死区，**可删除** |
| `TASK2_ARC_ERROR_FILTER_DIVISOR` | `3` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线红外误差滤波分母，**可删除** |
| `TASK2_ARC_DERIVATIVE_LIMIT` | `450` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线导数限幅，**可删除** |
| `TASK2_ARC_IR_KP_DIVISOR` | `260` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线红外辅助 P 分母，**可删除** |
| `TASK2_ARC_IR_KD_DIVISOR` | `90` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线红外辅助 D 分母，**可删除** |
| `TASK2_ARC_IR_CORR_MAX` | `18` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线红外辅助修正限幅，**可删除** |
| `TASK2_ARC_YAW_CORR_DIVISOR` | `220` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线航向误差修正分母，**可删除** |
| `TASK2_ARC_YAW_CORR_MAX` | `14` | 被旧代码引用但当前任务确认不生效 | 旧 Task2 弧线航向修正限幅，**可删除** |
| `TASK2_ARC_HEAD_EXIT_CDEG` | `15600` | 被旧代码引用但当前任务确认不生效 | 旧路径允许按航向进入出口判定的角度，**可删除** |
| `TASK2_ARC_HEAD_EXIT_TOL_CDEG` | `1200` | 被旧代码引用但当前任务确认不生效 | 旧路径出口航向容差，**可删除** |
| `TASK2_ARC_ALIGN_TARGET_CDEG` | `17000` | 被旧代码引用但当前任务确认不生效 | 旧弧线后对准目标转角，**可删除** |
| `TASK2_ARC_ALIGN_TOL_CDEG` | `200` | 有效 | 弧线后对准容差 |
| `TASK2_ARC_ALIGN_TIMEOUT_MS` | `2500` | 有效 | 弧线后对准超时 |
| `TASK2_ARC_ALIGN_INNER_PWM` | `400` | 有效 | 对准时内侧轮 PWM |
| `TASK2_ARC_ALIGN_OUTER_PWM` | `460` | 有效 | 对准时外侧轮 PWM |
| `TASK2_ARC_MAX_RUN_MS` | `12000` | 有效 | Task2 弧线段最大运行时间 |
| `TASK2_ARC_REPORT_PERIOD_MS` | `40` | 有效 | Task2 弧线打印周期 |
| `TASK2_AB_DISTANCE_CORR_DIVISOR` | `20` | 有效 | AB 直行距离修正分母 |
| `TASK2_AB_DISTANCE_CORR_MAX` | `65` | 有效 | AB 直行距离修正限幅 |
| `TASK2_AB_HEADING_CORR_DIVISOR` | `5` | 有效 | AB 直行航向修正分母 |
| `TASK2_AB_HEADING_CORR_MAX` | `160` | 有效 | AB 直行航向修正限幅 |
| `TASK2_AB_HEADING_DEADBAND_CDEG` | `25` | 有效 | AB 直行航向死区 |
| `TASK2_AB_BIAS_CORRECTION` | `-10` | 有效 | AB 直行经验偏置修正 |
| `TASK2_AB_STOP_MASK` | `0xFFU` | 有效 | AB 终点红外掩码 |
| `TASK2_AB_STOP_ERROR_MAX` | `4000` | 有效 | AB 终点允许的最大红外误差 |
| `TASK2_CD_STOP_CENTER_MASK` | `0xFFU` | 有效 | CD 终点红外掩码 |
| `TASK2_CD_STOP_ERROR_MAX` | `4000` | 有效 | CD 终点允许的最大红外误差 |
| `TASK2_STRAIGHT_SEARCH_*` | 多项 | 有效 | Task2 直线末端搜索线策略，包括开始距离、扫线修正、基础降速 |

Task2 无引用项已在“当前完全无源码引用宏”章节列出。`TASK2_ARC_EXIT_EXTRA_COUNT`、`TASK2_ARC_FINISH_COUNT`、`TASK2_BC_FINISH_TRIM_COUNT` 属于派生表达式相关参数，删除时必须同步检查 `TASK2_ARC_EXIT_ARM_COUNT`、`TASK2_ARC_FORCE_STOP_COUNT` 和 `TASK2_BC_FINISH_COUNT`。另外，许多 `TASK2_ARC_*` 参数虽然有引用，但引用集中在旧弧线函数中，是否实际影响当前 Task2 需要以任务调用链为准。

## ST011 声光模块

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `ST011_ACTIVE_LOW` | `1` | 有效 | 声光模块低电平触发；`1` 表示拉低触发、拉高空闲 |

该宏影响 `st011_set_active()` 中 GPIO 置位/清零方向。

## 编码器自检和方向

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `ENCODER_TEST_PWM` | `260` | 条件有效 | 编码器/电机自检时的测试 PWM |
| `ENCODER_TEST_MS` | `500` | 条件有效 | 自检单轮运行时间 |
| `ENCODER_MIN_PULSE` | `2` | 条件有效 | 自检认为编码器有效的最小脉冲数 |
| `ENCODER_MOTOR_A_FORWARD_SIGN` | `-1` | 有效 | 将 A 电机前进方向计数归一为正 |
| `ENCODER_MOTOR_B_FORWARD_SIGN` | `1` | 有效 | 将 B 电机前进方向计数归一为正 |

方向符号是硬件标定项，修改后会影响所有基于编码器的距离和速度判断。

## 直行 PID / PD 参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `STRAIGHT_PID_SCALE` | `20` | 有效 | 直行 PID 定点缩放分母 |
| `STRAIGHT_PID_KP` | `22` | 有效 | 直行速度差 PID 的 P 系数 |
| `STRAIGHT_PID_KI` | `2` | 有效 | 直行速度差 PID 的 I 系数 |
| `STRAIGHT_PID_KD` | `6` | 有效 | 直行速度差 PID 的 D 系数 |
| `STRAIGHT_I_LIMIT` | `180` | 有效 | 默认积分限幅 |
| `STRAIGHT_CORR_MAX` | `140` | 有效 | 默认修正限幅 |
| `STRAIGHT_TARGET_SPEED_DIFF` | `0` | 有效 | 默认目标速度差，`B_spd - A_spd` |
| `PID_TEST_TARGET_SPEED_DIFF` | `0` | 条件有效 | 05 PID 调试目标速度差 |
| `PID_TEST_I_LIMIT` | `600` | 条件有效 | 05 PID 调试积分限幅 |
| `PID_TEST_CORR_MAX` | `180` | 条件有效 | 05 PID 调试修正限幅 |
| `PID_TEST_DIFF_FF_GAIN` | `3` | 条件有效 | 05 PID 调试差速前馈增益 |
| `PID_TEST_DISTANCE_CORR_DIVISOR` | `9` | 条件有效 | 05 PID 调试距离差修正分母 |
| `PID_TEST_DISTANCE_CORR_MAX` | `45` | 条件有效 | 05 PID 调试距离差修正限幅 |
| `PD_TEST_TARGET_SPEED_DIFF` | `20` | 条件有效 | 07 PD 调试目标速度差 |
| `PD_TEST_KP` | `22` | 条件有效 | 07/Task11 差速 P 系数来源 |
| `PD_TEST_KD` | `6` | 条件有效 | 07/Task11 差速 D 系数来源 |
| `PD_TEST_CORR_MAX` | `180` | 条件有效 | 07/Task11 差速修正限幅来源 |
| `PD_TEST_DIFF_FF_GAIN` | `3` | 条件有效 | 07 PD 调试差速前馈增益 |
| `PD_TEST_DISTANCE_CORR_DIVISOR` | `12` | 条件有效 | 07 PD 调试距离差修正分母 |
| `PD_TEST_DISTANCE_CORR_MAX` | `0` | 条件有效 | 07 PD 调试距离差修正限幅，当前为 0 表示不参与 |

## Task3 参数

Task3 是 A-C 直线、C-B 红外弧线、B-D 直线、D-A 红外弧线的组合。

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK3_POINT_ALARM_MS` | `120` | 有效 | Task3 点位提示脉冲 |
| `TASK3_SENSOR_TO_AXIS_MM` | `175` | 有效 | 红外传感器到车轴距离 |
| `TASK3_ARC_LENGTH_COUNT` | 派生 | 有效 | Task3 弧线理论距离 |
| `TASK3_ARC_EXIT_IGNORE_COUNT` | `TASK3_ARC_LENGTH_COUNT / 2` | 有效 | 弧线前半段忽略出口线 |
| `TASK3_ARC_FINISH_COUNT` | 派生 | 有效 | Task3 弧线完成距离 |
| `TASK3_ARC_FORCE_STOP_COUNT` | 派生 | 有效 | Task3 弧线强制停止距离 |
| `TASK3_ARC_YAW_DONE_CDEG` | `17000` | 有效 | Task3 弧线完成角度 |
| `TASK3_ARC_EXIT_ARM_CDEG` | `15000` | 有效 | 允许出口判定的角度 |
| `TASK3_ARC_EXIT_CONFIRM_COUNT` | `1` | 有效 | 出口线确认次数 |
| `TASK3_B_EXIT_HEADING_TARGET_CDEG` | `18000` | 有效 | B 点出口航向目标 |
| `TASK3_B_EXIT_HEADING_TOL_CDEG` | `1600` | 有效 | B 点出口航向容差 |
| `TASK3_ARC_TURN_LEFT` | `-1` | 有效 | 左转方向符号 |
| `TASK3_ARC_TURN_RIGHT` | `1` | 有效 | 右转方向符号 |
| `TASK3_AC_HEADING_TARGET_CDEG` | `-3660` | 有效 | AC 直线航向目标 |
| `TASK3_BD_HEADING_TARGET_CDEG` | `-13920` | 有效 | BD 直线航向目标 |
| `TASK3_STRAIGHT_STOP_MASK` | `0xFFU` | 有效 | 直线段停车红外掩码 |
| `TASK3_STRAIGHT_STOP_MIN_IR_COUNT` | `1` | 有效 | 直线段停车所需黑线探头数 |
| `TASK3_STRAIGHT_CORR_MAX` | `155` | 有效 | Task3 直线修正限幅 |
| `TASK3_BD_HEADING_CORR_DIVISOR` | `5` | 有效 | BD 航向修正分母 |
| `TASK3_BD_HEADING_CORR_MAX` | `170` | 有效 | BD 航向修正限幅 |
| `TASK3_STRAIGHT_LINE_ARM_COUNT` | `6100` | 有效 | 直线段开始允许红外停车的距离 |
| `TASK3_STRAIGHT_FORCE_STOP_COUNT` | `11500` | 有效 | 直线段强制停止距离 |
| `TASK3_STRAIGHT_SEARCH_*` | 多项 | 有效 | Task3 直线末端搜索线策略 |
| `TASK3_ALIGN_*` | 多项 | 被旧代码引用但当前任务确认不生效 | 旧 Task3 航向对准参数，`run_task3_yaw_align_segment()` 已确认 0 callers，**可删除** |
| `TASK3_ARC_B_BASE_PWM` | `450` | 有效 | Task3 弧线 B 电机基础 PWM |
| `TASK3_ARC_A_BASE_PWM` | `480` | 有效 | Task3 弧线 A 电机基础 PWM |
| `TASK3_ARC_MIN_PWM` | `0` | 有效 | Task3 弧线最小 PWM |
| `TASK3_ARC_MAX_PWM` | `620` | 有效 | Task3 弧线最大 PWM |
| `TASK3_ARC_ERROR_DEADBAND` | `80` | 有效 | Task3 弧线红外误差死区 |
| `TASK3_ARC_ERROR_FILTER_DIVISOR` | `2` | 有效 | Task3 弧线误差滤波分母 |
| `TASK3_ARC_DERIVATIVE_LIMIT` | `800` | 有效 | Task3 弧线导数限幅 |
| `TASK3_ARC_KP_DIVISOR` | `11` | 有效 | Task3 弧线 P 分母 |
| `TASK3_ARC_KD_DIVISOR` | `6` | 有效 | Task3 弧线 D 分母 |
| `TASK3_ARC_TURN_MAX` | `280` | 有效 | Task3 弧线转向限幅 |
| `TASK3_ARC_ENTRY_*` | 多项 | 有效 | Task3 入弧阶段转向和丢线策略 |
| `TASK3_ARC_LOST_TURN` | `170` | 有效 | 丢线时保持/恢复转向量 |
| `TASK3_ARC_LOST_BASE_DROP` | `70` | 有效 | 丢线时基础 PWM 降低量 |
| `TASK3_ARC_YAW_CORR_DIVISOR` | `220` | 有效 | Task3 弧线航向修正分母 |
| `TASK3_ARC_YAW_CORR_MAX` | `55` | 有效 | Task3 弧线航向修正限幅 |
| `TASK3_ARC_WIDE_LINE_MIN_COUNT` | `6` | 有效 | 宽线/点位判定黑线数量阈值 |
| `TASK3_ARC_FINAL_LINE_MIN_COUNT` | `5` | 有效 | 最终线判定黑线数量阈值 |
| `TASK3_ARC_MAX_RUN_MS` | `12000` | 有效 | Task3 弧线最大运行时间 |
| `TASK3_ARC_REPORT_PERIOD_MS` | `100` | 有效 | Task3 弧线打印周期 |

Task3 中 `TASK3_BAC_THEORY_CDEG`、`TASK3_AC_TANGENT_CORR_CDEG`、`TASK3_BD_RELATIVE_TURN_CDEG` 当前更像计算备忘，不参与逻辑。`TASK3_ARC_RADIUS_CM` 和 `TASK3_ARC_EXIT_EXTRA_COUNT` 虽然没有被业务源码直接引用，但仍参与 `TASK3_ARC_LENGTH_COUNT`、`TASK3_ARC_FINISH_COUNT` 等派生距离计算，应保留或在改写公式后再处理。旧 `run_task3_arc_line_follow_segment()` 已确认不可达（0 callers），其中引用的一批 `TASK3_ARC_*` 参数应从“有效”降级为“确认不生效，待删除”。

## Task4 参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK4_LAP_COUNT` | `4` | 有效 | Task4 圈数 |
| `TASK4_AC_HEADING_TARGET_CDEG` | `TASK3_AC_HEADING_TARGET_CDEG` | 有效 | Task4 AC 航向目标，复用 Task3 |
| `TASK4_BD_HEADING_TARGET_CDEG` | `TASK3_BD_HEADING_TARGET_CDEG` | 有效 | Task4 BD 航向目标，复用 Task3 |
| `TASK4_AC_LINE_SEARCH_PROTECT` | `2` | 有效 | AC 搜线保护次数/策略参数 |
| `TASK4_AC_START_TURN_COUNT` | `3400` | 有效 | AC 起段转向距离阈值 |
| `TASK4_AC_START_BOOST_MIN_ERR_CDEG` | `1200` | 有效 | AC 起段加强修正的最小航向误差 |
| `TASK4_AC_START_HEADING_CORR_DIVISOR` | `3` | 有效 | AC 起段航向修正分母 |
| `TASK4_AC_START_HEADING_CORR_MAX` | `300` | 有效 | AC 起段航向修正限幅 |
| `TASK4_AC_START_CORR_MAX` | `285` | 有效 | AC 起段综合修正限幅 |
| `TASK4_D_TURN_TARGET_CDEG` | `-2200` | 有效 | BD 后 D 点右转目标角度 |
| `TASK4_D_TURN_B_PWM` | `760` | 有效 | D 点右转 B 电机 PWM |
| `TASK4_D_TURN_A_PWM` | `-220` | 有效 | D 点右转 A 电机 PWM |

## Task6 调试转向参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK6_C_TURN_TARGET_CDEG` | `2200` | 有效 | C 点快速左转目标角度 |
| `TASK6_C_TURN_B_PWM` | `-220` | 有效 | C 点快速左转 B 电机 PWM |
| `TASK6_C_TURN_A_PWM` | `760` | 有效 | C 点快速左转 A 电机 PWM |
| `TASK6_C_TURN_TIMEOUT_MS` | `1200` | 有效 | C 点快速转向超时 |
| `TASK6_C_TURN_REPORT_PERIOD_MS` | `40` | 有效 | 快速转向打印周期，也被 Task11 复用 |
| `TASK6_C_TURN_SAMPLE_MAX` | `64` | 有效 | 快速转向采样最大数量 |
| `TASK6_C_TURN_LINE_ARM_CDEG` | `1200` | 有效 | 转过该角度后才允许红外线停车 |
| `TASK6_C_TURN_LINE_STOP_MASK` | `0x7EU` | 有效 | 快速转向停车红外掩码 |
| `TASK6_C_TURN_LINE_STOP_MIN_COUNT` | `1` | 有效 | 快速转向停车所需黑线探头数 |

## Task11 日志参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK11_UART_LOG_ENABLE` | `0` | 条件有效 | 打开 Task11 详细 UART 运行日志 |
| `TASK11_RAM_LOG_ENABLE` | `0` | 条件有效 | 打开 Task11 RAM log 编译路径 |
| `TASK11_RAM_LOG_MAX_LAPS` | `5` | 条件有效 | Task11 RAM log 最大圈数 |
| `TASK11_RAM_WINDOW_CAPACITY` | `560` | 条件有效 | Task11 window log 容量 |
| `TASK11_RAM_EVENT_CAPACITY` | `112` | 条件有效 | Task11 event log 容量 |
| `TASK11_RAM_SUMMARY_CAPACITY` | `24` | 条件有效 | Task11 summary log 容量 |
| `TASK11_RAM_WINDOW_BEFORE_COUNT` | `1800` | 条件有效 | 进入点位前多少距离开始记录 window |
| `TASK11_DUMP_LINE_DELAY_MS` | `100` | 条件有效 | Task11 dump 行间延时 |
| `TASK11_DUMP_SECTION_DELAY_MS` | `1000` | 条件有效 | Task11 dump section 间延时 |
| `TASK11_REALTIME_EVENT_LOG_ENABLE` | `0` | 条件有效 | 打开实时事件日志 |

这些参数当前默认关闭较多，是减小体积和串口占用的关键开关。

## Task11 运动参数

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK11_POINT_ALARM_MS` | `80` | 有效 | Task11 点位提示脉冲 |
| `TASK11_POINT_SETTLE_MS` | `120` | 有效 | 点位动作后稳定等待 |
| `TASK11_TARGET_LAPS` | `5` | 有效 | Task11 目标圈数 |
| `TASK11_TOTAL_MAX_RUN_MS` | `240000` | 有效 | Task11 总超时 |
| `TASK11_LINE_REPORT_PERIOD_MS` | `200` | 有效 | Task11 线段打印周期 |
| `TASK11_LINE_BASE_PWM` | `560` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 纯红外线段基础 PWM，**可删除** |
| `TASK11_LINE_MIN_PWM` | `0` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 线段最小 PWM，**可删除** |
| `TASK11_LINE_MAX_PWM` | `760` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 线段最大 PWM，**可删除** |
| `TASK11_LINE_TURN_DIVISOR` | `9` | 被旧代码引用但当前任务确认不生效 | 旧红外误差转向 P 分母，**可删除** |
| `TASK11_LINE_KD_DIVISOR` | `7` | 被旧代码引用但当前任务确认不生效 | 旧红外误差导数转向分母，**可删除** |
| `TASK11_LINE_DERIV_LIMIT` | `700` | 被旧代码引用但当前任务确认不生效 | 旧红外误差导数限幅，**可删除** |
| `TASK11_LINE_TURN_LIMIT` | `260` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 转向限幅，**可删除** |
| `TASK11_LINE_ERROR_FILTER_DIVISOR` | `2` | 被旧代码引用但当前任务确认不生效 | 旧红外误差滤波分母，**可删除** |
| `TASK11_LINE_LOST_BASE_DROP` | `60` | 被旧代码引用但当前任务确认不生效 | 旧丢线基础 PWM 降低量，**可删除** |
| `TASK11_LINE_LOST_TURN` | `150` | 被旧代码引用但当前任务确认不生效 | 旧丢线维持/恢复转向量，**可删除** |
| `TASK11_STRAIGHT_BASE_PWM` | `600` | 有效 | Task11 直线基础 PWM |
| `TASK11_STRAIGHT_TARGET_DIFF` | `0` | 有效 | Task11 直线目标差速 |
| `TASK11_STRAIGHT_GYRO_NAV_ENABLE` | `1` | 有效 | Task11 直线启用陀螺航向辅助 |
| `TASK11_STRAIGHT_IR_ASSIST_ENABLE` | `0` | 有效 | Task11 直线启用红外辅助 |
| `TASK11_STRAIGHT_HEADING_CORR_DIVISOR` | `8` | 有效 | Task11 直线航向修正分母 |
| `TASK11_STRAIGHT_HEADING_CORR_MAX` | `140` | 有效 | Task11 直线航向修正限幅 |
| `TASK11_STRAIGHT_GYRO_DAMP_DIVISOR` | `1600` | 有效 | Task11 直线角速度阻尼分母 |
| `TASK11_ARC_BASE_PWM` | `540` | 有效 | Task11 弧线基础 PWM |
| `TASK11_ARC_ENTRY_COUNT` | `TASK3_ARC_ENTRY_COUNT` | 有效 | Task11 入弧阶段距离阈值 |
| `TASK11_CB_ARC_ENTRY_TARGET_DIFF` | `-48` | 有效 | CB 入弧目标差速 |
| `TASK11_CB_ARC_CRUISE_TARGET_DIFF` | `-48` | 有效 | CB 巡航目标差速 |
| `TASK11_DA_ARC_ENTRY_TARGET_DIFF` | `46` | 有效 | DA 入弧目标差速 |
| `TASK11_DA_ARC_CRUISE_TARGET_DIFF` | `46` | 有效 | DA 巡航目标差速 |
| `TASK11_ARC_YAW_NAV_ENABLE` | `0` | 有效 | Task11 弧线启用航向辅助 |
| `TASK11_ARC_YAW_CORR_DIVISOR` | `260` | 有效 | Task11 弧线航向修正分母 |
| `TASK11_ARC_YAW_CORR_MAX` | `45` | 有效 | Task11 弧线航向修正限幅 |
| `TASK11_ARC_GYRO_DAMP_DIVISOR` | `4400` | 有效 | Task11 弧线角速度阻尼分母 |
| `TASK11_DIFF_KP` | `PD_TEST_KP` | 有效 | Task11 差速 P 系数 |
| `TASK11_DIFF_KD` | `PD_TEST_KD` | 有效 | Task11 差速 D 系数 |
| `TASK11_DIFF_FF_GAIN` | `2` | 有效 | Task11 差速前馈增益 |
| `TASK11_DIFF_CORR_MAX` | `PD_TEST_CORR_MAX` | 有效 | Task11 差速修正限幅 |
| `TASK11_IR_LEFT_EDGE_MASK` | `0x03U` | 有效 | 左边缘红外掩码 |
| `TASK11_IR_RIGHT_EDGE_MASK` | `0xC0U` | 有效 | 右边缘红外掩码 |
| `TASK11_IR_CENTER_6_MASK` | `0x7EU` | 有效 | 中间 6 路红外掩码 |
| `TASK11_IR_CENTER_6_FORBID_MASK` | `0x81U` | 有效 | 中间 6 路判定时禁止边缘干扰 |
| `TASK11_IR_CENTER_4_MASK` | `0x3CU` | 有效 | 中间 4 路红外掩码 |
| `TASK11_IR_CENTER_4_FORBID_MASK` | `0xC3U` | 有效 | 中间 4 路判定时禁止边缘干扰 |
| `TASK11_IR_TURN_STOP_MIN_COUNT` | `1` | 有效 | 转向停车所需最小黑线探头数 |
| `TASK11_LEFT_TURN_*` | 多项 | 有效 | Task11 左转 PWM，包括快速和慢速 |
| `TASK11_RIGHT_TURN_*` | 多项 | 有效 | Task11 右转 PWM，包括快速和慢速 |
| `TASK11_FAST_TURN_TIMEOUT_MS` | `600` | 有效 | Task11 快速转向超时 |
| `TASK11_FAST_TURN_REPORT_PERIOD_MS` | `TASK6_C_TURN_REPORT_PERIOD_MS` | 有效 | Task11 快速转向打印周期 |
| `TASK11_FAST_TURN_GYRO_SLOW_ENABLE` | `1` | 有效 | 根据陀螺角度进入慢速转向 |
| `TASK11_FAST_TURN_GYRO_SLOW_CDEG` | `2600` | 有效 | 进入慢速转向的角度阈值 |
| `TASK11_EXIT_TURN_YAW_STOP_ENABLE` | `1` | 有效 | 出口转向允许按航向停车 |
| `TASK11_TURN_YAW_STOP_TOL_CDEG` | `260` | 有效 | 出口转向航向停车容差 |
| `TASK11_TURN_YAW_SLOW_ZONE_CDEG` | `900` | 有效 | 航向接近目标时进入慢速区 |
| `TASK11_TURN_YAW_STOP_GZLP_TOL_MDPS` | `14000` | 有效 | 航向停车时角速度容差 |
| `TASK11_B_EXIT_TARGET_CDEG` | `TASK11_BD_HEADING_TARGET_CDEG` | 有效 | B 点出口目标航向 |
| `TASK11_A_EXIT_TARGET_CDEG` | `TASK11_AC_HEADING_TARGET_CDEG` | 有效 | A 点出口目标航向 |
| `TASK11_TURN_CENTER6_ERROR_MAX` | `1500` | 有效 | 转向后中心 6 路允许误差 |
| `TASK11_TURN_CENTER4_ERROR_MAX` | `1000` | 有效 | 转向后中心 4 路允许误差 |
| `TASK11_POINT_ADVANCE_COUNT` | `300` | 有效 | 点位后前进距离 |
| `TASK11_ARC_POINT_ADVANCE_COUNT` | `800` | 有效 | 弧线点位后前进距离 |
| `TASK11_POINT_ADVANCE_PWM` | `360` | 有效 | 点位后前进 PWM |
| `TASK11_POINT_ADVANCE_TIMEOUT_MS` | `800` | 有效 | 点位后前进超时 |
| `TASK11_AC_POINT_ARM_COUNT` | `7300` | 有效 | AC 直线点位 arm count |
| `TASK11_BD_POINT_ARM_COUNT` | `7300` | 有效 | BD 直线点位 arm count |
| `TASK11_STRAIGHT_FORCE_COUNT` | `12800` | 有效 | Task11 直线强制停止距离 |
| `TASK11_STRAIGHT_POINT_ERROR_MIN` | `1200` | 有效 | 直线点位红外误差阈值 |
| `TASK11_STRAIGHT_POINT_WIDE_COUNT` | `3` | 有效 | 直线点位宽线黑线数量阈值 |
| `TASK11_STRAIGHT_POINT_CONFIRM_COUNT` | `1` | 有效 | 直线点位确认次数 |
| `TASK11_ARC_POINT_ARM_COUNT` | `TASK3_ARC_EXIT_IGNORE_COUNT` | 有效 | 弧线点位 arm count |
| `TASK11_ARC_POINT_YAW_ARM_CDEG` | `14000` | 有效 | 弧线点位航向 arm 角度 |
| `TASK11_ARC_FORCE_COUNT` | `TASK3_ARC_FORCE_STOP_COUNT` | 有效 | Task11 弧线强制停止距离 |
| `TASK11_ARC_POINT_WIDE_COUNT` | `TASK3_ARC_WIDE_LINE_MIN_COUNT` | 有效 | 弧线点位宽线黑线数量阈值 |
| `TASK11_ALIGN_TOL_CDEG` | `180` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准航向容差，**可删除** |
| `TASK11_ALIGN_STABLE_COUNT` | `3` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准稳定次数，**可删除** |
| `TASK11_ALIGN_TIMEOUT_MS` | `3000` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准超时，**可删除** |
| `TASK11_ALIGN_FAST_PWM` | `230` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准快速 PWM，**可删除** |
| `TASK11_ALIGN_SLOW_PWM` | `150` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准慢速 PWM，**可删除** |
| `TASK11_ALIGN_SLOW_ZONE_CDEG` | `900` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准慢速区角度，**可删除** |
| `TASK11_ALIGN_GZLP_TOL_MDPS` | `8000` | 被旧代码引用但当前任务确认不生效 | 旧 Task11 对准角速度稳定阈值，**可删除** |


## 2026-06-08 commit 0199d70 新增和变更的宏

以下宏在最近一次提交中新增或发生值变更。

### 新增宏

| 宏 | 当前值 | 状态 | 作用 |
| --- | --- | --- | --- |
| `TASK11_RAM_WINDOW_AFTER_START_COUNT` | `1800` | 条件有效 | Task11 窗口日志在段启动后也记录的距离 |
| `TASK11_STRAIGHT_IR_ASSIST_ENABLE` | `0` | 有效 | Task11 直线段红外辅助开关，当前关闭 |
| `TASK11_STRAIGHT_IR_ASSIST_DIVISOR` | `3` | 有效 | Task11 直线红外辅助转向分母 |
| `TASK11_AC_HEADING_TARGET_CDEG` | `-50` | 有效 | Task11 专用 AC 航向目标（原复用 Task3） |
| `TASK11_BD_HEADING_TARGET_CDEG` | `-10638` | 有效 | Task11 专用 BD 航向目标（原复用 Task3） |
| `TASK11_GYRO_TURN_TIMEOUT_MS` | `1200` | 有效 | 陀螺仪转向超时 |
| `TASK11_EXIT_LEFT_TURN_B_PWM` | `80` | 有效 | B 点出口左转 B 电机 PWM |
| `TASK11_EXIT_LEFT_TURN_A_PWM` | `440` | 有效 | B 点出口左转 A 电机 PWM |
| `TASK11_EXIT_LEFT_TURN_SLOW_B_PWM` | `60` | 有效 | B 点出口慢速左转 B 电机 PWM |
| `TASK11_EXIT_LEFT_TURN_SLOW_A_PWM` | `300` | 有效 | B 点出口慢速左转 A 电机 PWM |
| `TASK11_EXIT_RIGHT_TURN_B_PWM` | `440` | 有效 | A 点出口右转 B 电机 PWM |
| `TASK11_EXIT_RIGHT_TURN_A_PWM` | `80` | 有效 | A 点出口右转 A 电机 PWM |
| `TASK11_EXIT_RIGHT_TURN_SLOW_B_PWM` | `300` | 有效 | A 点出口慢速右转 B 电机 PWM |
| `TASK11_EXIT_RIGHT_TURN_SLOW_A_PWM` | `60` | 有效 | A 点出口慢速右转 A 电机 PWM |

### 功能变更

- **任务三/四入口改走 Task11 路径**：`run_task_dispatcher()` 中任务三改调 `run_task11_ir_map_test_laps(1U)`，任务四改调 `run_task11_ir_map_test_laps(4U)`。不再走 `run_task3_acbda()`/`run_task4_four_laps()`。
- **B/A 点出口转向从红外改为陀螺仪**：`task11_sensor_fast_turn` 改为 `task11_gyro_turn_to_yaw`，使用绝对航向目标停止。
- **Task11 使用独立航向目标**：`TASK11_AC_HEADING_TARGET_CDEG=-50`、`TASK11_BD_HEADING_TARGET_CDEG=-10638`，不再复用 `TASK3_AC/BD_HEADING_TARGET_CDEG`。
- **弧线点位判定简化**：CB/DA 弧线点位从边线+航向判定改为 `line_lost_seen` 判定。
- **直线点位判定放宽**：从 `edge_point_seen` 改为 `line_valid`。
- **新增 `task11_gyro_turn_to_yaw()`**：陀螺仪绝对航向转向函数，支持快/慢速 PWM 和过零检测。
- **任务十改为持续航向监控**：`run_task10_ab_zero_test()` 从单次检查改为 200ms 周期持续打印航向。

## 整理建议

建议将 `app_config.h` 拆成或重排为：

```text
app_config.h              // 只 include 各配置文件，保留全局模式开关
config_hw.h               // 电机接线、编码器方向、ST011、JY62 基础参数
config_motion.h           // CONTROL_PERIOD_MS、直行 PID、line follow 参数
config_task1.h
config_task2.h
config_task3.h
config_task4.h
config_task11.h
config_debug.h            // 05/06/07/10 调试任务参数
config_log.h              // RAM/UART log 开关和容量
```

如果暂时不拆文件，至少建议在 `app_config.h` 中按上述顺序重新分区，并给每组加上“影响的函数/任务”。

## 删除宏前的验证方式

删除“当前无引用宏”后建议执行：

```text
gmake -C Debug clean all
```

如果编译通过，再做最小实车验证：

| 验证项 | 目的 |
| --- | --- |
| 上电串口日志 | 确认主流程和 UART 正常 |
| `00` 停车命令 | 确认任务内停止检测正常 |
| 05/07 调试任务 | 确认直行 PID/PD 配置仍有效 |
| 任务一和任务二 | 覆盖直行、Task2 参数和 Task2 log |
| 任务三/四/十一 | 覆盖 Task3/Task4/Task11 交叉引用参数 |
