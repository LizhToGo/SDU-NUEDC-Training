# app_config.h 宏定义完整说明

所有调参宏集中在 `app_config.h`（359 行）。修改后需重新编译。

---

## 1. 全局控制参数

| 宏 | 说明 | 默认值 | 单位 |
|----|------|--------|------|
| `CONTROL_PERIOD_MS` | 主控制循环周期 | 20 | ms |

---

## 2. 基础直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `STRAIGHT_B_BASE_PWM` | B 轮（左轮）基础 PWM | 628 |
| `STRAIGHT_A_BASE_PWM` | A 轮（右轮）基础 PWM | 633 |
| `STRAIGHT_MIN_PWM` | 最小 PWM 输出 | 0 |
| `STRAIGHT_MAX_PWM` | 最大 PWM 输出 | 870 |

---

## 3. 直行 PID 参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `STRAIGHT_PID_SCALE` | PID 计算缩放因子 | 20 |
| `STRAIGHT_PID_KP` | 比例系数 | 22 |
| `STRAIGHT_PID_KI` | 积分系数 | 2 |
| `STRAIGHT_PID_KD` | 微分系数 | 6 |
| `STRAIGHT_I_LIMIT` | 积分限幅 | 180 |
| `STRAIGHT_CORR_MAX` | 最大修正量 | 140 |
| `STRAIGHT_TARGET_SPEED_DIFF` | 目标速度差 | 0 |

**PID 公式**：`output = (KP*err + KI*integral + KD*d_err) / SCALE`

---

## 4. 功能开关

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `ENABLE_IR_TRACKING_UART_TEST` | 红外串口测试模式 | 0 |
| `ENABLE_CONTEST_TASKS` | 正式任务模式 | 1 |
| `ENABLE_LINE_FOLLOW_TEST` | 纯红外循迹测试 | 0 |
| `ENABLE_JY62_NAV` | JY62 导航使能 | 1 |
| `ENABLE_ENCODER_SELF_TEST` | 编码器自检 | 0 |

---

## 5. JY62 陀螺仪参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `JY62_BOOT_ZERO_DELAY_MS` | 启动置零延时 | 300 ms |
| `JY62_TASK_REPORT_PERIOD_MS` | 状态打印周期 | 500 ms |

---

## 6. 红外模块参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `IR_TRACKING_TEST_PERIOD_MS` | 测试采样周期 | 100 ms |

---

## 7. 纯红外循迹测试参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `LINE_FOLLOW_PERIOD_MS` | 控制周期 | 20 ms |
| `LINE_FOLLOW_REPORT_PERIOD_MS` | 报告周期 | 300 ms |
| `LINE_FOLLOW_BASE_PWM` | 基础 PWM | 480 |
| `LINE_FOLLOW_MIN_PWM` | 最小 PWM | 0 |
| `LINE_FOLLOW_MAX_PWM` | 最大 PWM | 820 |
| `LINE_FOLLOW_TURN_DIVISOR` | 转向除数 | 11 |
| `LINE_FOLLOW_TURN_LIMIT` | 转向限幅 | 420 |
| `LINE_FOLLOW_TURN_SIGN` | 转向符号 | 1 |

---

## 8. 任务按键参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK_BUTTON_DEBOUNCE_MS` | 按键消抖时间 | 30 ms |
| `TASK_BUTTON_IDLE_MS` | 按键空闲时间 | 10 ms |

---

## 9. 任务一参数（A→B 直线）

### 9.1 时序参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK1_START_ALARM_MS` | 启动报警时间 | 120 ms |
| `TASK1_FINISH_ALARM_MS` | 完成报警时间 | 120 ms |
| `TASK1_START_SETTLE_MS` | 启动稳定时间 | 250 ms |
| `TASK1_AFTER_ZERO_DELAY_MS` | 置零后延时 | 100 ms |
| `TASK1_START_RAMP_MS` | 起步斜坡时间 | 400 ms |
| `TASK1_MAX_RUN_MS` | 最大运行时间 | 15000 ms |

### 9.2 起步 PWM

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK1_RAMP_B_START_PWM` | B 轮起步 PWM | 560 |
| `TASK1_RAMP_A_START_PWM` | A 轮起步 PWM | 600 |

### 9.3 距离参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK1_B_LINE_ARM_COUNT` | 线检测使能距离 | 6000 |
| `TASK1_APPROACH_SLOW_COUNT` | 接近减速距离 | 5000 |
| `TASK1_APPROACH_B_BASE_PWM` | 接近时 B 轮 PWM | 620 |
| `TASK1_APPROACH_A_BASE_PWM` | 接近时 A 轮 PWM | 630 |
| `TASK1_FORCE_STOP_COUNT` | 强制停止距离 | 9500 |
| `TASK1_STOP_MIN_IR_COUNT` | 停止最小红外计数 | 1 |

### 9.4 修正参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK1_DISTANCE_CORR_DIVISOR` | 距离修正除数 | 16 |
| `TASK1_DISTANCE_CORR_MAX` | 距离修正最大值 | 45 |
| `TASK1_HEADING_CORR_DIVISOR` | 航向修正除数 | 12 |
| `TASK1_HEADING_CORR_MAX` | 航向修正最大值 | 90 |
| `TASK1_HEADING_CORR_SIGN` | 航向修正符号 | -1 |
| `TASK1_HEADING_FILTER_DIVISOR` | 航向滤波除数 | 5 |
| `TASK1_HEADING_WOBBLE_FILTER_DIVISOR` | 晃动滤波除数 | 8 |
| `TASK1_HEADING_DEADBAND_CDEG` | 航向死区 | 60 |
| `TASK1_HEADING_GYRO_GATE_START_MDPS` | 陀螺仪门限起点 | 30000 |
| `TASK1_HEADING_GYRO_GATE_END_MDPS` | 陀螺仪门限终点 | 80000 |
| `TASK1_HEADING_PRIORITY_CDEG` | 航向优先阈值 | 250 |
| `TASK1_HEADING_PRIORITY_MAX_VERR` | 优先最大速度误差 | 18 |
| `TASK1_HEADING_PRIORITY_MAX_DERR` | 优先最大距离误差 | 240 |

---

## 10. 任务二参数（A→B→C→D→A）

### 10.1 时序参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_POINT_ALARM_MS` | 点位报警时间 | 120 ms |
| `TASK2_ARC_MAX_RUN_MS` | 弧线最大运行时间 | 12000 ms |
| `TASK2_ARC_REPORT_PERIOD_MS` | 弧线报告周期 | 40 ms |

### 10.2 RAM 日志参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_RAM_LOG_ENABLE` | RAM 日志使能 | 1 |
| `TASK2_RAM_SAMPLE_CAPACITY` | 采样容量 | 192 |
| `TASK2_RAM_EVENT_CAPACITY` | 事件容量 | 20 |
| `TASK2_DUMP_LINE_DELAY_MS` | 导出行延时 | 40 ms |
| `TASK2_STRAIGHT_RAM_LOG_PERIOD_MS` | 直线日志周期 | 40 ms |

### 10.3 CD 直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_CD_STRAIGHT_TARGET_CDEG` | CD 航向目标 | 18150 |
| `TASK2_CD_HEADING_CORR_DIVISOR` | CD 航向修正除数 | 5 |
| `TASK2_CD_HEADING_CORR_MAX` | CD 航向修正最大值 | 160 |
| `TASK2_CD_HEADING_GYRO_DAMP_DIVISOR` | CD 陀螺仪阻尼除数 | 700 |

### 10.4 几何参数

| 宏 | 说明 | 默认值 | 计算公式 |
|----|------|--------|----------|
| `COUNTS_PER_CM` | 每厘米编码器脉冲数 | 66 | — |
| `TASK2_ARC_RADIUS_CM` | 弧线半径 | 40 cm | — |
| `TASK2_ARC_SENSOR_TO_AXIS_MM` | 传感器到轴距 | 175 mm | — |
| `TASK2_ARC_LENGTH_COUNT` | 弧线长度脉冲数 | — | `(31416 * RADIUS * COUNTS_PER_CM) / 10000` |
| `TASK2_ARC_EXIT_EXTRA_COUNT` | 出口额外距离 | — | `(SENSOR_TO_AXIS * COUNTS_PER_CM + 5) / 10` |
| `TASK2_ARC_EXIT_ARM_COUNT` | 出口使能距离 | — | `LENGTH_COUNT - EXIT_EXTRA_COUNT` |
| `TASK2_ARC_FINISH_COUNT` | 完成距离 | — | `LENGTH_COUNT + EXIT_EXTRA_COUNT` |
| `TASK2_ARC_FORCE_STOP_COUNT` | 强制停止距离 | — | `FINISH_COUNT + 1800` |

### 10.5 弧线对齐参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_ARC_ALIGN_TARGET_CDEG` | 对齐目标航向 | 17000 |
| `TASK2_ARC_ALIGN_TOL_CDEG` | 对齐容差 | 200 |
| `TASK2_ARC_ALIGN_TIMEOUT_MS` | 对齐超时 | 2500 ms |
| `TASK2_ARC_ALIGN_INNER_PWM` | 内轮 PWM | 400 |
| `TASK2_ARC_ALIGN_OUTER_PWM` | 外轮 PWM | 460 |

---

## 11. ST011 声光模块

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `ST011_ACTIVE_LOW` | 低电平触发 | 1 |

---

## 12. 编码器自检参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `ENCODER_TEST_PWM` | 自检 PWM | 260 |
| `ENCODER_TEST_MS` | 自检时间 | 500 ms |
| `ENCODER_MIN_PULSE` | 最小脉冲数 | 2 |

**注意**：仅在小车架空时开启自检。

---

## 13. 05 轮速 PID 测试参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `PID_TEST_TARGET_SPEED_DIFF` | 目标速度差 | 0 |
| `PID_TEST_I_LIMIT` | 积分限幅 | 600 |
| `PID_TEST_CORR_MAX` | 最大修正量 | 180 |
| `PID_TEST_DIFF_FF_GAIN` | 前馈增益 | 3 |
| `PID_TEST_DISTANCE_CORR_DIVISOR` | 距离修正除数 | 9 |
| `PID_TEST_DISTANCE_CORR_MAX` | 距离修正最大值 | 45 |

---

## 14. 07 PD 测试参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `PD_TEST_TARGET_SPEED_DIFF` | 目标速度差 | 20 |
| `PD_TEST_KP` | 比例系数 | 22 |
| `PD_TEST_KD` | 微分系数 | 6 |
| `PD_TEST_CORR_MAX` | 最大修正量 | 180 |
| `PD_TEST_DIFF_FF_GAIN` | 前馈增益 | 3 |
| `PD_TEST_DISTANCE_CORR_DIVISOR` | 距离修正除数 | 12 |
| `PD_TEST_DISTANCE_CORR_MAX` | 距离修正最大值 | 0 |

---

## 15. 任务二 AB 直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_AB_DISTANCE_CORR_DIVISOR` | 距离修正除数 | 20 |
| `TASK2_AB_DISTANCE_CORR_MAX` | 距离修正最大值 | 65 |
| `TASK2_AB_HEADING_CORR_DIVISOR` | 航向修正除数 | 5 |
| `TASK2_AB_HEADING_CORR_MAX` | 航向修正最大值 | 160 |
| `TASK2_AB_HEADING_DEADBAND_CDEG` | 航向死区 | 25 |
| `TASK2_AB_BIAS_CORRECTION` | 偏差修正 | -10 |
| `TASK2_AB_STOP_MASK` | 停止掩码 | 0xFF |
| `TASK2_AB_STOP_ERROR_MAX` | 停止最大误差 | 4000 |
| `TASK2_CD_STOP_CENTER_MASK` | CD 停止中心掩码 | 0xFF |
| `TASK2_CD_STOP_ERROR_MAX` | CD 停止最大误差 | 4000 |

### 搜索参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK2_STRAIGHT_SEARCH_START_COUNT` | 搜索起始距离 | 5600 |
| `TASK2_STRAIGHT_SEARCH_SWEEP_START_COUNT` | 扫描起始距离 | 6500 |
| `TASK2_STRAIGHT_SEARCH_CORR_DIVISOR` | 搜索修正除数 | 45 |
| `TASK2_STRAIGHT_SEARCH_CORR_MAX` | 搜索修正最大值 | 80 |
| `TASK2_STRAIGHT_SEARCH_SOFT_CORR` | 软修正 | 38 |
| `TASK2_STRAIGHT_SEARCH_SWEEP_CORR` | 扫描修正 | 68 |
| `TASK2_STRAIGHT_SEARCH_BASE_DROP` | 基础降速 | 45 |

---

## 16. 任务三参数（A→C→B→D→A）

### 16.1 时序参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK3_POINT_ALARM_MS` | 点位报警时间 | 120 ms |
| `TASK3_ARC_MAX_RUN_MS` | 弧线最大运行时间 | 12000 ms |
| `TASK3_ARC_REPORT_PERIOD_MS` | 弧线报告周期 | 100 ms |

### 16.2 几何参数

| 宏 | 说明 | 默认值 | 计算公式 |
|----|------|--------|----------|
| `TASK3_SENSOR_TO_AXIS_MM` | 传感器到轴距 | 175 mm | — |
| `TASK3_ARC_RADIUS_CM` | 弧线半径 | 40 cm | — |
| `TASK3_ARC_LENGTH_COUNT` | 弧线长度脉冲数 | — | `(31416 * RADIUS * COUNTS_PER_CM) / 10000` |
| `TASK3_ARC_EXIT_IGNORE_COUNT` | 出口忽略距离 | — | `LENGTH_COUNT / 2` |
| `TASK3_ARC_EXIT_EXTRA_COUNT` | 出口额外距离 | — | `(SENSOR_TO_AXIS * COUNTS_PER_CM + 5) / 10` |
| `TASK3_ARC_FINISH_COUNT` | 完成距离 | — | `LENGTH_COUNT + EXIT_EXTRA_COUNT` |
| `TASK3_ARC_FORCE_STOP_COUNT` | 强制停止距离 | — | `FINISH_COUNT + 1800` |

### 16.3 航向参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK3_AC_HEADING_TARGET_CDEG` | AC 航向目标 | -3660 |
| `TASK3_BD_HEADING_TARGET_CDEG` | BD 航向目标 | -13920 |

### 16.4 直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK3_STRAIGHT_LINE_ARM_COUNT` | 线检测使能距离 | 6100 |
| `TASK3_STRAIGHT_FORCE_STOP_COUNT` | 强制停止距离 | 11500 |
| `TASK3_STRAIGHT_CORR_MAX` | 修正最大值 | 155 |
| `TASK3_BD_HEADING_CORR_DIVISOR` | BD 航向修正除数 | 5 |
| `TASK3_BD_HEADING_CORR_MAX` | BD 航向修正最大值 | 170 |

### 16.5 搜索参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK3_STRAIGHT_SEARCH_START_COUNT` | 搜索起始距离 | 6100 |
| `TASK3_STRAIGHT_SEARCH_SWEEP_START_COUNT` | 扫描起始距离 | 6500 |
| `TASK3_STRAIGHT_SEARCH_SWEEP_PERIOD_MS` | 扫描周期 | 70 ms |
| `TASK3_STRAIGHT_SEARCH_CORR_DIVISOR` | 搜索修正除数 | 38 |
| `TASK3_STRAIGHT_SEARCH_CORR_MAX` | 搜索修正最大值 | 145 |
| `TASK3_STRAIGHT_SEARCH_SOFT_CORR` | 软修正 | 85 |
| `TASK3_STRAIGHT_SEARCH_SWEEP_CORR` | 扫描修正 | 145 |
| `TASK3_STRAIGHT_SEARCH_BASE_DROP` | 基础降速 | 60 |

### 16.6 弧线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK3_ARC_B_BASE_PWM` | B 轮基础 PWM | 450 |
| `TASK3_ARC_A_BASE_PWM` | A 轮基础 PWM | 480 |
| `TASK3_ARC_MIN_PWM` | 最小 PWM | 0 |
| `TASK3_ARC_MAX_PWM` | 最大 PWM | 620 |
| `TASK3_ARC_ENTRY_COUNT` | 入口距离 | 3400 |
| `TASK3_ARC_ENTRY_TURN` | 入口转向 | 130 |
| `TASK3_ARC_ENTRY_BASE_DROP` | 入口基础降速 | 40 |
| `TASK3_ARC_WIDE_LINE_MIN_COUNT` | 宽线最小计数 | 6 |

---

## 17. 任务四参数（A→C→B→D→A 4 圈）

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK4_LAP_COUNT` | 圈数 | 4 |
| `TASK4_AC_HEADING_TARGET_CDEG` | AC 航向目标 | = TASK3_AC_HEADING_TARGET_CDEG |
| `TASK4_BD_HEADING_TARGET_CDEG` | BD 航向目标 | = TASK3_BD_HEADING_TARGET_CDEG |
| `TASK4_AC_LINE_SEARCH_PROTECT` | AC 搜索保护 | 2 |
| `TASK4_AC_START_TURN_COUNT` | AC 起始转向距离 | 3400 |
| `TASK4_AC_START_BOOST_MIN_ERR_CDEG` | AC 起始加速最小误差 | 1200 |
| `TASK4_AC_START_HEADING_CORR_DIVISOR` | AC 起始航向修正除数 | 3 |
| `TASK4_AC_START_HEADING_CORR_MAX` | AC 起始航向修正最大值 | 300 |
| `TASK4_AC_START_CORR_MAX` | AC 起始修正最大值 | 285 |

---

## 18. 任务六参数（C 点快速转向测试）

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK6_C_TURN_TARGET_CDEG` | 转向目标 | 2200 |
| `TASK6_C_TURN_B_PWM` | B 轮 PWM | -220 |
| `TASK6_C_TURN_A_PWM` | A 轮 PWM | 760 |
| `TASK6_C_TURN_TIMEOUT_MS` | 转向超时 | 1200 ms |
| `TASK6_C_TURN_REPORT_PERIOD_MS` | 报告周期 | 40 ms |
| `TASK6_C_TURN_SAMPLE_MAX` | 最大采样数 | 64 |
| `TASK6_C_TURN_LINE_ARM_CDEG` | 线使能角度 | 1200 |
| `TASK6_C_TURN_LINE_STOP_MASK` | 线停止掩码 | 0x7E |
| `TASK6_C_TURN_LINE_STOP_MIN_COUNT` | 线停止最小计数 | 1 |

### 任务四 D 点转向参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `TASK4_D_TURN_TARGET_CDEG` | 转向目标 | -2200 |
| `TASK4_D_TURN_B_PWM` | B 轮 PWM | 760 |
| `TASK4_D_TURN_A_PWM` | A 轮 PWM | -220 |

---

## 19. 竞速控制参数（RACE_*）

### 19.1 日志参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_UART_LOG_ENABLE` | 串口日志使能 | 0 |
| `RACE_RAM_LOG_ENABLE` | RAM 日志使能 | 0 |
| `RACE_RAM_LOG_MAX_LAPS` | 最大圈数 | 5 |
| `RACE_RAM_WINDOW_CAPACITY` | 窗口容量 | 560 |
| `RACE_RAM_EVENT_CAPACITY` | 事件容量 | 112 |
| `RACE_RAM_SUMMARY_CAPACITY` | 摘要容量 | 24 |
| `RACE_RAM_WINDOW_BEFORE_COUNT` | 窗口前距离 | 1800 |
| `RACE_RAM_WINDOW_AFTER_START_COUNT` | 窗口后距离 | 1800 |
| `RACE_DUMP_LINE_DELAY_MS` | 导出行延时 | 100 ms |
| `RACE_DUMP_SECTION_DELAY_MS` | 导出段延时 | 1000 ms |
| `RACE_REALTIME_EVENT_LOG_ENABLE` | 实时事件日志 | 0 |

### 19.2 时序参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_POINT_ALARM_MS` | 点位报警时间 | 80 ms |
| `RACE_START_ALARM_MS` | 启动报警时间 | = RACE_POINT_ALARM_MS |
| `RACE_POINT_SETTLE_MS` | 点位稳定时间 | 120 ms |
| `RACE_TOTAL_MAX_RUN_MS` | 最大运行时间 | 240000 ms |
| `RACE_LINE_REPORT_PERIOD_MS` | 报告周期 | 200 ms |

### 19.3 直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_LINE_BASE_PWM` | 直线基础 PWM | 560 |
| `RACE_LINE_MIN_PWM` | 最小 PWM | 0 |
| `RACE_LINE_MAX_PWM` | 最大 PWM | 760 |
| `RACE_TASK4_LINE_MAX_PWM` | 任务四直线最大 PWM | 900 |
| `RACE_LINE_TURN_DIVISOR` | 转向除数 | 9 |
| `RACE_LINE_KD_DIVISOR` | 微分除数 | 7 |
| `RACE_LINE_DERIV_LIMIT` | 微分限幅 | 700 |
| `RACE_LINE_TURN_LIMIT` | 转向限幅 | 260 |
| `RACE_LINE_ERROR_FILTER_DIVISOR` | 误差滤波除数 | 2 |
| `RACE_LINE_LOST_BASE_DROP` | 丢线基础降速 | 60 |
| `RACE_LINE_LOST_TURN` | 丢线转向 | 150 |

### 19.4 竞速直线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_STRAIGHT_BASE_PWM` | 直线基础 PWM | 600 |
| `RACE_STRAIGHT_TARGET_DIFF` | 目标速度差 | 0 |
| `RACE_TASK4_STRAIGHT_BASE_PWM` | 任务四直线基础 PWM | 840 |
| `RACE_TASK4_STRAIGHT_TARGET_DIFF` | 任务四直线目标速度差 | 0 |
| `RACE_TASK4_ENTRY_DECEL_START_COUNT` | 任务四入弯前减速起点 | RACE_AC_POINT_ARM_COUNT - 500 |
| `RACE_TASK4_ENTRY_DECEL_RAMP_COUNT` | 任务四入弯前减速距离 | 350 |
| `RACE_STRAIGHT_GYRO_NAV_ENABLE` | 陀螺仪导航使能 | 1 |
| `RACE_STRAIGHT_IR_ASSIST_ENABLE` | 红外辅助使能 | 0 |
| `RACE_STRAIGHT_IR_ASSIST_DIVISOR` | 红外辅助除数 | 3 |
| `RACE_STRAIGHT_HEADING_CORR_DIVISOR` | 航向修正除数 | 8 |
| `RACE_STRAIGHT_HEADING_CORR_MAX` | 航向修正最大值 | 140 |
| `RACE_STRAIGHT_GYRO_DAMP_DIVISOR` | 陀螺仪阻尼除数 | 1600 |

### 19.5 竞速弧线参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_ARC_BASE_PWM` | 弧线基础 PWM | 540 |
| `RACE_TASK4_ARC_BASE_PWM` | 任务四弧线基础 PWM | 670 |
| `RACE_CB_ARC_ENTRY_TARGET_DIFF` | CB 入口目标差 | -48 |
| `RACE_CB_ARC_CRUISE_TARGET_DIFF` | CB 巡航目标差 | -48 |
| `RACE_DA_ARC_ENTRY_TARGET_DIFF` | DA 入口目标差 | 46 |
| `RACE_DA_ARC_CRUISE_TARGET_DIFF` | DA 巡航目标差 | 46 |
| `RACE_ARC_YAW_NAV_ENABLE` | 弧线航向导航使能 | 0 |
| `RACE_ARC_YAW_CORR_DIVISOR` | 弧线航向修正除数 | 260 |
| `RACE_ARC_YAW_CORR_MAX` | 弧线航向修正最大值 | 45 |
| `RACE_ARC_GYRO_DAMP_DIVISOR` | 弧线陀螺仪阻尼除数 | 4400 |
| `RACE_TASK4_EXIT_DECEL_START_COUNT` | 任务四出弯前减速起点 | TASK3_ARC_LENGTH_COUNT - 2300 |
| `RACE_TASK4_EXIT_DECEL_RAMP_COUNT` | 任务四出弯前减速距离 | 350 |

### 19.6 差速参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_DIFF_KP` | 比例系数 | = PD_TEST_KP |
| `RACE_DIFF_KD` | 微分系数 | = PD_TEST_KD |
| `RACE_DIFF_FF_GAIN` | 前馈增益 | 2 |
| `RACE_DIFF_CORR_MAX` | 最大修正量 | = PD_TEST_CORR_MAX |

### 19.7 红外掩码

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_IR_LEFT_EDGE_MASK` | 左边缘掩码 | 0x03 |
| `RACE_IR_RIGHT_EDGE_MASK` | 右边缘掩码 | 0xC0 |
| `RACE_IR_CENTER_6_MASK` | 中心 6 路掩码 | 0x7E |
| `RACE_IR_CENTER_6_FORBID_MASK` | 中心 6 路禁止掩码 | 0x81 |
| `RACE_IR_TURN_STOP_MIN_COUNT` | 转向停止最小计数 | 1 |

### 19.8 转向 PWM 参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_LEFT_TURN_B_PWM` | 左转 B 轮 PWM | -120 |
| `RACE_LEFT_TURN_A_PWM` | 左转 A 轮 PWM | 520 |
| `RACE_LEFT_TURN_SLOW_B_PWM` | 左转慢速 B 轮 PWM | 80 |
| `RACE_LEFT_TURN_SLOW_A_PWM` | 左转慢速 A 轮 PWM | 360 |
| `RACE_RIGHT_TURN_B_PWM` | 右转 B 轮 PWM | 520 |
| `RACE_RIGHT_TURN_A_PWM` | 右转 A 轮 PWM | 120 |
| `RACE_RIGHT_TURN_SLOW_B_PWM` | 右转慢速 B 轮 PWM | 360 |
| `RACE_RIGHT_TURN_SLOW_A_PWM` | 右转慢速 A 轮 PWM | 80 |
| `RACE_EXIT_LEFT_TURN_B_PWM` | 出口左转 B 轮 PWM | 80 |
| `RACE_EXIT_LEFT_TURN_A_PWM` | 出口左转 A 轮 PWM | 440 |
| `RACE_EXIT_LEFT_TURN_SLOW_B_PWM` | 出口左转慢速 B 轮 PWM | 60 |
| `RACE_EXIT_LEFT_TURN_SLOW_A_PWM` | 出口左转慢速 A 轮 PWM | 300 |
| `RACE_EXIT_RIGHT_TURN_B_PWM` | 出口右转 B 轮 PWM | 440 |
| `RACE_EXIT_RIGHT_TURN_A_PWM` | 出口右转 A 轮 PWM | 80 |
| `RACE_EXIT_RIGHT_TURN_SLOW_B_PWM` | 出口右转慢速 B 轮 PWM | 300 |
| `RACE_EXIT_RIGHT_TURN_SLOW_A_PWM` | 出口右转慢速 A 轮 PWM | 60 |

### 19.9 快速转向参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_FAST_TURN_TIMEOUT_MS` | 快速转向超时 | 600 ms |
| `RACE_FAST_TURN_REPORT_PERIOD_MS` | 报告周期 | = TASK6_C_TURN_REPORT_PERIOD_MS |
| `RACE_FAST_TURN_GYRO_SLOW_ENABLE` | 陀螺仪慢速使能 | 1 |
| `RACE_FAST_TURN_GYRO_SLOW_CDEG` | 陀螺仪慢速阈值 | 2600 |
| `RACE_EXIT_TURN_YAW_STOP_ENABLE` | 出口转向航向停止使能 | 1 |
| `RACE_TURN_YAW_STOP_TOL_CDEG` | 航向停止容差 | 260 |
| `RACE_TURN_YAW_SLOW_ZONE_CDEG` | 航向慢速区域 | 900 |
| `RACE_TURN_YAW_STOP_GZLP_TOL_MDPS` | 航向停止角速度容差 | 14000 |
| `RACE_TASK4_EXIT_TURN_PREDICT_ENABLE` | 任务四出弯预测停止 | 1 |
| `RACE_TASK4_EXIT_TURN_PREDICT_MS` | 任务四出弯预测时间 | 40 ms |
| `RACE_TASK4_EXIT_TURN_PREDICT_MIN_GZ_MDPS` | 任务四预测停止最小角速度 | = RACE_TURN_YAW_STOP_GZLP_TOL_MDPS |

### 19.10 竞速航向参数

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `RACE_TASK3_AC_HEADING_TARGET_CDEG` | 任务三 AC 航向目标 | -3300 |
| `RACE_TASK3_BD_HEADING_TARGET_CDEG` | 任务三 BD 航向目标 | -18000 + 3850 |
| `RACE_TASK4_AC_HEADING_TARGET_CDEG` | 任务四 AC 航向目标 | -3300 |
| `RACE_TASK4_BD_HEADING_TARGET_CDEG` | 任务四 BD 航向目标 | -18000 + 3800 |
| `RACE_TURN_CENTER6_ERROR_MAX` | 中心 6 路最大误差 | 1500 |
| `RACE_POINT_ADVANCE_COUNT` | 点位前进距离 | 300 |
| `RACE_ARC_POINT_ADVANCE_COUNT` | 弧线点位前进距离 | 800 |
| `RACE_POINT_ADVANCE_PWM` | 点位前进 PWM | 360 |
| `RACE_POINT_ADVANCE_TIMEOUT_MS` | 点位前进超时 | 800 ms |
| `RACE_AC_POINT_ARM_COUNT` | AC 点位使能距离 | 7300 |
| `RACE_BD_POINT_ARM_COUNT` | BD 点位使能距离 | 7300 |
| `RACE_TASK4_BD_POINT_ARM_COUNT` | 任务四 BD 点位使能距离 | 6600 |
| `RACE_TASK3_AC_FORCE_TURN_COUNT` | 任务三 AC 强制入弯距离 | 7750 |
| `RACE_TASK4_AC_FORCE_TURN_COUNT` | 任务四 AC 强制入弯距离 | 7520 |
| `RACE_TASK4_FIRST_AC_FORCE_TURN_COUNT` | 任务四第一圈 AC 强制入弯距离 | 7800 |
| `RACE_BD_FORCE_TURN_COUNT` | BD 强制入弯距离 | 7400 |
| `RACE_TASK4_BD_FORCE_TURN_COUNT` | 任务四 BD 强制入弯距离 | 7200 |
| `RACE_FORCE_ENTRY_TURN_CDEG` | 强制入弯转向角 | 3000 |
| `RACE_FORCE_FIND_LINE_COUNT` | 强制转向后找线距离 | 1800 |
| `RACE_FORCE_FIND_LINE_TIMEOUT_MS` | 强制转向后找线超时 | 1200 ms |
| `RACE_STRAIGHT_FORCE_COUNT` | 直线强制停止距离 | 12800 |
| `RACE_STRAIGHT_POINT_CONFIRM_COUNT` | 直线点位确认次数 | 1 |
| `RACE_ARC_POINT_YAW_ARM_CDEG` | 弧线点位航向使能 | 14000 |
| `RACE_GYRO_TURN_TIMEOUT_MS` | 陀螺仪转向超时 | 1200 ms |

---

## 20. 编码器方向符号

| 宏 | 说明 | 默认值 |
|----|------|--------|
| `ENCODER_MOTOR_A_FORWARD_SIGN` | A 轮前进符号 | -1 |
| `ENCODER_MOTOR_B_FORWARD_SIGN` | B 轮前进符号 | 1 |

**说明**：统一让前进方向计数为正。

---

## 调参建议

1. **每次只改一个参数**
2. **记录修改前后的值**
3. **修改后重新编译验证**
4. **重要参数修改前备份**
5. **实车测试时观察串口日志**
