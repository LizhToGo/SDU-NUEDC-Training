# AGENTS.md — MSPM0G3507 自动驾驶小车固件

## 工程概况

TI MSPM0G3507 嵌入式固件，CCS + SysConfig + DriverLib，控制差速小车（八路红外循迹、JY62 惯导、TB6612 电机驱动、ST011 声光模块）。

## 构建

```powershell
# 增量构建
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug all

# clean 构建（偶尔 .d 文件缺失警告可忽略，退出码 0 即成功）
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug clean all
```

## 关键约束

- **不要手动修改生成物**：`Debug/`、`ti_msp_dl_config.c/.h`、`makefile`、`.out`、`.map` 等。
- **`main.syscfg` 是外设配置源头**：引脚、时钟、中断路由由此生成，改外设先改 `.syscfg`。
- **Header-only 架构**：新增业务模块用 `.h` 实现，不拆 `.c/.h`——CCS 生成的 makefile 不自动纳入新 `.c` 文件。拆分前需确认 `.cproject` 注册。
- **Include 顺序敏感**：`main.c` 中 header-only 模块的 include 顺序不能随意移动，会导致隐式声明或前向依赖问题。当前顺序：
  1. `heading/heading_straight.h`（任务四调试钩子之后）
  2. `race/task2_ram_log.h`
  3. `turn/arc_segment.h`（Task2 弧线对准 helper 之后）
  4. `race/race_laps.h`（任务一/二/六入口之后、`run_task_dispatcher()` 之前）
- **调参只改 `app_config.h`**：所有速度、PID、航向、距离、日志开关宏集中在此。
- **竞速参数前缀统一**：`RACE_*` 宏、`race_` 函数、`RACE_` 日志前缀。

## 任务入口

| 触发 | 任务 | 入口函数 |
|------|------|----------|
| A26 按键 / UART `01` | 任务一 A→B 直线 | `run_task1_ab()` |
| A24 按键 / UART `02` | 任务二 A→B→C→D→A | `run_task2_abcd()` |
| B24 按键 / UART `03` | 任务三 1 圈竞速 | `run_race_laps(1U)` |
| A22 按键 / UART `04` | 任务四 4 圈竞速 | `run_race_laps(4U)` |
| UART `05` | 轮速 PID 测试 | `run_motor_pid_stream()` |
| UART `06` | C 点转向测试 | `run_task6_ac_c_turn_test()` |
| UART `07` | 轮速 PD 测试 | `run_motor_pd_stream()` |
| UART `10` | AB 零角度标定 | `run_task10_ab_zero_test()` |
| UART `00` | 强制停车 | `TASK_ID_STOP` |

串口同时接受 HEX 字节和 ASCII 数字（发 `0x03` 或 `"03"` 均可）。

## 代码结构要点

```
main.c              # 启动、ISR、任务分发、任务一/二入口（~510 行）
app_config.h        # 所有调参宏（423 行）
app_control.h       # 限幅、斜坡、PID、航向滤波
app_straight.h      # 差速直行闭环控制
app_motion_utils.h  # normalize_cdeg()、距离工具
app_services.c/.h   # ST011 声光、带声光延时、UART stop 检测
app_task_ids.c/.h   # 任务 ID / UART 命令解析
app_debug_modes.h   # 05/07 轮速测试、红外打印等调试模式
race/               # 竞速核心：race_laps.h（主逻辑）、race_phase.h（阶段控制）、race_primitives.h（运动原语）、race_log.h（RAM 日志）
heading/            # 航向直线控制
turn/               # 快速转向、弧线段
straight/           # 直线行驶封装
tasks/              # 任务调度器、任务序列、Task6 测试
Board/              # UART 打印 lc_printf()、delay
BSP/                # TB6612、红外、JY62、编码器驱动
tools/              # task11_log_to_csv.py 日志整理
```

## 竞速路径（任务三/四）

```
AC 直线（航向目标）→ CB 左弧线（红外循迹）→ BD 直线（航向目标）→ DA 右弧线（红外循迹）
```

关键调参入口：`RACE_AC_HEADING_TARGET_CDEG`、`RACE_BD_HEADING_TARGET_CDEG`、`RACE_STRAIGHT_BASE_PWM`、`RACE_ARC_BASE_PWM`。

## 日志系统

- `RACE_UART_LOG_ENABLE`：实时串口文本日志（注意：太密会影响实车控制）。
- `RACE_RAM_LOG_ENABLE`：RAM dump 后处理，竞速大体量日志首选。
- `TASK2_RAM_LOG_ENABLE`：Task2 RAM 日志。
- 日志整理脚本：`tools/task11_log_to_csv.py`。

## Git 注意

- `.gitignore` 已排除 `Debug/`、`.out`、`.map`、`opencode.json`、`.mimocode/`、日志 CSV。
- Push 前确认 `app_config.h` 日志开关、完整编译通过、无生成物误提交。

## 文档索引

| 文件 | 内容 |
|------|------|
| `docs/代码结构与协作说明.md` | 模块边界、header-only 约定、include 关系 |
| `docs/串口调试与跑车流程.md` | 串口命令、跑车流程、日志字段 |
| `docs/任务三调试说明.md` / `任务四调试说明.md` | 路径、点位、常见问题 |
| `docs/app_config宏定义说明.md` | 调参宏索引 |
| `docs/TB6612驱动使用说明.md` | 电机驱动接线、方向校准 |
| `docs/JY62陀螺仪驱动说明.md` | 惯导接线、协议解析、排错 |
| `docs/八路红外循迹模块驱动说明.md` | 红外 I2C 读取、mask/err 含义 |
