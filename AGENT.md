# 工程协作说明

本文记录当前 `main` 分支的实际工程状态，供协作调试和后续 Agent 接手时快速对齐。

## 工程边界

- 芯片/工程类型：TI MSPM0G3507，SysConfig + DriverLib + CCS 工程。
- 不手动修改生成物：`Debug/`、SysConfig 生成文件、`.out/.map/.obj` 等构建输出。
- `main.syscfg` 是引脚、外设、时钟和中断配置源头。
- 新增源码文件时要注意 CCS 工程是否需要注册；本轮 `race_log.h` 是 header-only 拆分，不需要改 `.cproject`。
- 验证命令优先使用 `C:\ti\ccs2051\ccs\utils\bin\gmake.exe -C Debug clean all`。

## 当前源码结构

- `main.c`：保留系统初始化、ISR、任务调度、任务一/二/六/十、竞速编排和运动原语，当前约 4400 行。
- `race_log.h`：竞速 RAM 日志系统，包含 `race_ram_log_*()`、`RACE_RAM_*` dump 输出和日志存储。
- `app_config.h`：集中放置调参宏。当前竞速参数宏已统一使用 `RACE_*` 命名，函数和日志前缀为 `race_` / `RACE_`。
- `app_control.h`：限幅、斜坡、PID、航向滤波等控制基础工具。
- `app_motion_utils.h`：角度归一化 `normalize_cdeg()` 和左右轮平均距离工具。
- `app_services.h/.c`：ST011 声光模块、带 ST011 服务的延时、UART stop 检测。
- `app_task_ids.h/.c`：任务 ID、UART/按键任务命令解析、等待任务入口。
- `app_straight.h`：差速直行控制模块。
- `app_debug_modes.h`：05/07 轮速测试、红外打印、纯红外循迹等调试模式。
- `tools/task11_log_to_csv.py`：历史 Task11/当前 RACE RAM 日志整理脚本，仍保留旧文件名。
- `Board/`：UART0 打印和基础延时。
- `BSP/`：TB6612、红外、编码器、JY62 驱动。

## 任务入口

- UART/按键 `01`：任务一。
- UART/按键 `02`：任务二。
- UART/按键 `03`：调用 `run_race_laps(1U)`。
- UART/按键 `04`：调用 `run_race_laps(4U)`。
- UART `05/06/07/10`：调试或专项验证入口。
- UART `00`：运行中强制停车。
- UART `11` 已移除，不再作为可运行入口。

## 代码重构状态

- 已删除旧 Task3/4 死代码：`run_task3_acbda()`、`run_task4_lap()`、`run_task4_four_laps()` 等。
- 已删除旧 `run_task3_race_arc_line_follow_segment()` 残留。
- `run_task11_ir_map_test_laps()` 已改名为 `run_race_laps()`。
- 实现层 `task11_*` 已重命名为 `race_*`，日志前缀已统一为 `RACE_*`。
- `app_config.h` 中竞速相关参数已统一为 `RACE_*` 前缀，这些参数仍是当前竞速路径实际配置。
- 已消除 `bsp_encoder.h` 部分重复逻辑。
- 已减少 `task5_ram_log_sample()` 参数数量。
- `race_log.h` 已从 `main.c` 抽出，当前代码质量总分约 `61.44/100`，`main.c` 仍是后续重构重点。

## 调参提醒

- 任务一/任务二 AB 主要看 `run_straight_to_line_segment()` 和 `app_straight.h`。
- 任务三/四当前入口走竞速路径，优先看 `run_race_laps()`、`race_*` helper、`RACE_*` 竞速参数、`TASK3_AC_HEADING_TARGET_CDEG`、`TASK3_BD_HEADING_TARGET_CDEG`。
- 串口日志太密会影响实车控制，竞速大体量日志优先使用 RAM dump 后处理。
- 旧文档中关于 UART `11`、`run_task11_ir_map_test_laps()`、`RACE_DATA/RACE_RAM_END` 的描述需要按当前 `03/04 + run_race_laps + RACE_*` 理解。
