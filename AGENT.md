# 工程协作说明

本文记录当前 `main` 分支的实际工程状态，供协作调试和后续 Agent 接手时快速对齐。

## 工程边界

- 芯片/工程类型：TI MSPM0G3507，SysConfig + DriverLib + CCS 工程。
- 不手动修改生成物：`Debug/`、SysConfig 生成文件、`.out/.map/.obj` 等构建输出。
- `main.syscfg` 是引脚、外设、时钟和中断配置源头。
- 新增 `.c` 文件前要确认 CCS 工程会纳入编译；当前业务代码仍以 header-only 为主。
- 验证命令优先使用 `C:\ti\ccs2051\ccs\utils\bin\gmake.exe -C Debug all`。

## 当前入口

验收版只保留任务一二三四：

- UART/按键 `01`：任务一，`run_task1_ab()`。
- UART/按键 `02`：任务二，`run_task2_abcd()`。
- UART/按键 `03`：任务三，`run_race_laps(1U)`。
- UART/按键 `04`：任务四，`run_race_laps(4U)`。
- UART `00`：运行中强制停车。

UART `05/06/07/08/10/11` 均不是当前验收版入口。

## 当前源码结构

- `main.c`：系统启动、JY62/编码器/TB6612 初始化、ISR、进入任务调度器。
- `app_task_ids.c/.h`：只解析 `01..04` 和运行中 stop。
- `tasks/task_dispatcher.h`：任务调度，只分发任务一二三四。
- `tasks/task_sequences.h`：任务一/二顶层序列。
- `straight/straight_line.h`：任务一/二直线段。
- `turn/arc_segment.h`：任务二弧线段。
- `race/race_laps.h`：任务三/四跑圈主流程。
- `race/race_phase.h`：任务三/四阶段控制和点位判定。
- `race/race_primitives.h`：入弯、出弯、强制找线、航向转向等运动原语。
- `race/race_log.h`：任务三/四 RAM 日志。
- `race/task2_ram_log.h`：任务二 RAM 日志。
- `app_config.h`：集中调参入口。

已删除历史调试模块：`app_debug_modes.h`、`heading/heading_straight.h`、`turn/line_fast_turn.h`、`tasks/task6_turn_test.h`、`tasks/task8_exit_turn_calibration.h`。

## 调参提醒

- 默认只改 `app_config.h`，尤其是任务四高速参数。
- 除非明确要求，不要改任务一/二/三的稳定参数。
- 任务三/四共用竞速实现，任务四通过 `RACE_TASK4_*` 拆出高速参数。
- 串口实时日志过密会影响实车控制，高速复盘优先用 RAM dump。
- 本地日志/CSV 属于测试产物，不要随手提交。
