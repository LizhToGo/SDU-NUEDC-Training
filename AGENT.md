# 工程协作说明

本文记录当前 `main` 分支的实际工程状态，供协作调试和后续 Agent 接手时快速对齐。

## 工程边界

- 芯片/工程类型：TI MSPM0G3507，SysConfig + DriverLib + CCS 工程。
- 不手动修改生成物：`Debug/`、SysConfig 生成文件、`.out/.map/.obj` 等构建输出。
- 当前仓库没有 `Debug/` 目录，直接执行 `gmake -C Debug clean all` 会因目录缺失失败。需要先由 CCS/SysConfig 生成 Debug 构建目录后再验证。
- 本轮新增了 `app_services.c` 和 `app_task_ids.c`。首次在 CCS 中构建时，应确认这两个根目录 `.c` 文件已进入工程源文件列表。

## 当前源码结构

- `main.c`：保留任务调度、任务一/二/三/四、任务六、任务十/十一及 ISR。
- `app_config.h`：集中放置调参宏。
- `app_control.h`：限幅、斜坡、PID、航向滤波等控制基础工具。
- `app_motion_utils.h`：角度归一化 `normalize_cdeg()` 和左右轮平均距离工具。
- `app_services.h/.c`：ST011 声光模块、带 ST011 服务的延时、UART stop 检测。
- `app_task_ids.h/.c`：任务 ID、UART/按键任务命令解析、等待任务入口。
- `app_straight.h`：差速直行控制模块。
- `app_debug_modes.h`：05/07、红外打印、纯红外循迹等调试模式。
- `Board/`：UART0 打印和基础延时。
- `BSP/`：TB6612、红外、编码器、JY62 驱动。

## 任务入口

- UART/按键 `01`：任务一。
- UART/按键 `02`：任务二。
- UART/按键 `03`：当前调用 `run_task11_ir_map_test_laps(1U)`。
- UART/按键 `04`：当前调用 `run_task11_ir_map_test_laps(4U)`。
- UART `05/06/07/10/11`：调试或专项验证入口。
- UART `00`：运行中强制停车。

## 当前清理状态

- 已删除 11 个确认不可达旧函数。
- 已删除 84 个当前源码零引用宏。
- 已删除旧例程接口：`u8/u16/u32/u64`、`delay_1us/delay_1ms`、`LOG_D/LOG_Debug_Out`、`TB6612_Motor_Stop/AO_Control/BO_Control`。
- 旧 `run_task5_straight_to_line_segment()` 已改名为 `run_straight_to_line_segment()`。
- 原任务三/四目标航向直线函数已改名为 `run_heading_straight_to_line_segment()`。

## 调参提醒

- 任务一/任务二 AB 主要看 `run_straight_to_line_segment()` 和 `app_straight.h`。
- 任务三/任务四当前入口走 Task11 风格路径，优先看 `TASK11_*`、`TASK3_AC_HEADING_TARGET_CDEG`、`TASK3_BD_HEADING_TARGET_CDEG`。
- 串口日志太密会影响实车控制，Task11 大体量日志优先使用 RAM dump 后处理。
