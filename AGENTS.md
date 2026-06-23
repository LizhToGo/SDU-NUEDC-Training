# AGENTS.md - MSPM0G3507 自动行驶小车固件

## 工程概况

TI MSPM0G3507 嵌入式固件，CCS + SysConfig + DriverLib，控制差速小车。外设包括八路红外循迹、JY62 惯导、TB6612 电机驱动、ST011 声光模块和编码器。

## 构建

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug all
```

clean build：

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug clean all
```

不要手动修改 `Debug/`、`ti_msp_dl_config.c/.h`、makefile、`.out`、`.map` 等生成物。

## 验收版任务入口

| 触发 | 任务 | 入口函数 |
|---|---|---|
| A26 / UART `01` | 任务一 A->B 直线 | `run_task1_ab()` |
| A24 / UART `02` | 任务二 A->B->C->D->A | `run_task2_abcd()` |
| B24 / UART `03` | 任务三 1 圈竞速路线 | `run_race_laps(1U)` |
| A22 / UART `04` | 任务四 4 圈竞速路线 | `run_race_laps(4U)` |
| UART `00` | 运行中强制停车 | `TASK_ID_STOP` |

当前验收版不再包含 UART `05/06/07/08/10` 调试任务。

## 代码结构要点

```text
main.c              # 启动、ISR、进入任务调度器
app_config.h        # 任务一二三四调参宏
app_control.h       # 限幅、斜坡、PID、航向滤波
app_straight.h      # 差速直行基础配置
app_motion_utils.h  # normalize_cdeg()、距离工具
app_services.c/.h   # ST011、带声光 delay、UART stop 检测
app_task_ids.c/.h   # 任务 ID / UART 命令解析，仅 01..04
straight/           # 任务一/二直线段
turn/               # 任务二弧线段
race/               # 任务三/四竞速核心
tasks/              # 任务调度器、任务一/二序列
Board/              # lc_printf()、delay
BSP/                # TB6612、红外、JY62、编码器驱动
tools/              # RACE RAM 日志整理
```

已删除历史调试文件：`app_debug_modes.h`、`heading/heading_straight.h`、`turn/line_fast_turn.h`、`tasks/task6_turn_test.h`、`tasks/task8_exit_turn_calibration.h`。

## 协作规则

- 默认只做小步、可回退的改动。
- 实车稳定参数优先，重构不要改变任务一二三四行为。
- 调参优先改 `app_config.h`。
- 任务四高速参数尽量使用 `RACE_TASK4_*`，不要顺手改任务三。
- Push 前运行构建，并确认日志/CSV/Debug 输出未误提交。

## 文档索引

| 文件 | 内容 |
|---|---|
| `README.md` | 项目总览 |
| `docs/串口调试与跑车流程.md` | 验收版串口命令和跑车流程 |
| `docs/代码结构与协作说明.md` | 模块边界 |
| `docs/app_config宏定义说明.md` | 调参宏索引 |
| `docs/任务三调试说明.md` / `docs/任务四调试说明.md` | 任务三四说明 |
