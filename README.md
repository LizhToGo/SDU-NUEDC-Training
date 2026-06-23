# SDU NUEDC Training

2026 校赛 F 题自动行驶小车固件。工程基于 TI MSPM0G3507、CCS + SysConfig + DriverLib，控制对象是差速小车、八路红外循迹模块、JY62 惯导、TB6612 电机驱动和 ST011 声光模块。

当前代码已经收敛到验收模式：固件只保留任务一、任务二、任务三、任务四入口。历史调试任务 05/06/07/08/10、Task6/Task8 测试头文件、轮速/红外调试模式入口均已从源码中移除。

## 任务入口

| 按键 / UART0 | 任务 | 入口函数 |
|---|---|---|
| `A26` / `01` | 任务一：A -> B 直线 | `run_task1_ab()` |
| `A24` / `02` | 任务二：A -> B -> C -> D -> A | `run_task2_abcd()` |
| `B24` / `03` | 任务三：A -> C -> B -> D -> A，1 圈 | `run_race_laps(1U)` |
| `A22` / `04` | 任务四：任务三路线，4 圈竞速 | `run_race_laps(4U)` |
| UART0 `00` | 运行中强制停车 | `TASK_ID_STOP` |

UART0 同时支持二进制字节和 ASCII 数字，例如 HEX `03` 或 ASCII `03` 都能启动任务三。验收版不再接受 `05/06/07/08/10`。

## 代码结构

```text
main.c                 # 系统启动、JY62/编码器/TB6612 初始化、ISR、进入任务调度器
app_config.h           # 任务一二三四和竞速路径的集中调参入口
app_control.h          # 限幅、斜坡、PID、角度归一化、航向滤波
app_motion_utils.h     # 运动相关小工具
app_services.c/.h      # ST011 声光、带声光服务的 delay、UART stop 检测
app_straight.h         # 差速直行闭环基础配置
app_task_ids.c/.h      # UART/按键任务命令解析，仅 01..04 + 00
straight/straight_line.h
                       # 任务一/任务二直线段
turn/arc_segment.h     # 任务二弧线段
race/race_laps.h       # 任务三/四跑圈主流程
race/race_phase.h      # 任务三/四阶段状态、点位判定、阶段切换
race/race_primitives.h # 入弯、出弯、强制找线、航向转向等运动原语
race/race_log.h        # 任务三/四 RAM 日志
race/task2_ram_log.h   # 任务二 RAM 日志
tasks/task_sequences.h # 任务一/二顶层序列
tasks/task_dispatcher.h# 任务调度器
BSP/                   # TB6612、红外、JY62、编码器驱动
Board/                 # UART printf、基础 delay
tools/                 # 日志后处理工具
docs/                  # 协作文档、调试说明、模块说明
```

## 构建

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug all
```

clean build：

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug clean all
```

不要手动修改 `Debug/` 下的 SysConfig/编译生成物、`.out`、`.map`、`.obj`。

## 调参入口

当前主要调参文件是 [app_config.h](app_config.h)。

| 参数族 | 用途 |
|---|---|
| `STRAIGHT_*` | 全局直线基础 PWM、直线 yaw 辅助修正 |
| `TASK1_*` | 任务一 A->B 直线 |
| `TASK2_*` | 任务二 AB/CD 直线、BC/DA 弧线、Task2 RAM 日志 |
| `TASK3_*` | 任务三几何、弧线、直线搜索、点位声光 |
| `TASK4_*` | 任务四圈数和起步兼容参数 |
| `RACE_*` | 任务三/四共享竞速路径实际控制参数 |
| `RACE_TASK4_*` | 任务四单独高速参数 |

`PID_TEST_*` 和 `PD_TEST_*` 目前只是历史命名的共享差速闭环参数，不再对应独立 UART 调试任务。

## 日志

- `TASK2_RAM_LOG_ENABLE`：任务二 RAM 日志。
- `RACE_UART_LOG_ENABLE`：任务三/四实时文本日志，实车高速时谨慎开启。
- `RACE_RAM_LOG_ENABLE`：任务三/四 RAM dump，当前复盘首选。

日志整理脚本仍在 [tools/task11_log_to_csv.py](tools/task11_log_to_csv.py)。文件名保留历史名称，但已兼容当前 `RACE_*` RAM 日志。

## 文档入口

| 文档 | 内容 |
|---|---|
| [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md) | 验收版串口命令、跑车流程、日志字段 |
| [docs/任务三调试说明.md](docs/任务三调试说明.md) | 任务三路线和注意事项 |
| [docs/任务四调试说明.md](docs/任务四调试说明.md) | 任务四竞速参数和 RAM 日志复盘 |
| [docs/app_config宏定义说明.md](docs/app_config宏定义说明.md) | 当前宏分类说明 |
| [docs/代码结构与协作说明.md](docs/代码结构与协作说明.md) | 模块边界和协作注意事项 |

## Push 前检查

1. 运行 `gmake -C Debug all`。
2. 确认 UART ready 文案只显示 `01..04`。
3. 确认日志/CSV/Debug 生成物没有误加入 Git。
4. 若改动任务三/四参数，优先保留一份实车日志用于回退。
