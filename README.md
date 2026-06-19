# SDU NUEDC Training

2026 校赛 F 题自动驾驶小车固件。工程基于 TI MSPM0G3507、CCS + SysConfig + DriverLib，控制对象是差速小车、八路红外循迹、JY62 惯导、TB6612 电机驱动和 ST011 声光模块。

当前版本日期：2026-06-16。

## 当前状态

这一版已经完成了一轮结构重构：`main.c` 不再承载大段控制实现，只保留系统启动、任务分发、任务一/二入口、调试入口和中断入口。任务三/任务四统一走 `run_race_laps()`，实现放在 `race/race_laps.h`；历史经验采集逻辑已经并入当前竞速路径，配置宏已统一改为 `RACE_*` 命名。

为避免 CCS 生成 makefile 不自动纳入新增 `.c` 文件，本项目当前新增业务模块采用 header-only 方式集成。后续如果要拆成 `.c/.h`，需要先确认 CCS 工程配置会把新源文件加入编译和链接。

## 硬件摘要

| 模块 | 说明 |
|---|---|
| 主控 | TI MSPM0G3507 |
| 电机驱动 | TB6612，A/B 两路差速 |
| 编码器 | A 轮 `PA16/PA17`，B 轮 `PA14/PA15` |
| 红外 | 八路红外循迹模块，I2C 读取 |
| 惯导 | JY62，UART1，115200 |
| 声光 | ST011，低电平触发 |
| 任务按键 | `A26/A24/B24/A22` 分别触发任务一/二/三/四 |
| 调试串口 | UART0，115200 8N1 |

接线细节见 [暂行接线图.md](暂行接线图.md)。

## 任务入口

| 按键 / UART0 | 任务 | 当前入口 |
|---|---|---|
| `A26` / `01` | 任务一，A -> B 直线 | `run_task1_ab()` |
| `A24` / `02` | 任务二，A -> B -> C -> D -> A | `run_task2_abcd()` |
| `B24` / `03` | 任务三，A -> C -> B -> D -> A | `run_race_laps(1U)` |
| `A22` / `04` | 任务四，竞速路径 4 圈 | `run_race_laps(4U)` |
| UART0 `05` | 轮速 / PID 流式测试 | `run_motor_pid_stream()` |
| UART0 `06` | A -> C 后快速转向测试 | `run_task6_ac_c_turn_test()` |
| UART0 `07` | 轮速 / PD 流式测试 | `run_motor_pd_stream()` |
| UART0 `10` | AB 零角度标定与航向监控 | `run_task10_ab_zero_test()` |
| UART0 `00` | 运行中强制停车 | `TASK_ID_STOP` |

串口命令同时支持二进制字节和文本数字。也就是说发送 HEX `03` 或 ASCII `03` 都可以启动任务三。

任务三和任务四在按键或 UART0 启动时，会先触发一次 ST011 声光模块作为起步确认；触发入口在 `tasks/task_dispatcher.h` 的 `prepare_task_start()`，脉冲时长由 `RACE_START_ALARM_MS` 控制。

## 代码结构

```text
.
├── main.c                 # 启动、任务分发、任务一/二入口、中断入口
├── app_config.h           # 速度、PID、航向、距离、日志开关等核心参数
├── app_control.h          # 通用限幅、角度归一化、直线 PID 数据结构
├── app_motion_utils.h     # 运动相关小工具
├── app_services.c/.h      # UART 打印、延时、声光、任务停止请求等服务
├── app_straight.h         # 通用差速直行控制
├── app_task_ids.c/.h      # UART/按键任务命令解析
├── app_debug_modes.h      # 05/07 等调试模式
├── straight/
│   └── straight_line.h    # 任务一/二等直线行驶封装
├── tasks/
│   ├── task_dispatcher.h  # 任务调度、启动前准备和任务入口分发
│   ├── task_sequences.h   # 任务一/二序列入口
│   └── task6_turn_test.h  # C 点快速转向测试
├── heading/
│   └── heading_straight.h # 航向直线到线段控制
├── turn/
│   ├── line_fast_turn.h   # C/D 点快速转向测试和转向交接
│   └── arc_segment.h      # Task2 竞速弧线段
├── race/
│   ├── task2_ram_log.h    # Task2 RAM 日志
│   ├── race_log.h         # 任务三/四竞速 RAM 日志
│   ├── race_laps.h        # 任务三/四统一跑圈流程
│   ├── race_primitives.h  # 竞速转向、前进、航向计算等基础动作
│   └── race_phase.h       # 竞速阶段状态更新、控制量计算和阶段切换
├── Board/                 # UART 打印、delay 等板级工具
├── BSP/                   # TB6612、红外、JY62、编码器驱动
├── tools/
│   └── task11_log_to_csv.py
└── docs/
    └── 协作文档和调试说明
```

更详细的模块边界见 [docs/代码结构与协作说明.md](docs/代码结构与协作说明.md)。

## 构建

常用构建命令：

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug all
```

clean build：

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug clean all
```

注意：

- `main.syscfg` 是管脚、时钟、中断和外设配置源头。
- 不要手动修改 `Debug/ti_msp_dl_config.c`、`Debug/ti_msp_dl_config.h`、`Debug/makefile`、`.out`、`.map` 等生成物。
- clean 阶段偶尔会提示找不到旧的 `.d` 依赖文件，只要最终退出码为 0 且目标文件生成，就不算构建失败。

## 关键参数入口

当前调参主要集中在 [app_config.h](app_config.h)。

| 参数族 | 作用 |
|---|---|
| `STRAIGHT_*` | 全局直线基础 PWM、最小/最大 PWM |
| `TASK1_*` | 任务一 A->B 直线、停止线、起步斜坡、航向修正 |
| `TASK2_*` | 任务二 AB/CD 直线、BC/DA 弧线、Task2 RAM 日志 |
| `TASK3_*` | 任务三几何、弧线、直线搜索、点位声光 |
| `TASK4_*` | 任务四多圈逻辑和历史兼容参数 |
| `TASK6_*` | C 点快速转向测试 |
| `RACE_*` | 当前竞速路径参数，实际由 `race_*` 逻辑使用 |

几个当前重要值：

```c
#define STRAIGHT_B_BASE_PWM (628)
#define STRAIGHT_A_BASE_PWM (633)
#define RACE_TASK3_AC_HEADING_TARGET_CDEG (-3300)
#define RACE_TASK3_BD_HEADING_TARGET_CDEG (-18000 + 3850)
#define RACE_TASK4_AC_HEADING_TARGET_CDEG (-3300)
#define RACE_TASK4_BD_HEADING_TARGET_CDEG (-18000 + 3800)
#define RACE_STRAIGHT_BASE_PWM      (600)
#define RACE_TASK4_STRAIGHT_BASE_PWM (840)
#define RACE_ARC_BASE_PWM           (540)
#define RACE_TASK4_ARC_BASE_PWM     (670)
#define RACE_START_ALARM_MS         (RACE_POINT_ALARM_MS)
#define RACE_AC_POINT_ARM_COUNT     (7300)
#define RACE_BD_POINT_ARM_COUNT     (7300)
#define RACE_TASK4_BD_POINT_ARM_COUNT (6600)
#define RACE_TASK4_BD_FORCE_TURN_COUNT (7200)
```

宏说明见 [docs/app_config宏定义说明.md](docs/app_config宏定义说明.md)。

## 任务三/四路径

任务三和任务四共用 `race/race_laps.h` 的竞速跑圈逻辑：

```text
AC 直线 -> CB 左弧线 -> BD 直线 -> DA 右弧线
```

控制要点：

- 起步提示：任务三/四启动时使用 `st011_start_pulse(RACE_START_ALARM_MS)` 非阻塞触发一次声光模块。
- AC/BD 直线：以 JY62 航向目标为主，红外用于点位检测和辅助记录；任务四有独立高速 PWM、BD 点位 arm 和 BD 强制入弯距离。
- CB/DA 弧线：以红外循迹和差速控制为主，可记录 JY62 航向误差。
- C/D 点：通过红外线判定进入转向动作。
- B/A 点：弧线出口后执行到目标航向的转向动作。
- 任务四跑 4 圈，并在任务三稳定参数外叠加更高速度、入弯/出弯保护和 RAM 日志分析参数。

调试说明见 [docs/任务三调试说明.md](docs/任务三调试说明.md) 和 [docs/任务四调试说明.md](docs/任务四调试说明.md)。

## 日志

常用串口字段：

| 字段 | 含义 |
|---|---|
| `dist` / `phase_dist` | 当前段里程计数 |
| `yaw` / `rel_cdeg` | JY62 相对航向，单位 0.01 度 |
| `raw` / `mask` / `cnt` | 红外原始值、黑线掩码、命中数量 |
| `err` / `filt` / `der` | 红外误差、滤波误差、误差变化 |
| `line_turn` / `nav_turn` / `turn` | 红外修正、惯导修正、最终转向量 |
| `tdiff` / `ff` / `fb` / `corr` | 差速目标、前馈、反馈和合成修正 |
| `pwm=B/A` | B 轮和 A 轮 PWM |
| `reason=point` | 正常点位触发 |
| `reason=force` | 距离保护触发 |
| `reason=uart_stop` | 串口停止触发 |

日志开关：

- `TASK2_RAM_LOG_ENABLE`：Task2 RAM 日志。
- `RACE_UART_LOG_ENABLE`：任务三/四实时 `RACE_*` 文本日志。
- `RACE_RAM_LOG_ENABLE`：任务三/四 `RACE_RAM_BEGIN` 到 `RACE_RAM_END` 的 RAM dump。

日志整理脚本见 [tools/task11_log_to_csv.py](tools/task11_log_to_csv.py)，用法已并入 [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md) 的“日志整理”章节。

## 文档入口

| 文档 | 内容 |
|---|---|
| [docs/代码结构与协作说明.md](docs/代码结构与协作说明.md) | 当前模块边界、header-only 约定、协作注意事项 |
| [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md) | 串口命令、跑车流程、日志字段 |
| [docs/任务三调试说明.md](docs/任务三调试说明.md) | 任务三路径、点位、常见问题 |
| [docs/任务四调试说明.md](docs/任务四调试说明.md) | 任务四多圈路径、误差积累分析 |
| [docs/app_config宏定义说明.md](docs/app_config宏定义说明.md) | 调参宏索引 |
| [docs/Board模块说明.md](docs/Board模块说明.md) | UART 打印、延时和板级工具说明 |
| [docs/TB6612驱动使用说明.md](docs/TB6612驱动使用说明.md) | 电机驱动接线、接口和方向校准 |
| [docs/JY62陀螺仪驱动说明.md](docs/JY62陀螺仪驱动说明.md) | 惯导接线、协议解析、任务日志和排错 |
| [docs/八路红外循迹模块驱动说明.md](docs/八路红外循迹模块驱动说明.md) | 红外 I2C 读取、mask/err 含义和测试流程 |

## Push 前检查

1. 确认 `app_config.h` 中日志开关是否符合要留档的版本。
2. 运行 `gmake -C Debug all` 或在 CCS 中完整编译。
3. 确认 `Debug/`、`.out`、`.map`、本地串口日志、CSV 数据没有误加入 Git。
4. 说明本次改动是控制参数变化、结构重构，还是文档更新。
