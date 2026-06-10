# SDU NUEDC Training：2026 校赛 F 题自动行驶小车

本仓库用于 2026 信息科学与工程学院电子设计竞赛选拔赛 F 题「自动行驶小车」的固件、接线文档、调试工具和协作说明。当前工程基于 TI MSPM0G3507，使用 CCS + SysConfig + DriverLib 开发。

项目思路参考 [ZhijianLi2003/ZLC_MSPM0_Peripheral_Library](https://github.com/ZhijianLi2003/ZLC_MSPM0_Peripheral_Library)，但硬件接线、任务逻辑、控制参数和日志体系已经按本队小车重新适配。

## 当前版本状态

日期：2026-06-08

当前 `main` 分支相对早期版本变化很大：任务十、任务十一重新成为经验数据采集入口，任务三/四的 AC、BD 直线目标改为固定绝对航向，弧线段接入 Task11 风格的差速控制、红外 PD 和 JY62 航向修正。由于任务三/四使用绝对航向，跑 `03/04` 前建议先让车头朝 AB 方向发送 `10` 完成零航向标定。

| 入口 | 任务 | 当前状态 |
|---|---|---|
| `A26` / UART0 `01` | 任务一 A -> B 直线 | 使用差速直线模块，实车直线参数以当前 `app_config.h` 为准 |
| `A24` / UART0 `02` | 任务二 A -> B -> C -> D -> A | 调试态，AB/CD 直线与 Task11 风格弧线组合 |
| `B24` / UART0 `03` | 任务三 A -> C -> B -> D -> A | 走 `run_task11_ir_map_test_laps(1U)`，Task11 专用航向目标 |
| `A22` / UART0 `04` | 任务四，按任务三路线连续 4 圈 | 走 `run_task11_ir_map_test_laps(4U)`，Task11 专用航向目标 |
| UART0 `05` | 轮速 / 编码器 / PWM PID 测试 | 可用 |
| UART0 `06` | A -> C 后 C 点快速左转测试 | 可用，用于测试 C 点入弯快转角度 |
| UART0 `07` | 轮速 / 编码器 / PWM PD 测试 | 可用 |
| UART0 `10` | Task10，AB 方向零航向标定+持续监控 | 可用，车头朝 AB 放置后执行 |
| UART0 `11` | Task11，全程黑线经验数据采集 | 可用，默认关闭大体量日志输出 |
| UART0 `00` | 强制停车 | 运行中可用 |

串口可以发送 ASCII `01`、`10`、`11`，也可以发送原始 HEX 字节 `0x01..0x07`、`0x10`、`0x11`。运行中发送 `00` 或原始 `0x00` 会请求停车。

## 硬件摘要

| 模块 | 当前方案 |
|---|---|
| 主控 | 立创·天猛星 MSPM0G3507 开发板 |
| 底盘 | 万向轮 + 后驱差速 |
| 电机驱动 | TB6612 双路电机驱动模块 |
| 编码器 | 左右后轮霍尔编码器 |
| 循迹 | 八路红外循迹模块，I2C 读取 |
| 姿态 | JY62 陀螺仪，UART1 接收 |
| 按键 | `A26/A24/B24/A22` 分别触发任务一/二/三/四 |
| 声光 | ST011 声光模块，低电平触发 |
| 调试串口 | UART0，`115200 8N1` |

完整接线请看 [暂行接线图.md](暂行接线图.md)。

## 仓库结构

```text
.
├── README.md
├── main.c                         # 主入口、任务调度、任务一二三四、任务六、任务十/十一
├── app_config.h                   # PWM、PID、阈值、几何尺寸、任务参数
├── app_control.h                  # 限幅、斜坡、直线 PID、航向滤波
├── app_straight.h                 # 差速直线/弧线共用的轮速差控制模块
├── app_debug_modes.h              # 05/07 轮速测试、红外打印、纯循迹测试
├── main.syscfg                    # SysConfig 外设、时钟、引脚配置源头
├── Board/                         # 串口打印、延时等通用工具
├── BSP/                           # TB6612、红外、JY62、编码器驱动
├── tools/
│   └── task11_log_to_csv.py       # Task11 串口日志转 CSV 工具
├── docs/                          # 协作文档、调试流程、模块说明
└── 暂行接线图.md
```

`Debug/`、`Release/`、`.out`、`.map`、本地编辑器配置和 Task11 导出的本地 CSV/TXT 经验数据不应提交到 GitHub。`.gitignore` 已加入 root 和 `data/` 下的 `task11_experience_*` 规则。

## 开发环境

| 工具 | 当前版本或路径特征 |
|---|---|
| CCS | `ccs2051` |
| MSPM0 SDK | `mspm0_sdk_2_10_00_04` |
| SysConfig | `1.27.1` |
| 编译器 | TI Arm Clang `4.0.4.LTS` |
| 主控型号 | MSPM0G3507 |

常用构建命令：

```powershell
C:\ti\ccs2051\ccs\utils\bin\gmake.exe -C Debug all
```

注意：

- `main.syscfg` 是引脚、外设、时钟、中断配置源头。
- 不要手动修改 `Debug/ti_msp_dl_config.c`、`Debug/ti_msp_dl_config.h`、`Debug/makefile` 等自动生成文件。
- `bsp_encoder.h` 和 `bsp_jy62.h` 目前仍以头文件内 `static` 实现方式集成，主要是为了避免改 CCS 自动生成 Makefile。

## 当前关键参数

基础直线和起步：

```c
#define STRAIGHT_B_BASE_PWM (626)
#define STRAIGHT_A_BASE_PWM (633)
#define STRAIGHT_MAX_PWM    (870)
#define TASK1_RAMP_B_START_PWM (560)
#define TASK1_RAMP_A_START_PWM (600)
```

任务三/四航向目标：

```c
#define TASK3_AC_HEADING_TARGET_CDEG  (-3660)
#define TASK3_BD_HEADING_TARGET_CDEG  (-13920)
#define TASK4_AC_HEADING_TARGET_CDEG  TASK3_AC_HEADING_TARGET_CDEG
#define TASK4_BD_HEADING_TARGET_CDEG  TASK3_BD_HEADING_TARGET_CDEG
```

说明：`TASK3_BD_RELATIVE_TURN_CDEG` 仍保留在 `app_config.h` 中作为历史/经验参数，但当前任务三和任务四实际使用的是 `mode=absolute` 的 AC/BD 固定目标角。这两个目标角默认相对 Task10 建立的 AB 零航向。

Task11 风格弧线和采集：

```c
#define TASK11_TARGET_LAPS              (5)
#define TASK11_LINE_BASE_PWM            (560)
#define TASK11_ARC_BASE_PWM             (540)
#define TASK11_CB_ARC_CRUISE_TARGET_DIFF (-44)
#define TASK11_DA_ARC_CRUISE_TARGET_DIFF (42)
#define TASK11_AC_POINT_ARM_COUNT       (8000)
#define TASK11_BD_POINT_ARM_COUNT       (7800)
```

几何参数：

```c
#define COUNTS_PER_CM                 (66)
#define TASK2_ARC_RADIUS_CM           (40)
#define TASK2_ARC_WHEEL_BASE_MM       (126)
#define TASK2_ARC_SENSOR_TO_AXIS_MM   (175)
#define TASK3_SENSOR_TO_AXIS_MM       (175)
```

## 当前任务逻辑

### 任务一

`TASK1_AB` 使用 `run_straight_to_line_segment()`：

1. A 点启动，声光提示。
2. 可选 JY62 当前航向置零。
3. `app_straight.h` 的差速控制模块根据左右轮速度差、累计距离差和 JY62 航向修正输出 PWM。
4. 到 `TASK1_B_LINE_ARM_COUNT` 后允许红外线触发。
5. 扫到 B 点线后停车并声光提示；若到 `TASK1_FORCE_STOP_COUNT` 仍未扫到线，则强制停止。

### 任务二

任务二当前是调试组合版本：

- `TASK2_AB`：复用任务一风格直线。
- `TASK2_BC`：使用 Task11 风格弧线调试函数。
- `TASK2_CD`：使用固定目标角直线，目标来自 `TASK2_CD_STRAIGHT_TARGET_CDEG`。
- `TASK2_DA`：使用 Task11 风格弧线。

任务二带有 `TASK2_RAM_*` 日志结构，便于弧线和 CD 出线分析。

### 任务三

任务三当前走 `run_task11_ir_map_test_laps(1U)`，即 Task11 单圈模式。

```text
A -> C -> B -> D -> A
```

当前实现：

- `TASK11_AC`：A 到 C，目标航向 `TASK11_AC_HEADING_TARGET_CDEG=-50`。
- `TASK11_CB`：C 到 B，使用 Task11 弧线控制（差速+红外），点位判定改为 `line_lost_seen`。
- B 点出口转向从红外 `sensor_fast_turn` 改为陀螞仪 `gyro_turn_to_yaw`，使用 `TASK11_B_EXIT_TARGET_CDEG`。
- `TASK11_BD`：B 到 D，目标航向 `TASK11_BD_HEADING_TARGET_CDEG=-10638`。
- `TASK11_DA`：D 到 A，使用 Task11 弧线控制，A 点出口转向同样改为 `gyro_turn_to_yaw`。

跑任务三前，建议先用 Task10 建立 AB 零航向，再把车放到 A 点、车头朝 C 启动。详细说明见 [docs/任务三调试说明.md](docs/任务三调试说明.md)。

### 任务四

任务四走 `run_task11_ir_map_test_laps(4U)`，连续 4 圈 Task11 路径：

```text
TASK4_L1_AC -> TASK4_L1_CB -> TASK4_L1_BD -> TASK4_L1_DA
...
TASK4_L4_DA
```

前三圈 A 点出线后不结束，继续下一圈；第四圈回到 A 点后刹车并触发结束声光。B/A 点出口转向均使用 `gyro_turn_to_yaw`，按绝对航向停止。

详细说明见 [docs/任务四调试说明.md](docs/任务四调试说明.md)。

### 任务十和任务十一

- `10`：Task10，把当前 JY62 相对航向置零，用于 AB 方向标定。标定后进入持续航向监控模式，每 200ms 打印航向状态。
- `11`：Task11，全程黑线经验数据采集。车头从 A 指向 C，按 `AC -> CB -> BD -> DA` 跑 5 圈，用于估计任务三/四可复用的距离、航向、转向和点位识别参数。

当前 `TASK11_UART_LOG_ENABLE=0`、`TASK11_RAM_LOG_ENABLE=0`，默认不输出大体量采集日志。需要采集时在 `app_config.h` 中打开对应开关，再用 [tools/task11_log_to_csv.py](tools/task11_log_to_csv.py) 整理日志。

## 串口日志

协作调参时优先保存这些片段：

- `BOOT: ...`
- `TASK ready ...`
- `TASK UART command accepted: id=...`
- 每段 `start`
- 周期日志中的 `dist/yaw/h_tgt/h_corr/raw/mask/cnt/err/turn/pwm`
- 每段 `stop: reason=...`
- `TASK11_RAM_BEGIN` 到 `TASK11_RAM_END`，如果打开了 Task11 RAM 日志

常见字段：

| 字段 | 含义 |
|---|---|
| `dist` | 当前段编码器距离估计 |
| `yaw/rel_cdeg` | JY62 相对航向，单位 0.01 度 |
| `raw/mask/cnt/lost` | 红外原始值、黑线掩码、触发探头数、是否丢线 |
| `err/filt/der` | 红外误差、滤波误差、差分项 |
| `line_turn/nav_turn/turn` | 红外、陀螺仪和最终转向量 |
| `tdiff/ff/fb/corr` | 差速目标、前馈、反馈和总修正 |
| `B_pwm/A_pwm` 或 `pwm=B/A` | 左轮 B、右轮 A 输出 PWM |
| `reason=line/point` | 红外或任务点触发 |
| `reason=force` | 距离兜底触发 |
| `reason=uart_stop` | 串口停车 |

更完整的字段表见 [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md)。

## 文档入口

- [暂行接线图.md](暂行接线图.md)：当前硬件接线和电源共地说明。
- [docs/当前调试状态与后续问题.md](docs/当前调试状态与后续问题.md)：当前主分支状态、关键参数、已知问题和后续调试建议。
- [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md)：UART0 命令、日志字段、判断方法。
- [docs/项目实现思路.md](docs/项目实现思路.md)：软件分层和控制策略总览。
- [docs/任务三调试说明.md](docs/任务三调试说明.md)：任务三当前绝对航向版本的分段逻辑。
- [docs/任务四调试说明.md](docs/任务四调试说明.md)：任务四四圈流程和调试重点。
- [docs/task11_log_to_csv_usage.md](docs/task11_log_to_csv_usage.md)：Task11 日志整理脚本使用方式。
- [docs/2026-06-07_task11_work_summary.md](docs/2026-06-07_task11_work_summary.md)：Task11 经验采集阶段历史交接记录。

## Push 前检查

```powershell
git status --short
C:\ti\ccs2051\ccs\utils\bin\gmake.exe -C Debug all
```

建议确认：

- 只提交源码、`.syscfg`、工具脚本和文档。
- 不提交 `Debug/`、`Release/`、`.out`、`.map`。
- 不提交本地代理配置和 Task11 经验导出文件。
- 任务参数发生变化时同步更新 README 和 `docs/当前调试状态与后续问题.md`。
