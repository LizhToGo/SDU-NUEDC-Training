# SDU NUEDC Training：2026 校赛 F 题自动行驶小车

本仓库用于 2026 信息科学与工程学院电子设计竞赛选拔赛 F 题「自动行驶小车」的固件、接线文档和调试记录。当前工程基于 TI MSPM0G3507，使用 CCS + SysConfig + DriverLib 开发。

项目思路参考 [ZhijianLi2003/ZLC_MSPM0_Peripheral_Library](https://github.com/ZhijianLi2003/ZLC_MSPM0_Peripheral_Library)，但硬件接线、任务逻辑和控制参数已经按本队小车做了适配。

## 当前版本状态

日期：2026-06-01

当前准备 push 的版本已经接入四个实体任务按键和 UART0 命令：

| 入口 | 任务 | 当前状态 |
|---|---|---|
| `A26` / UART0 `01` | 任务一 A -> B 直线 | 已实测稳定 |
| `A24` / UART0 `02` | 任务二 A -> B -> C -> D -> A | 可运行，继续调弧线稳定性 |
| `B24` / UART0 `03` | 任务三 A -> C -> B -> D -> A | 基础可跑，`B->D` 相对转角留档为 67 度 |
| `A22` / UART0 `04` | 任务四，按任务三路线连续 4 圈 | 基础可跑，留档后继续调稳定性和用时 |
| UART0 `05` | 轮速 / 编码器 / PWM 测试 | 可用 |
| UART0 `00` | 强制停车 | 可用 |

当前重点留档内容：

- 直线基础 PWM：`B=532`，`A=543`。
- 任务一使用编码器速度 PID、累计距离修正和 JY62 航向修正直行，到 B 点红外黑线停车。
- 任务二使用复用直线段 + 任务二弧线逻辑。
- 任务三使用任务三专用弧线红外循迹逻辑，不复用任务二弧线逻辑。
- 任务三 B 点出线后不刹车，触发声光后直接进入 B->D，按 B 点实际出线航向做相对转向。
- 任务四复用任务三路线，前三圈 A 点出线不刹车，按 A 点实际出线航向等大反向转入下一圈 AC，第四圈 A 点停车；当前属于勉强可跑的留档版本。

最新调试状态请看 [docs/当前调试状态与后续问题.md](docs/当前调试状态与后续问题.md)。

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
├── main.c                         # 主入口、任务调度、任务一/二/三/四流程
├── app_config.h                   # PWM、PID、阈值、任务参数
├── app_control.h                  # PID、航向滤波、限幅等控制辅助函数
├── app_debug_modes.h              # 05 轮速测试、红外测试、纯循迹测试
├── main.syscfg                    # SysConfig 外设、时钟、引脚配置源头
├── Board/
│   ├── board.h
│   └── board.c                    # 串口打印、延时等通用工具
├── BSP/
│   ├── inc/
│   │   ├── bsp_encoder.h          # 编码器，当前以头文件 static 方式集成
│   │   ├── bsp_ir_tracking.h
│   │   ├── bsp_jy62.h             # JY62，当前以头文件 static 方式集成
│   │   └── bsp_tb6612.h
│   └── src/
│       ├── bsp_ir_tracking.c
│       └── bsp_tb6612.c
├── docs/
│   ├── 当前调试状态与后续问题.md
│   ├── 串口调试与跑车流程.md
│   ├── 任务三调试说明.md
│   ├── 任务四调试说明.md
│   ├── 项目实现思路.md
│   ├── Board模块说明.md
│   ├── JY62陀螺仪驱动说明.md
│   ├── TB6612驱动使用说明.md
│   └── 八路红外循迹模块驱动说明.md
└── 暂行接线图.md
```

`Debug/`、`Release/`、`.out`、`.map`、本地编辑器配置和编译产物不提交到 GitHub。

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
- 现在 `bsp_encoder.h` 和 `bsp_jy62.h` 为了避免改 CCS 自动生成 Makefile，采用头文件内 `static` 实现，只在 `main.c` 中 include 一次。

## 当前任务逻辑

### 任务一

```text
TASK1_AB: A -> B 直线
```

流程：

1. 按 A26 或发送 `01`。
2. 声光提示。
3. 延时后 JY62 当前航向置零。
4. 编码器速度 PID + 累计距离修正 + JY62 航向修正直行。
5. 到 `TASK1_B_LINE_ARM_COUNT` 后允许红外黑线停车。
6. 识别 B 点黑线后刹车并声光提示。
7. 若没有识别到线，到 `TASK1_FORCE_STOP_COUNT` 强制停车。

### 任务二

```text
TASK2_AB: A -> B 直线
TASK2_BC: B -> C 半圆弧
TASK2_CD: C -> D 直线
TASK2_DA: D -> A 半圆弧
```

任务二弧线使用 `run_arc_line_follow_segment()`，包含编码器差速前馈、红外误差修正和 JY62 弧线进度判断。任务二仍处于调参状态。

### 任务三

```text
TASK3_AC: A -> C 直线
TASK3_CB: C -> B 任务三专用红外弧线
TASK3_BD: B -> D 相对航向直线
TASK3_DA: D -> A 任务三专用红外弧线
```

任务三当前实现要点：

- 起点把车摆在 A 点，车头朝 C。
- A 点启动时 JY62 置零。
- AC 和 BD 复用直线段控制。
- CB 和 DA 使用任务三专用 `run_task3_arc_line_follow_segment()`。
- B 点出线由红外出线事件触发，触发声光后不刹车，直接进入 BD 边走边转。
- BD 目标角使用 `B 点实际出线航向 + TASK3_BD_RELATIVE_TURN_CDEG`，当前相对转角为 `6700`，即 67.00 度。
- A 点最终由 DA 段红外出线触发停车和结束声光。

任务三详细说明和串口字段见 [docs/任务三调试说明.md](docs/任务三调试说明.md)。

### 任务四

```text
TASK4: 按任务三 A -> C -> B -> D -> A 路线连续 4 圈
```

任务四当前实现要点：

- 第一圈从 A 点启动，和任务三一样把当前车头朝 C 的方向作为 JY62 零角度。
- 每圈 C、B、D 点由对应入线/出线事件触发声光。
- 每圈 B 点沿用任务三逻辑：`B_exit + TASK3_BD_RELATIVE_TURN_CDEG`，当前为 `+6700`。
- 前三圈 A 点出线后不刹车，只触发声光，然后用 `A_exit + TASK4_AC_RELATIVE_TURN_CDEG` 转入下一圈 AC。
- `TASK4_AC_RELATIVE_TURN_CDEG` 当前等于 `-6700`，与 B 点转向等大反向。
- 第四圈 DA 到 A 点后刹车并触发结束声光。

## 串口日志

协作调参时优先保存以下日志片段：

- `TASK ready ...`
- `TASK*_start`
- 每段 `t=...` 周期日志
- 每段 `stop: reason=...`
- `abort after ...`
- `JY62 mode=...`

常见字段快速判断：

| 字段 | 含义 |
|---|---|
| `dist` | 当前段编码器距离估计 |
| `raw/mask/cnt/lost` | 红外原始值、黑线掩码、触发探头数、是否丢线 |
| `h_raw/h_tgt/h_corr` | 当前航向、目标航向、航向修正 |
| `B_total/A_total` | 左右轮当前段累计编码器计数 |
| `B_spd/A_spd` | 左右轮当前周期速度估计 |
| `v_err/P/I/D/v_corr` | 速度 PID 状态 |
| `corr/B_pwm/A_pwm` | 最终修正量和输出 PWM |
| `reason=line` | 红外线触发到点 |
| `reason=force` | 编码器距离兜底停止 |
| `reason=arc_done` | 弧线出线完成 |

更完整的字段表见 [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md)。

## 文档入口

- [暂行接线图.md](暂行接线图.md)：当前硬件接线和电源共地说明。
- [docs/当前调试状态与后续问题.md](docs/当前调试状态与后续问题.md)：准备 push 前的可用状态、关键参数、已知问题和后续调试计划。
- [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md)：UART0 命令、串口字段、日志判断方法。
- [docs/任务三调试说明.md](docs/任务三调试说明.md)：任务三分段逻辑、B 点出线、BD 转向、弧线字段。
- [docs/任务四调试说明.md](docs/任务四调试说明.md)：任务四四圈流程、A 点换圈转角和调参日志。
- [docs/项目实现思路.md](docs/项目实现思路.md)：软件分层和控制策略总览。
- [docs/JY62陀螺仪驱动说明.md](docs/JY62陀螺仪驱动说明.md)：JY62 接线、协议和驱动说明。
- [docs/八路红外循迹模块驱动说明.md](docs/八路红外循迹模块驱动说明.md)：I2C 协议、数据格式、误差计算。
- [docs/TB6612驱动使用说明.md](docs/TB6612驱动使用说明.md)：电机驱动函数和方向约定。
- [docs/Board模块说明.md](docs/Board模块说明.md)：串口打印、日志和延时函数说明。

## Push 前检查

```powershell
git status --short
C:\ti\ccs2051\ccs\utils\bin\gmake.exe -C Debug all
```

建议确认：

- 只提交源码、`.syscfg` 和文档。
- 不提交 `Debug/`、`Release/`、`.out`、`.map`。
- 任务参数发生变化时同步更新 README 和 `docs/当前调试状态与后续问题.md`。
- 实车调参日志尽量随 commit 或 issue 附上关键片段，便于回溯。
