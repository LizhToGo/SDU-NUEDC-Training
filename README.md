# SDU NUEDC Training

2026 校赛 F 题自动行驶小车固件。工程基于 TI MSPM0G3507、CCS + SysConfig + DriverLib，控制对象是差速小车、八路红外循迹模块、JY62 惯导、TB6612 电机驱动和 ST011 声光模块。

当前代码已经收敛到验收模式：固件只保留任务一、任务二、任务三、任务四入口。历史调试任务 05/06/07/08/10、Task6/Task8 测试头文件、轮速/红外调试模式入口均已从源码中移除。

## 开源状态

源码许可证：MIT License，见 [LICENSE](LICENSE)。

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
race/race_laps.h       # 任务三/四跑圈主流程；任务二 BC/DA 复用这里的竞速弧线阶段
race/race_phase.h      # 竞速阶段状态、点位判定、阶段切换
race/race_primitives.h # 入弯、出弯、强制找线、航向转向等运动原语
tasks/task_sequences.h # 任务一/二顶层序列
tasks/task_dispatcher.h# 任务调度器
BSP/                   # TB6612、红外、JY62、编码器驱动
Board/                 # UART printf、基础 delay
docs/                  # 协作文档、调试说明、模块说明
```

## 构建

```powershell
gmake -C Debug all
```

clean build：

```powershell
gmake -C Debug clean all
```

不要手动修改 `Debug/` 下的 SysConfig/编译生成物、`.out`、`.map`、`.obj`。

## 调参入口

当前主要调参文件是 [app_config.h](app_config.h)。

| 参数族 | 用途 |
|---|---|
| `STRAIGHT_*` | 全局直线基础 PWM、直线 yaw 辅助修正 |
| `TASK1_*` | 任务一 A->B 直线 |
| `TASK2_*` | 任务二点位声光、CD 固定航向和 CD 到线使能 |
| `TASK3_*` | 任务三几何、弧线、直线搜索、点位声光 |
| `TASK4_*` | 任务四圈数和起步兼容参数 |
| `RACE_*` | 任务三/四共享竞速路径实际控制参数 |
| `RACE_TASK4_*` | 任务四单独高速参数 |

`PID_TEST_*` 和 `PD_TEST_*` 目前只是历史命名的共享差速闭环参数，不再对应独立 UART 调试任务。

## 日志

- 任务一/二默认输出直线或弧线段的串口文本诊断。
- `RACE_UART_LOG_ENABLE`：任务三/四实时文本日志开关，实车高速时默认关闭，短时定位问题时再打开。
- RAM 缓存日志和离线整理脚本已经删除，复盘以串口文本、视频、手工计时和实车成功率为准。

## 文档入口

| 文档 | 内容 |
|---|---|
| [docs/串口调试与跑车流程.md](docs/串口调试与跑车流程.md) | 验收版串口命令、跑车流程、日志字段 |
| [docs/app_config宏定义说明.md](docs/app_config宏定义说明.md) | 当前宏分类说明 |
| [docs/硬件接线说明.md](docs/硬件接线说明.md) | 当前硬件模块接线和 SysConfig 对应关系 |
