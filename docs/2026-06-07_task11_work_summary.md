# 2026-06-07 Task11 工作交接说明

> 本文是 2026-06-07 当天的历史交接记录，不等同于当前 `main` 分支状态。当前状态请优先看 [当前调试状态与后续问题.md](当前调试状态与后续问题.md)、[项目实现思路.md](项目实现思路.md) 和 README。

本文整理 2026-06-07 围绕 MSPM0 小车 Task11 所做的调试、代码改动、日志体系调整和后续待办。当前核心目标仍是：用 Task11 积累任务 3/4 可复用的经验数据，为之后去掉 AC、BD 间黑胶布、改用陀螺仪导航和数学前馈做准备。

## 工作目标

1. 让 Task11 成为任务 3/4 的数据采集和控制试验平台。
2. 在保留循迹传感器点位识别的同时，让陀螺仪参与 AC、BD 直线导航和 CB、DA 弧线姿态修正。
3. 解决串口日志阻塞运动控制的问题，改为运动期间 RAM 记录，任务结束后分段慢速输出。
4. 积累每段航向、转向、传感器 mask、轮孔计数等数据，用于后续生成任务 3/4 的经验参数和数学前馈。
5. 暂时只把控制改动落到 Task11，任务 3/4 仍作为未来迁移目标。

## 当前仓库改动概览

已修改的主要文件：

- `main.c`
  - 串口任务解析。
  - JY62 零航向和导航数据读取。
  - Task11 RAM 日志结构、输出内容、阶段控制、点位判断、快转逻辑。
- `app_config.h`
  - Task11 关键参数、日志容量、陀螺仪介入开关、点位门槛。
- `Board/board.c`
  - `lc_printf` / `LOG_Debug_Out` 使用静态 UART 格式化缓冲，降低栈压力。
- `tools/task11_log_to_csv.py`
  - 解析 Task11 新日志字段并增量写入本地 CSV。
- `docs/task11_log_to_csv_usage.md`
  - 补充日志脚本字段说明和使用说明。
- `data/task11_experience_*.csv`
  - Task11 经验数据文件有更新，属于数据积累产物。

注意：`data/task11_experience_data.csv.bak*` 当前在工作区显示为删除状态，这是数据文件清理/脚本运行过程留下的工作区状态，不是控制代码改动。

## 串口任务选择

今天把串口任务选择整理为二进制任务号方案：

- `0x00`：停止当前任务。
- `0x01`：任务 1。
- `0x0A`：任务 10，AB 方向零航向标定。
- `0x0B`：任务 11。

相关逻辑在 `main.c` 的 `task_uart_read_command()`。运行中允许 `0x00` 立即作为 stop，且会清空当前解析状态，避免之前 `00` 无法退出的问题。

之前为了排查串口误识别曾添加过串口接收字节回显，今天已去掉回显相关代码，避免串口输出额外占用时间。

## Task10 零航向

Task10 当前用于把车头朝向 AB 方向时的 JY62 相对航向置零。正常串口日志形态类似：

```text
TASK UART command accepted: id=10
JY62 mode=TASK10_AB_ZERO t=0 df=0 ok=1 flags=0x00 yaw_cdeg=... rel_cdeg=0 ...
TASK10 zero_ab: ok=1 nav=1 rel=0 gzlp=...
```

如果 `ok=1 nav=1 rel=0`，说明零航向标定成功。Task11 后续输出的 `yaw` 使用相对航向角，`yaw_raw` 是 JY62 原始航向角。

## Task11 当前关键配置

当前 `app_config.h` 中 Task11 关键参数如下：

```c
#define TASK11_RAM_LOG_MAX_LAPS        (5)
#define TASK11_RAM_WINDOW_CAPACITY     (560)
#define TASK11_RAM_EVENT_CAPACITY      (112)
#define TASK11_RAM_SUMMARY_CAPACITY    (24)

#define TASK11_LINE_BASE_PWM           (560)
#define TASK11_ARC_BASE_PWM            (540)

#define TASK11_STRAIGHT_GYRO_NAV_ENABLE (1)
#define TASK11_STRAIGHT_IR_ASSIST_ENABLE (1)
#define TASK11_ARC_YAW_NAV_ENABLE      (1)

#define TASK11_CB_ARC_ENTRY_TARGET_DIFF  (-44)
#define TASK11_CB_ARC_CRUISE_TARGET_DIFF (-44)
#define TASK11_DA_ARC_ENTRY_TARGET_DIFF  (42)
#define TASK11_DA_ARC_CRUISE_TARGET_DIFF (42)
#define TASK11_DIFF_FF_GAIN            (2)

#define TASK11_EXIT_TURN_YAW_STOP_ENABLE (1)
#define TASK11_TURN_YAW_STOP_TOL_CDEG    (260)
#define TASK11_TURN_YAW_SLOW_ZONE_CDEG   (900)
#define TASK11_B_EXIT_TARGET_CDEG        TASK3_BD_HEADING_TARGET_CDEG
#define TASK11_A_EXIT_TARGET_CDEG        TASK3_AC_HEADING_TARGET_CDEG

#define TASK11_AC_POINT_ARM_COUNT      (8000)
#define TASK11_BD_POINT_ARM_COUNT      (7800)
#define TASK11_ARC_POINT_YAW_ARM_CDEG  (15000)
```

说明：

- `TASK11_STRAIGHT_GYRO_NAV_ENABLE=1`：AC、BD 直线段使用陀螺仪目标航向。
- `TASK11_STRAIGHT_IR_ASSIST_ENABLE=1`：直线段如果能看到 AC/BD 胶布，会叠加循迹修正；如果未来去掉胶布且 `line_valid=0`，仍会退回以陀螺仪导航为主。
- `TASK11_ARC_YAW_NAV_ENABLE=1`：弧线段使用循迹 + 弧线 yaw 模型修正。
- `TASK11_BD_POINT_ARM_COUNT=7800`：D 点检测门槛已单独提前；AC 仍为 8000。
- 用户最后提出“最短距离都提前一点”，该需求尚未统一落地。当前只完成了 BD 门槛从 8000 到 7800 的调整。

## RAM 日志体系

今天确认实时串口输出会干扰控制，尤其会造成 C/D 点识别错过。因此 Task11 运动期间主要写 RAM，结束后再分节慢速输出：

- `TASK11_RAM_BEGIN`
- `TASK11_CFG`
- `TASK11_DUMP_SECTION name=EVT`
- `TASK11_DUMP_SECTION name=SUM`
- `TASK11_DUMP_SECTION name=WIN`
- `TASK11_RAM_END`

日志容量当前为：

- `WIN`：560 条。
- `EVT`：112 条。
- `SUM`：24 条。

为了降低 SRAM 压力，容量从原先更大的配置收缩过。最后一次检查 SRAM 仍有约 `0x0f2c` 剩余。

## 日志字段用途

`TASK11_EVT` 记录关键事件：

- `start`
- `point`
- `advance_start`
- `advance_stop`
- `turn_start`
- `turn_stop`
- `force`
- `complete`

重点字段：

- `lap` / `seg` / `phase`：圈数和路段。
- `dist` / `phase_dist`：当前事件对应距离。
- `yaw`：相对航向角。
- `yprog` / `ydelta`：当前段或转向期间转过角度。
- `exp`：期望航向角。
- `herr`：航向误差。
- `nav_turn`：陀螺仪导航介入量。
- `mask` / `err` / `cnt`：循迹传感器状态。
- `B` / `A`：两侧编码器累计。

`TASK11_SUM` 记录每段汇总：

- `dist`
- `yaw=start/end`
- `yprog`
- `avg_herr` / `max_herr`
- `avg_line`
- `avg_nav`
- `avg_turn`
- `lost`
- `pmask`

`TASK11_WIN` 记录关键窗口的较密集数据，主要用于分析点位附近的角度和传感器变化。

## 日志整理脚本

新增/完善了 `tools/task11_log_to_csv.py`，用于将串口接收到的 Task11 文本日志整理成本地增量数据文件。

默认输出：

- `data/task11_experience_data.csv`
- `data/task11_experience_runs.csv`
- `data/task11_experience_segments.csv`
- `data/task11_experience_turns.csv`
- `data/task11_experience_summary.txt`

脚本能力：

- 拼回被串口接收工具拆开的长行。
- 解析 `TASK11_* key=value` 格式。
- 校验 `seq`、`idx` 连续性。
- 检查 RAM 溢出。
- 默认增量追加，不覆盖旧数据。
- 使用 `run_id` 防止重复导入同一份日志。
- 支持 `--strict`、`--replace`、`--allow-duplicate`。

使用说明见 `docs/task11_log_to_csv_usage.md`。

## 控制逻辑调整

### 1. 直线段 AC/BD

当前直线段逻辑：

1. AC 目标航向使用 `TASK3_AC_HEADING_TARGET_CDEG`。
2. BD 目标航向使用 `TASK3_BD_HEADING_TARGET_CDEG`。
3. 若 JY62 有效，则用航向误差生成 `nav_turn`。
4. 若直线段传感器看到胶布，叠加循迹 `line_turn`。
5. 若看不到胶布，保持陀螺仪导航，不强行找线。

这解决了之前“AC/BD 有胶布时横向偏移无法纠正”的问题，也保留了未来去掉 AC/BD 胶布后的过渡能力。

### 2. 弧线段 CB/DA

当前弧线段逻辑：

1. 使用循迹 PD 作为主控制。
2. 使用 `task11_arc_expected_yaw_cdeg()` 生成理论弧线期望角。
3. 将实际相对转角与期望角的误差换算为 `nav_turn`。
4. 最终控制为 `line_turn + nav_turn`。

弧线点识别当前需要：

- 距离超过 `TASK11_ARC_POINT_ARM_COUNT`。
- 已转过至少 `TASK11_ARC_POINT_YAW_ARM_CDEG=15000`。
- 对应边缘传感器看到线。

这个 yaw 门槛是为了避免 AC/BD 交叉胶布或弧线中途误触发。它不是唯一方式，未来也可以根据稳定数据调整为距离/窗口判定。

### 3. B/A 出弧快转

今天多次根据 B 点日志调整过：

1. B/A 出弧启用 yaw 目标角：
   - B 出弧目标：`TASK11_B_EXIT_TARGET_CDEG`
   - A 出弧目标：`TASK11_A_EXIT_TARGET_CDEG`
2. yaw 误差跨过目标角即可停，不再要求低角速度后才停。
3. 如果启用了 yaw-stop 且陀螺仪有效，快转一开始就使用慢速 PWM，降低过冲。
4. 后续又改成“线优先，yaw 兜底”：
   - 如果中间线、宽线或误差阈值先满足，则先停。
   - 如果一直没扫到线，再用 yaw 目标停，避免无限转。

这个调整是因为某次 B 点日志显示 yaw 停得准，但车已经转过线；当场地仍有 BD 胶布时，入线稳定性比强行追绝对 yaw 更重要。

### 4. C/D 入弧快转

C/D 入弧仍主要依赖循迹传感器将线转到中心区域。D 点最近的问题不是右转动作本身，而是 BD 直线末端 D 点识别错过。

## 今天的关键日志结论

### AC 起步偏

有一次日志显示 AC 航向并没有明显偏：

- 起点和目标角接近。
- 但 `lost` 和 `avg_line` 显示横向位置偏移。

结论：不是纯航向问题，而是直线段只听陀螺仪时无法纠横向偏移。因此打开了 `TASK11_STRAIGHT_IR_ASSIST_ENABLE`。

### B 点出弯过大

早期日志：

- B 点识别正常。
- B 出弯快转到 yaw 目标，但停止时 mask 不在中间或已经丢线。

结论：B 点不是没识别到，而是出弯策略不应只服从 yaw。已改为线优先、yaw 兜底。

### D 点入弯识别失败

最新一次失败发生在 lap1 BD：

- `dist=7877 mask=0xE0`：右侧传感器已经看到 D 点线。
- `dist=7988 mask=0xC0`：仍看到右侧线。
- 原门槛 `TASK11_BD_POINT_ARM_COUNT=8000`，这两帧都不能触发。
- `dist=8099 mask=0x00`：门槛打开时线已经过去。
- 最后 `BD force dist=12825`。

结论：不是传感器没看到，而是 BD 最短距离门槛略晚。已把 BD 门槛单独提前到 `7800`。

## 编译和资源状态

今天对控制代码修改后多次执行：

```powershell
& "C:\ti\ccs2051\ccs\utils\bin\gmake.exe" -C Debug all
```

最后几次修改后均编译通过。由于 `lc_printf` 之前使用 512 字节栈缓冲，怀疑会挤压栈空间并影响 JY62/全局状态，今天把 UART 格式化缓冲改为静态全局：

```c
static char g_uart_printf_buffer[512];
```

这样降低了函数调用栈压力。最后一次查看 map 时 SRAM 仍有约 `0x0f2c` 可用空间。

## 当前未完成和后续建议

1. “所有最短距离都提前一点”尚未统一落地。
   - 当前只完成了 `TASK11_BD_POINT_ARM_COUNT=7800`。
   - AC 仍为 `8000`。
   - 弧线段仍受 `TASK11_ARC_POINT_ARM_COUNT` 和 `TASK11_ARC_POINT_YAW_ARM_CDEG=15000` 共同限制。

2. 继续跑 Task11，重点观察：
   - BD 是否能在 `dist≈7800~8000` 触发 D 点。
   - B/A 出弧 `turn_stop` 是 `reason=line` 还是 `reason=yaw`。
   - B/A 出弧停止时 `mask` 是否在中心四路附近。
   - AC/BD 的 `lost` 是否降低。
   - CB/DA 的 `avg_herr` 和 `max_herr` 是否持续偏大。

3. 如果 D 点仍失败：
   - 再提前 `TASK11_BD_POINT_ARM_COUNT`。
   - 或加入“接近门槛时右边线预触发缓存”，即在门槛前若看到右边线，门槛到达后短时间内仍允许触发。

4. 如果 B 点仍转大：
   - 降低 B/A 出弧慢速 PWM。
   - 或给 B/A 单独配置更保守的出弧转向 PWM，不与 C/D 入弧共用。

5. 如果准备迁移到任务 3/4：
   - 先用脚本积累多次 Task11 数据。
   - 用稳定后的 BD 方向估计 BD 目标角。
   - 用稳定后的 AC 方向估计 AC 目标角。
   - 用 B/A/C/D 点转向前后角度和距离，形成分段经验参数。

## 交接注意事项

- 目前控制改动主要落在 Task11，任务 3/4 尚未同步迁移。
- Task11 的 AC/BD 胶布仍参与当前阶段调试；未来去掉胶布后，需要重新评估点位判断。
- 串口实时日志不要重新打开，否则可能再次阻塞控制周期。
- RAM 日志结束后输出是阻塞式，但发生在任务结束后，对运动控制影响较小。
- 本仓库当前有数据文件变更和 `.bak` 删除状态，处理 git 提交前需要确认是否保留这些数据变化。
