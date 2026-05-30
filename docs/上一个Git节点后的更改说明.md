# 上一个 Git 节点后的更改说明

> 注意：本文包含较早阶段的更改记录，部分参数已经不是当前实车可用版本。准备 push 前的最新状态、当前参数和已知问题，请优先查看 [当前调试状态与后续问题.md](当前调试状态与后续问题.md)。

本文整理当前工作区相对上一个 Git 提交的主要改动。范围来自 `git diff`，涉及 `main.c`、`app_config.h`、`main.syscfg` 和 `暂行接线图.md`。

## 1. 任务启动方式

- 新增任务编号枚举：`TASK_ID_NONE`、`TASK_ID_1`、`TASK_ID_2`。
- 任务启动从单一按键触发扩展为“按钮 + UART0 双控制”：
  - UART0 收到 `01`、`1` 或 `0x01`：启动任务一。
  - UART0 收到 `02`、`2` 或 `0x02`：启动任务二。
  - `PB00 / B00`：启动任务一。
  - `PB21`：启动任务二。
- 串口命令被接受后会打印：

```text
TASK UART command accepted: id=...
```

## 2. SysConfig 和接线文档

- `main.syscfg` 中 `KEY1` 从 `PB22` 改为 `PB00`。
- `暂行接线图.md` 已同步：
  - `Key1 / 任务一` 标注为 `PB00 / B00`。
  - `Key2 / 任务二` 保持为 `PB21`。
  - 说明当前程序已使用 `PB00` 触发任务一、`PB21` 触发任务二。
  - ST011 声光模块说明从“硬件端口已记录但未写逻辑”更新为“任务到点和任务结束时短暂触发声光提示”。

## 3. 直线段复用与任务一

- 原来的任务一直行逻辑被整理为可复用函数：

```c
run_straight_to_line_segment(tag, zero_heading, start_alarm_ms, stop_alarm_ms)
```

- 该函数用于：
  - 任务一：A -> B 直行。
  - 任务二的 A -> B 直行段。
  - 任务二的 C -> D 直行段。
- 串口报告中的模式名前缀由固定 `TASK1` 改为传入的 `tag`，例如 `TASK2_AB`、`TASK2_CD`。
- 直线段仍使用编码器速度 PID、编码器累计误差修正和 JY62 航向修正。
- 直线段停车条件保持为：到达设定里程后，红外检测到黑线即停车；没有增加“线必须接近中间”的条件。

## 4. 任务二流程

新增任务二完整流程：

```text
A -> B 直行
B -> C 半圆弧循迹
C -> D 直行
D -> A 半圆弧循迹
```

执行顺序：

1. `TASK2_AB`：直行到 B 点黑线。
2. `TASK2_BC`：沿半弧线循迹，到线尾后进行航向对准。
3. `TASK2_CD`：重置里程后，从 C 点直行到 D 点黑线。
4. `TASK2_DA`：沿半弧线循迹回到 A 点，线尾结束后直接停车复位，不再额外对准航向。

任务二异常时会打印中止位置，例如：

```text
TASK2 abort after BC: stop_reason=...
```

任务二完成时会打印：

```text
TASK2 complete: stopped at A, distance reset
```

## 5. 半弧循迹 PD 控制

新增半弧循迹函数：

```c
run_arc_line_follow_segment(tag, stop_alarm_ms, align_required)
```

主要逻辑：

- 使用八路红外循迹模块读取黑线位置误差。
- 对误差做低通滤波。
- 使用 PD 控制生成转向量：
  - `TASK2_ARC_KP_DIVISOR`
  - `TASK2_ARC_KD_DIVISOR`
- 对 D 项和转向输出增加限制：
  - `TASK2_ARC_DERIVATIVE_LIMIT`
  - `TASK2_ARC_TURN_LIMIT`
  - `TASK2_ARC_TURN_SLEW_STEP`
- 根据误差和 D 项大小动态降低基础速度，减小高速摆头和冲线风险。
- 串口报告包含：
  - `raw_err`
  - `err`
  - `der`
  - `base`
  - `turn`
  - `L`
  - `R`
  - `yaw_rel`
  - `yaw_abs`
  - `yaw_err`

## 6. 半弧出线后的航向对准

任务二第一段半弧 `TASK2_BC` 出线后会进行航向对准：

- 当前目标角：`17800`，即 `178.00` 度。
- 当前容差：`100`，即 `1.00` 度。
- 连续稳定次数：`TASK2_ARC_ALIGN_STABLE_COUNT`。
- 超时保护：`TASK2_ARC_ALIGN_TIMEOUT_MS`。

相关参数：

```c
#define TASK2_ARC_ALIGN_TARGET_CDEG   (17800)
#define TASK2_ARC_ALIGN_TOL_CDEG      (100)
#define TASK2_ARC_ALIGN_STABLE_COUNT  (4)
#define TASK2_ARC_ALIGN_TIMEOUT_MS    (2500)
```

说明：

- `TASK2_BC` 使用 `align_required=1U`，线尾后必须对准接近发车方向的反方向，之后进入 C -> D 直线段。
- `TASK2_DA` 使用 `align_required=0U`，线尾结束后直接认为任务完成，不再对准方向。

## 7. 速度参数调整

直线速度相对上一个 Git 节点提高：

```c
#define STRAIGHT_B_BASE_PWM (515)
#define STRAIGHT_A_BASE_PWM (555)
#define TASK1_RAMP_B_START_PWM    (475)
#define TASK1_RAMP_A_START_PWM    (515)
```

半弧循迹速度相对上一个 Git 节点提高：

```c
#define TASK2_ARC_BASE_PWM            (390)
#define TASK2_ARC_MIN_BASE_PWM        (250)
```

当前仍保留 `STRAIGHT_MAX_PWM` 和 `TASK2_ARC_MAX_PWM` 为 `650`。

## 8. 声光提示逻辑

新增 ST011 非阻塞提示机制：

```c
st011_start_pulse(pulse_ms)
st011_service(elapsed_ms)
delay_ms_with_st011(total_ms)
```

效果：

- 任务二经过 B、C、D、A 点时会触发声光提示。
- 声光提示不再通过 `delay_ms(120)` 阻塞任务二后续流程。
- 控制循环中的等待函数会顺带维护声光模块关闭时机。

保留行为：

- 任务一仍使用阻塞式 `st011_pulse()` 作为起点和终点提示。
- 任务二段切换内部仍有必要的刹车、JY62 置零和里程复位动作；本次只消除了声光提示本身造成的等待。

## 9. 串口调参输出

当前串口输出已经覆盖任务二关键调参信息：

- 直线段：
  - 距离 `dist`
  - 红外原始值 `raw`
  - 红外线掩码 `mask`
  - A/B 电机累计计数
  - A/B 电机速度
  - 速度 PID 的 `P/I/D`
  - 航向修正 `h_corr`
  - 最终 PWM
- 弧线段：
  - 红外误差、滤波误差、D 项
  - 基础 PWM、转向量
  - JY62 相对航向和目标角误差
  - 出线对准状态和停止原因

## 10. 构建验证状态

本地使用 CCS gmake 验证时，`main.c` 可以完成编译：

```text
Finished building: "../main.c"
```

但链接阶段仍出现 Windows 临时目录写入失败：

```text
fatal error #10296: cannot open output file "...AppData\\Local\\Temp\\{...}" for writing
```

该错误发生在链接阶段，不是当前源码语法错误导致。

## 11. 当前注意点

- 任务二速度已经提高，实车测试时重点观察：
  - 半弧循迹是否继续摆头。
  - B/C/D/A 点是否因为速度提高而冲过线。
  - `TASK2_BC` 出线后是否稳定停在接近 `178` 度。
- 若仍觉得过点有明显停顿，下一步要改的是任务二段切换中的刹车、置零和状态切换策略，而不是声光提示逻辑。
- 若 D 点入线仍偏一侧，目前没有加入“直线段停车时线必须居中”的条件，这是按之前要求保留的行为。
