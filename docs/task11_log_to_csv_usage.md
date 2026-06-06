# Task11 日志整理脚本使用指南

本文说明如何使用 `tools/task11_log_to_csv.py` 将串口接收到的 Task11 日志整理成本地累计数据文件。

## 脚本用途

`tools/task11_log_to_csv.py` 用于处理类似 VOFA+、串口助手或 Codex 附件中保存的 Task11 文本日志。

它会自动完成：

- 拼回被接收工具时间戳拆开的长行。
- 解析 `TASK11_* key=value` 格式数据。
- 校验 `seq` 是否连续。
- 校验 `EVT/SUM/WIN` 的 `idx` 是否连续。
- 校验是否收到 `TASK11_DUMP_SECTION_END` 和 `TASK11_RAM_END`。
- 检查 RAM 日志是否溢出。
- 默认增量追加到本地累计 CSV 文件。
- 自动防止同一份日志重复导入。

## 基本用法

先把串口接收到的完整 Task11 日志保存为文本文件，例如：

```text
C:\Users\orang\Desktop\task11_run_001.txt
```

然后在工程根目录运行：

```powershell
& "C:\Users\orang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\task11_log_to_csv.py "C:\Users\orang\Desktop\task11_run_001.txt" --strict
```

`--strict` 表示如果日志缺行、计数不一致或 RAM 溢出，脚本会返回失败，方便及时发现这次数据不能直接用于调参。

## 默认输出文件

脚本默认采用增量模式，不会覆盖旧数据。

每次导入一份新日志，会追加写入这些文件：

| 文件 | 内容 |
| --- | --- |
| `task11_experience_data.csv` | 全部重建后的 `TASK11_*` 原始记录，适合做细粒度分析。 |
| `task11_experience_runs.csv` | 每次 Task11 运行一行总览，包括完整性、总记录数、完成状态、主要配置。 |
| `task11_experience_segments.csv` | 每次运行的 `TASK11_SUM` 分段摘要，通常每 5 圈为 20 行。 |
| `task11_experience_turns.csv` | 每次运行的转向事件摘要，记录转向前角度、转向后角度、转过角度、转向距离。 |
| `task11_experience_summary.txt` | 人类可读的 summary 文本，会追加每次运行的校验和关键统计。 |

## 重复导入保护

脚本会根据日志内容生成 `run_id`，格式类似：

```text
task11_f1a18e8ccac2
```

如果同一份日志已经导入过，再次运行时默认不会重复追加。

如确实需要重复导入同一份日志，使用：

```powershell
& "C:\Users\orang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\task11_log_to_csv.py "C:\Users\orang\Desktop\task11_run_001.txt" --allow-duplicate
```

## 清空重建

一般不建议清空累计数据。

如果你明确要用某份日志重新生成所有输出文件，可以使用 `--replace`：

```powershell
& "C:\Users\orang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\task11_log_to_csv.py "C:\Users\orang\Desktop\task11_run_001.txt" --replace --strict
```

注意：`--replace` 会清空并重写默认输出文件。

## 自定义输出路径

如果想把某次测试写到单独目录，可以指定输出文件：

```powershell
& "C:\Users\orang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" tools\task11_log_to_csv.py "C:\Users\orang\Desktop\task11_run_001.txt" `
  --output logs\task11_records.csv `
  --runs-output logs\task11_runs.csv `
  --segments-output logs\task11_segments.csv `
  --turns-output logs\task11_turns.csv `
  --summary logs\task11_summary.txt `
  --strict
```

## 推荐实验流程

1. 运行任务 10，完成零航向标定。
2. 运行任务 11，让小车完成目标圈数。
3. 等待串口完整输出到 `TASK11_RAM_END`。
4. 保存串口接收文本。
5. 运行本脚本导入日志。
6. 查看 `task11_experience_summary.txt`，确认 `validation_ok=1`。
7. 使用 `task11_experience_segments.csv` 和 `task11_experience_turns.csv` 统计多次实验数据。

## 判断日志是否可用

summary 中建议重点看这些字段：

```text
validation_ok=1
seq: ... missing=0
evt_idx: ... missing=0
sum_idx: ... missing=0
win_idx: ... missing=0
EVT: ... ok=1
SUM: ... ok=1
WIN: ... ok=1
overflow: win_ov=0 ev_ov=0 sum_ov=0 ok=1
complete: lap=5 ... reason=point
```

如果 `validation_ok=0`，这次日志不建议直接作为经验数据使用。

## 输出字段说明

### `task11_experience_segments.csv`

常用字段：

| 字段 | 含义 |
| --- | --- |
| `run_id` | 本次日志导入编号。 |
| `lap` | 圈数。 |
| `seg` | 当前路段，通常为 `AC/CB/BD/DA`。 |
| `dist` | 当前段结束时编码器距离计数。 |
| `yaw_start` / `yaw_end` | 当前段开始和结束相对航向角。 |
| `yprog` | 当前段相对转过角度。 |
| `avg_herr` / `max_herr` | 航向误差统计。 |
| `nav_lost` | 导航数据丢失计数。 |
| `lost` | 循迹丢线计数。 |

### `task11_experience_turns.csv`

常用字段：

| 字段 | 含义 |
| --- | --- |
| `phase_dist` | 开始转向前，上一段已经走过的距离。 |
| `yaw_before` | 快速转向开始前航向角。 |
| `yaw_after` | 快速转向停止后航向角。 |
| `yaw_delta` | 快速转向实际转过角度。 |
| `turn_dist` | 快速转向期间编码器距离。 |
| `stop_reason` | 转向停止原因。 |
| `stop_mask` | 停止时循迹传感器 mask。 |

这些字段适合用来估计 C/D 点转向前距离、B/A 点弧线出线姿态，以及未来任务 3/4 的航向目标经验值。

## 常见问题

### 为什么同一条 `TASK11_SUM` 在串口日志里断成两行？

接收工具可能会给长行中间插入新的时间戳。本脚本会自动把这种分块拼回完整记录。

### 为什么第二次导入同一份日志没有追加？

这是重复导入保护。脚本检测到相同 `run_id` 后会跳过，避免同一组数据被重复统计。

### 能不能只生成 summary，不写 CSV？

当前脚本设计目标是积累数据，因此默认会写 CSV。若只想看校验结果，可以把输出路径指定到临时目录。

