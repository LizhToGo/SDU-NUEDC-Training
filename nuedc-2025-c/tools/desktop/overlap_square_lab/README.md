# 重叠正方形桌面验证实验

这个目录只用于验证算法，不会修改 CanMV/K230 主程序。生成器只统计“图像上
可观测”的布局：每个正方形至少有三条边的片段和一个角露出。完全埋在同色
黑块里的正方形没有任何图像证据，任何纯视觉算法都无法恢复。

依赖：Python 3.10+、Pillow。

```powershell
python overlap_square_lab.py generate --seed 1
python overlap_square_lab.py batch --count 100
python overlap_square_lab.py example target5-1.png target6-1.png
```

每次检测会把结果图和调试图分开：

- `*_result.png`：最终输出，只保留全局解释器选中的正方形；普通方块为绿色，
  最小方块使用红色粗框；
- `*_accepted.png`：只显示通过质量门的候选，不显示橙色拒绝框；
- `*_debug.png`：完整诊断图，包含拒绝候选；
- `*.json`：候选、得分和拒绝原因等数据。

调试图中的颜色：

- 蓝色：生成器真值；
- 绿色：通过质量门的正方形候选；
- 橙色：被质量门拒绝的诊断候选；
- 左上角 `MIN`：按照边长排序后第一个通过质量门的候选。

主要统计量：

- `minimum_success_rate`：输出候选匹配某个真实正方形，且输出边长与真实最小边
  长的相对误差不超过 4% 的比例；
- `minimum_identity_success_rate`：输出候选恰好是最小真值正方形本身的比例；两
  个方块边长非常接近时，这个指标可能下降，但并不一定影响题目要求的数值输出；
- `all_square_recall`：所有生成正方形中至少被一个合格候选匹配到的比例；
- `minimum_side_error_*`：最小边长相对误差；
- `runtime_ms_*`：电脑端纯 Python 参考时间，不代表 K230 最终耗时。

## 2026-07-21 本机验证结果

使用带抗锯齿、0～0.75 px 模糊、0～7 灰度标准差噪声和随机二值阈值的 300 个
场景：

```text
minimum_success_rate          99.67% (299/300)
minimum_identity_success_rate 97.67%
all_square_recall             99.75%
median coarse side error       0.90%
max coarse side error          4.27%
mean desktop runtime          98.44 ms
p95 desktop runtime          109.89 ms
```

唯一一次严格失败仍然定位到了正确方块，只是低分辨率 Hough 粗边长为 72.0 px，
真值为 75.21 px（误差 4.27%）。K230 方案会在候选产生后调用已有的 1080p 基本题
边缘精测，因此粗边长不是最终输出。

渲染后的官方目标物样例也成功得到最小候选：

```text
目标物5：149.5 px（同时识别旋转方向族约 30°）
目标物6：149.5 px
```

局部质量门在目标物6上还会产生一个 `238 px` 的冗余候选。现在增加了一个轻量
全局解释器：在通常只有 4～8 个强候选的前提下精确枚举候选子集，比较候选并集
与观测黑色区域，同时对冗余方块施加模型复杂度惩罚。最终保留的边长为：

```text
目标物5：149.5、170、199、249 px
目标物6：149.5、170、195、224 px
```

目标物6错误的 `238 px` 候选会从最终结果图中删除。

新增全局解释器后的另一批 300 个随机噪声场景结果：

```text
minimum_success_rate  99.33%
all_square_precision 100.00%
all_square_recall      99.67%
exact_explanation     98.67%
total runtime mean   116.56 ms（电脑端纯 Python）
```

这里的 `exact_explanation` 要求输出方块数量、位置和每一个真值方块全部一一对应，
比只判断最小边长严格得多。
