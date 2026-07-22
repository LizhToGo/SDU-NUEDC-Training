# PC 端工具

- `analyze_distance_calibration.py`：分析 K230 采集的距离标定 CSV，输出稳定性统计、拟合误差和标定表。
- `overlap_square_lab/`：在电脑上生成并验证重叠正方形场景，不参与 K230 开机运行。

示例：

```powershell
python .\tools\desktop\analyze_distance_calibration.py `
  .\data\calibration\distance_calibration_5cm_v3.csv `
  --expected-points 21
```
