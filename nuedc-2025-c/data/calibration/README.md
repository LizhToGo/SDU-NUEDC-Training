# 距离标定数据

- `distance_calibration_5cm_v3.csv`：100～200 cm、每 5 cm 一个节点的正式采集数据。
- `distance_calibration_5cm_v3_summary.txt`：上述数据的分析摘要。
- `distance_calibration_temp_20260722.csv`：当前机械/光学状态变化后建立的临时验证表，其中部分节点为插值或外推。

K230 实际运行使用的标定点仍直接写在项目根目录的 `distance_estimator.py` 中，避免开机时依赖额外 CSV 文件。
