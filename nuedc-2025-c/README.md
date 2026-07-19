# NUEDC 2025 C：基于单目视觉的目标物测量装置

当前阶段实现了 A4 黑框的按键单次定位与距离测量。程序待机时只显示实时画面；每次按下板载按键后独立完成一次粗定位、1080p 局部精修、测距和文件保存，并将本次定位框固定显示到下一次按键。

## 文件

- `frame_detector.py`：低分辨率 A4 内外框粗定位。
- `high_res_refiner.py`：依据粗定位结果，在 1920×1080 图像的预测边缘附近进行梯度搜索和直线拟合。
- `distance_estimator.py`：利用 A4 已知尺寸进行针孔估距或标定表插值。
- `single_shot_measurement.py`：当前推荐入口，负责按键、相机、预览、保存和任务调度。
- `frame_detection_test.py`：旧版 640×360 持续检测调试程序。
- `frame_detection_fast_test.py`：中等分辨率实验程序。
- `camera_test.py`：GC2093 相机测试程序。

## 部署

将下列文件复制到 K230 的 `/sdcard`：

```text
frame_detector.py
high_res_refiner.py
distance_estimator.py
single_shot_measurement.py
```

在 CanMV IDE 中运行：

```text
/sdcard/single_shot_measurement.py
```

测量文件保存在：

```text
/data/captures/measure_NNN_1080.jpg
/data/captures/measure_NNN_preview.jpg
/data/captures/measure_NNN.txt
```

## 单次测量流程

1. 512×288 RGB565 图像执行 2～3 次全画面粗定位。
2. 获取 1920×1080 RGB888 图像。
3. 将粗定位角点按同视场比例映射到 1080p。
4. 沿预测内外框的法线搜索黑白梯度。
5. 对每条边的高分辨率边缘点进行总最小二乘拟合并求交点。
6. 利用 A4 外框 `21.0 cm × 29.7 cm` 或内框 `17.0 cm × 25.7 cm` 估计距离。
7. 保存结果，并固定显示本次定位框，等待下一次按键。

## 距离标定

`distance_estimator.py` 当前包含由约 100/150/200 cm 实拍图估出的临时焦距 `1155 px`。程序每次测量都会在终端和元数据中输出：

```text
frame_scale=...
```

正式标定时，在 100～200 cm 范围内等间距放置目标，记录每个已知距离对应的 `frame_scale`，然后填写：

```python
DISTANCE_CALIBRATION_POINTS = (
    (100.0, scale_at_100cm),
    (110.0, scale_at_110cm),
    # ...
    (200.0, scale_at_200cm),
)
```

存在两个以上标定点后，程序自动改用分段线性插值，并在结果中显示 `method=CALIBRATION_TABLE`。



## 黑框尺度稳定性测试

在开始 100～200 cm 查表标定前，先用 `frame_scale_stability_test.py` 验证黑框定位的重复性。将下列文件复制到 `/sdcard`：

```text
frame_detector.py
high_res_refiner.py
frame_scale_stability_test.py
```

修改脚本顶部的 `KNOWN_DISTANCE_CM`，固定摄像头和 A4 目标后按一次板载按键。程序默认执行 30 次独立定位，不保存 JPEG，结果写入：

```text
/data/stability/stability_100cm_NNN.csv
/data/stability/stability_100cm_NNN_summary.txt
```

初步通过标准：30/30 有效、无 `HIGH_RES_FALLBACK`、`effective_scale_geom4` 的 CV 不大于 0.5%，峰峰值不大于 1.0%。
