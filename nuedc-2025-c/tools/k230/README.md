# K230 调试与标定工具

这里的脚本不是开机运行所需的生产文件。使用时，将需要的脚本临时复制到 K230 的 `/sdcard` 根目录，与项目根目录中的生产模块放在一起，再通过 CanMV IDE 手动运行。

| 文件 | 用途 |
|---|---|
| `camera_test.py` | GC2093 分辨率、预览和拍照测试 |
| `frame_detection_test.py` | 黑框持续定位调试 |
| `frame_detection_fast_test.py` | 低延迟定位实验 |
| `frame_scale_stability_test.py` | 内框尺度稳定性采样 |
| `distance_calibration_capture.py` | 100～200 cm 距离标定数据采集 |
| `ina226_probe.py` | INA226 I2C 扫描与身份寄存器诊断 |
| `LCD_test.py` | 淘晶驰串口屏控件通信测试 |

正式部署清单见 [`../../docs/K230_DEPLOYMENT.md`](../../docs/K230_DEPLOYMENT.md)。
