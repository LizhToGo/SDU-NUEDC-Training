# K230 独立运行部署清单

用户手动把以下文件复制到 K230 `/sdcard`（本仓库不会自动写入设备）：

```text
boot.py
main.py
single_shot_measurement.py
tjc_screen.py
numbered_square_detector.py
advanced_target_detector.py
composite_square_detector.py
digit_template_recognizer.py
plane_binary_mask.py
plane_mapper.py
frame_detector.py
high_res_refiner.py
inner_aperture_locator.py
distance_estimator.py
measurement_consistency.py
basic_shape_detector.py
power_monitor.py
ina226_power_monitor_fixed.py
```

设备已有的运行时目录也必须保留：

```text
/sdcard/ybUtils/
/sdcard/media/
```

结果目录由程序自动创建：

```text
/data/captures/
```

屏幕端变量、按钮事件和状态定时器配置见同目录 `TJC_SCREEN_SETUP.md`；它不需要复制到 K230，只需在淘晶驰工程中照此配置。

## 启动方式

- 正常竞赛部署：删除 `/data/disable_autostart`，重启 K230；`boot.py` 会显式导入 `main.py`。
- IDE 维护：创建 `/data/disable_autostart` 后重启，boot 只关灯并打印提示，不启动相机主循环。
- 若设备固件本身已经在启动阶段自动执行 `main.py`，重复导入只会命中 MicroPython 的模块缓存；本项目仍以 `boot.py` 为唯一显式入口。

## 生产依赖与调试文件

以下目录不属于正式运行文件，不需要复制到竞赛 `/sdcard`：

```text
tools/k230/       # 板端相机、定位、标定、屏幕和 INA226 诊断脚本
tools/desktop/    # PC 端分析与仿真工具
data/calibration/ # 标定原始数据和分析结果
tests/            # PC 端自动测试
```

需要使用某个板端诊断工具时，只将 `tools/k230/` 中对应的脚本临时复制到 `/sdcard` 根目录运行。例如重新标定时复制 `tools/k230/distance_calibration_capture.py`，完成后可从设备删除。

## INA226 功率模块

默认接线和参数：

```text
K230 GPIO34 / IIC1_SCL -> INA226 SCL
K230 GPIO35 / IIC1_SDA -> INA226 SDA
K230 3.3V              -> INA226 VCC
K230 GND               -> INA226 GND
INA226 地址            = 0x40
分流器                 = R100 / 0.1 ohm
```

`single_shot_measurement.py` 会自动检测 INA226，并以 100 ms 间隔读取。若未检测到模块，程序仍可使用原来的 UART 手动功率命令；R100 配置的最大测量电流约为 `0.819 A`。

## 串口屏接线

```text
GPIO9 / 排针3 UART1_TXD  -> 淘晶驰 RX
GPIO10 / 排针4 UART1_RXD <- 淘晶驰 TX
GND                       -- 淘晶驰 GND
```

115200、8 数据位、无校验、1 停止位，3.3 V TTL；USB-C 可同时连接 CanMV IDE/MTP，但不连接 USB-TTL VCC。
