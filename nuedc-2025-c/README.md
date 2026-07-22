# NUEDC 2025 C：基于单目视觉的目标物测量装置

当前入口已经同时覆盖基本题和发挥题：基本圆/等边三角形/正方形，随机组合正方形的最小边长，指定编号正方形，30°～60°倾斜目标平面，以及外部功率采样的 `P/Pmax` 显示。所有尺寸都先通过 A4 内框单应性恢复到 `17.0 cm × 25.7 cm` 物面；组合图形使用双帧独立识别一致性门，不依赖 AI/KPU。程序由 UART1 命令触发并将结果轮廓锁定到下一次触发。

## 目录结构

```text
nuedc-2025-c/
├─ *.py                  # K230 生产程序，保持扁平以便直接复制到 /sdcard
├─ docs/                 # 部署与串口屏文档
├─ data/calibration/     # 距离标定数据与摘要
├─ tools/k230/           # 板端调试、诊断和标定脚本
├─ tools/desktop/        # PC 分析与算法验证工具
└─ tests/                # 纯 Python 自动化测试
```

生产文件刻意保留在项目根目录，因为 CanMV 部署时这些模块需要位于同一个 `/sdcard` 模块搜索目录；调试脚本、生成数据和文档不混入生产部署清单。

## 主要文件

- `frame_detector.py`：低分辨率 A4 内外框粗定位。
- `high_res_refiner.py`：依据粗定位结果，在 1920×1080 图像的预测边缘附近进行亚像素梯度搜索、鲁棒直线拟合和黑色环带验证。
- `distance_estimator.py`：利用 A4 已知尺寸进行针孔估距或标定表插值，并在尺度空间融合两个一致结果。
- `measurement_consistency.py`：生产测量与标定程序共用的双帧尺度差、中心位移计算。
- `plane_mapper.py`：由内框四角建立图像像素与 `17.0 cm × 25.7 cm` 物面坐标之间的双向单应性。
- `basic_shape_detector.py`：检测基本圆/等边三角形/正方形，在物面坐标中分类并计算 `x`。
- `plane_binary_mask.py`：将 1080p A4 内框逆映射为 8 pixel/cm 的物面灰度和二值图。
- `composite_square_detector.py`：基于方向族、线段、正方形假设和联合覆盖解释恢复分离/重叠/旋转正方形。
- `numbered_square_detector.py`：编号正方形和倾斜单正方形的连通域快速路径；失败时自动回退到联合解释器。
- `digit_template_recognizer.py`：不使用 AI 的白色数字模板识别器。
- `advanced_target_detector.py`：发挥题最小正方形、指定编号和倾斜模式的任务封装与双帧质量门。
- `tjc_screen.py`：淘晶驰 X2 固定 7 字节触摸屏帧和 ASCII 控件写入协议。
- [`docs/TJC_SCREEN_SETUP.md`](docs/TJC_SCREEN_SETUP.md)：屏幕端 `program.s`、按钮事件、状态定时器和联调清单。
- `power_monitor.py`：统一功率状态、Pmax 和无字体预览显示；自动接入 INA226，未检测到硬件时保留 UART 手动输入。
- `ina226_power_monitor_fixed.py`：INA226 R100 硬件驱动，默认 IIC1 GPIO34/35、地址 `0x40`。
- `single_shot_measurement.py`：当前推荐入口，负责触发、双帧一致性、`D/x` 测量、相机、预览、保存和任务调度。
- `boot.py` / `main.py`：K230 脱离电脑后的显式开机启动入口。
- `tools/k230/distance_calibration_capture.py`：K230 板端 100～200 cm 距离—尺度标定数据采集器。
- `tools/desktop/analyze_distance_calibration.py`：PC 端标定 CSV 统计、拟合和留一距离交叉验证工具。
- `tests/test_consistency_and_calibration.py`：双帧一致性门和标定分析的纯 Python 测试。
- `tests/test_measurement_pipeline.py`：黑框精修、质量门和测距管线测试。
- `tests/test_plane_and_basic_shapes.py`：PC 端单应性往返映射与三种合成图形测试。
- `tests/test_advanced_targets.py`：重叠/旋转/编号正方形、双帧发挥质量门和功率状态测试。
- `tools/k230/frame_detection_test.py`：旧版 640×360 持续检测调试程序。
- `tools/k230/frame_detection_fast_test.py`：中等分辨率实验程序。
- `tools/k230/camera_test.py`：GC2093 相机测试程序。

## 部署

完整部署说明见 [`docs/K230_DEPLOYMENT.md`](docs/K230_DEPLOYMENT.md)，串口屏工程配置见 [`docs/TJC_SCREEN_SETUP.md`](docs/TJC_SCREEN_SETUP.md)。

将下列文件复制到 K230 的 `/sdcard`：

```text
frame_detector.py
high_res_refiner.py
inner_aperture_locator.py
distance_estimator.py
measurement_consistency.py
plane_mapper.py
basic_shape_detector.py
plane_binary_mask.py
composite_square_detector.py
numbered_square_detector.py
digit_template_recognizer.py
advanced_target_detector.py
tjc_screen.py
power_monitor.py
ina226_power_monitor_fixed.py
single_shot_measurement.py
boot.py
main.py
```

竞赛部署时重启 K230 即自动执行 `/sdcard/boot.py -> /sdcard/main.py -> single_shot_measurement.py`。调试时若不希望开机启动，先在 `/data` 创建空文件 `disable_autostart`，调试结束后删除该文件再重启。

`boot.py` 会显式把 `/sdcard` 加入模块搜索路径，并在部分固件无法 `import main` 时直接执行 `/sdcard/main.py`；因此不能只复制 `main.py`，必须同时复制 `boot.py`。

在 CanMV IDE 中手动运行（或调试自动启动程序）：

```text
/sdcard/single_shot_measurement.py
```

生产测量程序默认使用淘晶驰 X2 触摸屏的 UART1 二进制触发协议，参数为 `115200 8N1`。程序优先用 `machine.UART/FPIOA` 显式映射 GPIO9/GPIO10；若固件不支持该构造形式才回退到 `/sdcard/ybUtils/YbUart.py`。两者对应开发板排针 3/4：

```text
K230 GPIO9 / 排针3 UART1_TXD  -> 屏幕 RX
K230 GPIO10 / 排针4 UART1_RXD <- 屏幕 TX
K230 GND                       -- 屏幕 GND
```

屏幕发送固定 7 字节帧：

```text
55 CMD DATA0 DATA1 FF FF FF
```

```text
CMD=01 DATA0=00  基本题圆/三角形/正方形
CMD=01 DATA0=01  发挥最小正方形
CMD=01 DATA0=02 DATA1=0..9  发挥指定编号正方形
CMD=01 DATA0=03  发挥 30°～60°倾斜平面，只输出 x
CMD=02             复位 Pmax
CMD=03             屏幕就绪/复位显示
CMD=04             请求刷新当前显示
```

K230 只向屏幕的数字控件发送 ASCII 指令并追加 `FF FF FF`；`main.tStatus` 和 `main.tTarget` 由屏幕端事件/定时器管理。当前屏幕小数位对应的数值缩放为 `xD=D×100`、`xSize=x×100`、`xCurrent=I×1000`、`xPower=P×100`、`xPmax=Pmax×100`。状态码：`0`待机、`1`测量中、`2`成功、`3`失败、`4`未检测到靶纸、`5`图形失败、`6`编号未找到、`7`系统异常。

USB-C 可同时连接 CanMV IDE/MTP；串口屏占用独立 UART1，不会与 USB-C 冲突。不要连接 USB-TTL 的 VCC，使用 3.3 V TTL 电平。

如需在没有屏幕时用 MobaXterm/USB-TTL 调试 ASCII 命令，可将 `single_shot_measurement.py` 顶部 `USE_TJC_SCREEN` 临时改为 `False`，此时命令为：

```text
M        基本题：圆/三角形/正方形，输出 D 和 x
A        发挥1/2：组合正方形，输出 D 和最小边长 x
N1..N9  发挥3：选择指定数字，输出 D 和对应正方形 x
T        发挥4：30°～60°倾斜平面，只输出 x
W=1.23   输入外部功率计测得的 1.23 W，并更新 P/Pmax
I=0.823,W=4.12  同时输入电流(A)和功率(W)，更新 xCurrent/xPower/xPmax
PW       查询 P/Pmax
R        将 Pmax 重置为当前功率
S        查询 IDLE、LOCKED 或 FAILED 状态及最近结果
P        连通性测试，板端回复 PONG
H        返回简短命令帮助
```

命令可带 CR/LF；`M/A/T` 也支持单字节立即触发。程序先回复 `ACK MEASURE`，完成后回复 `RESULT OK`、`RESULT RETRY` 或 `RESULT ERROR`。上位机收到结果前不得重复发送测量命令；测量忙碌期间进入缓冲区的额外命令会被清空。

测量文件保存在：

```text
/data/captures/measure_NNN_1080.jpg
/data/captures/measure_NNN_preview.jpg
/data/captures/measure_NNN.txt
/data/captures/measure_NNN_plane.pgm   # 仅发挥模式，物面二值诊断图
```

## 单次触发测量流程

1. 640×360 RGB565 图像执行自适应全画面粗定位：高分候选可在第 1 帧早停，低分候选再看第 2 帧；前两帧完全无候选时，第 3 帧才把 `find_rects()` 阈值从 `1800` 降到 `1200` 兜底。640×360 与 1920×1080 恰好是 1:3，同样能够直接显示在 640×480 LCD 中。
2. 获取 1920×1080 RGB888 图像。
3. 将粗定位角点按同视场比例映射到 1080p。
4. 沿预测内外框的法线搜索黑白梯度；平顶梯度取浮点中心，单峰用三点抛物线插值得到亚像素边缘位置。
5. 对每条边的高分辨率边缘点进行 MAD/Huber 鲁棒总最小二乘拟合，再由相邻直线交点得到浮点角点。
6. 最多只精修 3 个不同粗定位假设，并按粗定位分数从高到低执行；第一个通过全部 1080p 质量门的 `STRONG` 结果立即早停，不再浪费时间精修低排名候选。
7. 独立外框完整且比例正确时继续使用 `HIGH_RES_PAIR`；独立外框在 175～200 cm 处缺失时，只要内框四边、亚像素 RMS、矩形规则性和四边黑色环带全部通过，允许 `HIGH_RES_INNER` 作为 `STRONG`。远距离下外框搜索偶尔会拟合到四条整齐但位置漂移的梯度：若独立外框与内框比例冲突，但“由内框和官方尺寸预测出的外框位置”同时通过至少 24 个采样点、内侧覆盖率 ≥85%、最差单边 ≥70%、外侧白区覆盖率 ≥50% 的完整白—黑—白环带门，则把错误外框降级，仅使用内框；完整环带不通过时仍以 `FRAME_DISAGREEMENT` 拒绝。
8. 仅使用内框 `17.0 cm × 25.7 cm` 的浮点宽高尺度几何平均值作为当前帧尺度。
9. 最多执行 5 次独立测量，寻找两个 `STRONG` 结果；两者尺度相对差必须不大于 `0.50%`，内框中心位移还必须不大于两帧平均内框短边的 `10%`。尺度门允许实测中可接受的亚像素波动，中心门负责拒绝位置已经明显变化但尺度碰巧相近的两帧。
10. 先平均两个尺度，再进行查表插值或针孔换算，得到 `D`；不直接平均两个距离。
11. 双帧尺度一致后，用当前帧的 `inner_corners_float` 建立“1080p 图像四边形 → `17.0 cm × 25.7 cm` 物面”的单应性。
12. 只在内框包围 ROI 中运行 `find_blobs()`，筛选位于纸面中央、物面尺寸合理的黑色连通域；它只负责产生候选，不直接决定图形类型和尺寸。
13. 利用目标中心黑色样本和纸面上下方白色样本计算本次自适应灰度阈值；随后在物面坐标中沿 72 个方向追踪黑白边界。
14. 使用径向长度的旋转不变特征分类：圆的半径近似恒定，等边三角形具有强三次谐波，正方形具有强四次谐波。利用极坐标面积计算圆直径或多边形边长，得到 `x`。
15. 只有 `D` 的双帧一致性和图形质量门同时通过，才保存并锁定黑框与黄色图形轮廓；图形失败时释放当前 1080p 图像并继续下一次独立尝试。

### 发挥题快速路径与倾斜模式

- `DIGIT_SELECT` 先在 8 pixel/cm 的矫正物面上做黑色 8 连通域，按尺寸、填充率、方形度过滤；官方四个分离编号方块可在几十毫秒内得到，最小方块不会再被复合候选预算截掉。若少于四个独立方块或目标数字未找到，自动回退 `CompositeSquareDetector`，因此重叠正方形仍受支持。
- 指定数字已知时，模板识别器的高置信度小 margin 结果允许 `digit_match_mode=SOFT_TARGET`，但仍要求两帧目标数字/数量/中心/边长一致，不会全局放宽数字分类。
- `TILT_MIN` 只精修内框四条边并建立单应性，跳过不需要的外框高分辨率搜索；倾斜引起的尺度各向异性不再被平行平面质量门误判。该模式最多三次尝试，基本/其它发挥模式仍最多五次，目标是在 5 s 内完成。倾斜任务的单个旋转正方形使用有界方向投影框；普通编号题先走轴对齐连通域，只有检测到旋转迹象时才启用该慢路径。
- `AUTO_MIN`/`DIGIT_SELECT` 的内框各向异性门使用 `0.14` 的高级任务上限，普通基本题仍保持 `0.08` 严格上限；后续的物面图形质量门、目标身份门和双帧尺度/中心一致性会继续拒绝错误定位。元数据中的 `attempt_N_geometry_profile=ADVANCED_RELAXED` 可确认该路径。

UART触发后不固定等待 300 ms；高分辨率通道只刷新 2 帧后取图，不依赖机械按键消抖。成功元数据还会记录：

```text
measurement_attempts=...
fusion_count=2
component_frame_scales=...,...
frame_scale_spread_pct=...
frame_center_shift_ratio=...
inner_only_accepted=YES|NO
outer_independently_valid=YES|NO
outer_wide_recovery_used=YES|NO
outer_wide_search_used=YES|NO
outer_conflict_demoted=YES|NO
detected_frame_disagreement=...
ring_outside_pass_ratio=...
shape_valid=YES
shape_type=CIRCLE|TRIANGLE|SQUARE
shape_x_cm=...
shape_confidence=...
shape_fill_ratio=...
shape_width_cm=...
shape_height_cm=...
shape_ms=...
```

彻底失败时仍保存 `measure_NNN_failed.jpg`。如果存在可精修但被质量门拒绝的 1080p 帧，还会保存 `measure_NNN_failed_1080.jpg`：绿色为内框，红色为独立检测外框，蓝色为内框按官方尺寸预测的外框。若后续尝试成功，该临时诊断图会自动删除。

`measure_once()` 返回的结果字典同时含有 `distance_cm`、`x_cm`、`shape_type` 和 `shape_confidence`，后续触摸屏界面可以直接读取，不需要再解析文本元数据。

## 图形识别与尺寸计算原理

### 1. 四角单应性，而不是拿图像水平宽高硬算

高分辨率精修输出的内框角点顺序固定为 `TL, TR, BR, BL`。`plane_mapper.py` 用 4 对对应点建立 8 个线性方程，通过带主元选择的 8×8 高斯消元求得单应矩阵：

```text
inner TL,TR,BR,BL in image  <->  (0,0),(17,0),(17,25.7),(0,25.7) cm
```

模块同时求出 `image_to_plane(x,y)` 和 `plane_to_image(u,v)`。因此后续长度、中心、边界和面积都在厘米坐标中计算，A4 在图像中的平移、缩放、旋转和一定程度的透视不会直接污染 `x`。

### 2. Blob 只做粗候选，避免把包围框填充率当最终答案

RGB565 图像使用 LAB 黑色阈值：

```python
(0, 50, -128, 127, -128, 127)
```

程序排除靠近内框边缘、中心偏离过大、物面包围尺寸或面积不合理的连通域。黑色边框和 ROI 外的背景即使进入 `find_blobs()`，也不能仅凭像素多而成为结果。

### 3. 在物面中追踪 72 条径向边界

候选中心附近取黑色参考，纸面上、下安全区域取白色参考，本次阈值取二者之间的 48%。每条射线在厘米坐标中前进，再用 `plane_to_image()` 回到原图读取像素；黑到白的过渡位置做线性插值。这样得到按角度排列的半径 `r(θ)`，而不是受透视影响的图像轴对齐矩形。

分类采用径向变异系数以及三、四次傅里叶谐波幅值：

- 圆：`r(θ)` 几乎恒定；
- 等边三角形：三次谐波占主导；
- 正方形：四次谐波占主导。

谐波取幅值、不取相位，所以图形在纸面内旋转时分类规则不变。

### 4. 极坐标面积与直线交点联合得到 `x`

均匀角度采样后先计算：

```text
A = 1/2 * Σ(r_i² * Δθ)
```

圆直接由面积反解直径：

```text
圆直径 x = sqrt(4A / π)
```

对于三角形和正方形，程序还利用三/四次谐波的相位预测各顶点方向，将相邻顶点方向之间的径向边界点分配给一条边；排除顶点附近 14% 的采样后，对每条边执行物面总最小二乘直线拟合，再用相邻直线交点得到真正的 3/4 个角。最终边长由“75% 四边/三边均值 + 25% 面积反解值”得到，同时检查边长 CV、直线 RMS 和面积尺度分歧。预览对多边形只画拟合后的直边和角点，不再把稀疏径向采样折线当作方框。

这比直接使用 blob 的图像包围框宽高更能抵抗纸面透视和图形旋转。`basic_shape_detector.py` 仍只负责基本部分，发挥算法放在独立模块中，避免继续膨胀基本检测器。

## 发挥题算法

1. 用 A4 内框四角建立单应性，将目标区域逆映射成固定 `136×206`、`8 pixel/cm` 的物面图。
2. Otsu 阈值加极端值约束生成黑色掩膜；正方形边长直接以厘米计算，不再由距离反推。
3. 从黑区外轮廓梯度统计模 90° 的方向族，同时支持水平族和旋转族。
4. 将边界聚为直线段，由平行线间距和可见 L 角产生 5.5～12.5 cm 正方形假设。
5. 每个假设检查内部黑色填充、四边黑内白外支持、端点是否继续延伸和外露角点；“在大黑块角落随便塞一个小方块”的假解会因边界继续延伸被拒绝。
6. 对最多 20 个候选执行最多 6 个正方形的有界束搜索，使候选并集同时具有高黑区召回率和高精度，从而恢复部分重叠后被遮住的边。
7. `A`/`T` 选择联合解释中的最小边长；`Nn` 在每个黑方块中央提取不接触边界的白色字形，用 0～9 二值模板、双向膨胀重合率、宽高比和孔洞数识别编号。
8. 两张独立有效帧都完整运行发挥检测，要求正方形数量、选择身份一致，选中边长差不超过 `0.30 cm`、物面中心位移不超过 `0.75 cm`。
9. `T` 模式保留四角定位但绕过“平行目标面才成立”的距离各向异性门；只利用单应性输出 `x`，明确不输出 `D`。

成功时黄色框是最终选中的正方形，紫色框是同一联合解释中的其他正方形。`measure_NNN_plane.pgm` 是已经矫正到物面的黑白诊断图，可用 Pillow、ImageMagick 或常见图像工具转换成 PNG。

### 功率显示

功率模块默认使用 INA226 硬件采样：IIC1、GPIO34=SCL、GPIO35=SDA、地址 `0x40`、R100 分流器。驱动以 100 ms 节流读取，避免影响预览帧率；当前实测经验修正为 INA226 电流读数加 `0.077 A`，该修正只作用于硬件采样，UART 手动输入不修正；有总线电压时功率同步按修正后的电流重算。若 INA226 未接入，则回退到 UART 的 `W=<瓦特>` 或 `I=<安培>,W=<瓦特>` 手动输入。程序保存平滑值 `I/P` 和最大值 `Pmax`，并使用内置点阵线段在预览左上角绘制，不调用会触发 FreeType 错误的 `draw_string()`。R100 默认最大可测电流约 `0.819 A`，超出范围会报告溢出。

## 距离标定

`distance_estimator.py` 当前暂时切换到 2026-07-22 当前安装状态的临时标定表。110～190 cm 的 10 cm 节点来自 `measure_225`～`measure_233`，其余 5 cm 节点和端点为倒数尺度插值/外推，占位用于测试。最终比赛前仍需在最终机械安装状态下重新采集完整 21 点、每点 5 次的正式表。

先将 `tools/k230/distance_calibration_capture.py` 临时复制到 K230 的 `/sdcard` 根目录。将目标平面与相机尽量保持平行，统一用同一种距离定义从相机参考平面量到目标平面，然后在 CanMV IDE 中运行：

```text
/sdcard/distance_calibration_capture.py
```

采集器通过板载 UART1 触发，参数为 `115200 8N1`。接线如下：

```text
K230 排针 3：UART1_TXD  -> USB-TTL/上位机 RXD
K230 排针 4：UART1_RXD  <- USB-TTL/上位机 TXD
K230 GND                -- USB-TTL/上位机 GND
```

使用 3.3 V TTL 电平；K230 已由 USB-C 供电时不要再连接 USB-TTL 的 VCC。USB-C 可继续连接 CanMV IDE/MTP，UART1 可同时连接独立的 USB-TTL 或下位机，两条链路互不占用；USB-C 本身不会自动转发 UART1 数据。

串口命令均为 ASCII：

```text
M\n 或单字节 1：连续采满当前距离点的5组有效样本
S\n：查询当前距离和完成数量
P\n：连通性测试，板端回复 PONG
```

采集器依次提示 `100, 105, 110, ..., 200 cm`。目标放到提示距离后只发送一次 `M`，程序会连续取得当前点的 5 组有效样本；单组失败会自动重试，整批最多执行 15 次样本采集，达到 5 组后才发送最终 `RESULT OK`。如果 15 次仍未采满，则发送 `RESULT RETRY` 并保留已经成功的数据，再发一次 `M` 即可续满。批处理期间进入串口缓冲区的额外命令会被清空。

每一组样本仍由两个独立 `STRONG` 帧组成：双帧尺度差须不大于 `0.50%`，中心位移须不大于平均内框短边的 `10%`。批内进度以 `SAMPLE OK` 或 `SAMPLE RETRY` 输出；收到最终 `RESULT OK` 和下一条 `MOVE D=...` 后再移动目标。程序只自动完成当前距离，不会在未移动目标时采集下一个距离。

v3 只复用上一张严格通过1080p精修的四角位置作为 `TRACK_SEED`。下一次尺度仍从一张全新的1920×1080图像重新测量，不会沿用旧尺度。跟踪精修失败时会立即丢弃种子并恢复 `CENTER -> CENTER_LOW -> FULL_FALLBACK` 冷启动搜索。严格种子会在同一距离的5组样本间复用，也会携带到相邻5 cm点；相邻点的尺寸变化由高分辨率边缘搜索重新吸收。

结果会持续写入并支持断点续采：

```text
/data/captures/distance_calibration_5cm_v3.csv
/data/captures/distance_calibration_5cm_v3_summary.txt
```

新文件名会保留旧版10 cm数据和v2失败批次，避免断点续采时混表。如需从头重采当前v3数据，将脚本顶部的 `RESET_EXISTING_DATA` 临时改为 `True` 运行一次，随后立即改回 `False`。完成后把 CSV 复制到 PC，在本项目目录运行：

```powershell
python .\tools\desktop\analyze_distance_calibration.py .\data\calibration\distance_calibration_5cm_v3.csv --expected-points 21
```

分析器按距离输出 `median_scale`、`MAD`、耗时和双帧差，检查尺度是否随距离单调下降，拟合 `D=a/scale+b` 与 `D=a/scale`，并对最终使用的倒数尺度分段线性查表算法进行“整段距离点留一”交叉验证。最后会打印可直接复制的：

```python
DISTANCE_CALIBRATION_POINTS = (
    (100.0, median_scale_at_100cm),
    (105.0, median_scale_at_105cm),
    (110.0, median_scale_at_110cm),
    # ...
    (200.0, median_scale_at_200cm),
)
```

生产程序先计算 `u=1/frame_scale`，再在相邻标定点的 `u` 之间线性插值距离；这既保留查表法，又符合近似针孔关系 `D=a/scale+b`。结果仍显示 `method=CALIBRATION_TABLE`。当前21点表已经写入 `distance_estimator.py`；接下来应在未参与标定的位置独立验证，例如 `102、103、107、108、...、197、198 cm`，重点检查标定节点附近是否仍出现局部最大误差。



## 黑框尺度稳定性测试

在开始 100～200 cm 查表标定前，先用 `tools/k230/frame_scale_stability_test.py` 验证黑框定位的重复性。将下列文件复制到 `/sdcard`：

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

初步通过标准：30/30 为 `STRONG`、无 `HIGH_RES_FALLBACK`、`measurement_scale_inner_geom` 的 CV 不大于 0.5%，峰峰值不大于 1.0%。调参时还应检查 `confidence_counts`、`reject_counts`、`ring_inside_pass_ratio`、`ring_min_side_pass_ratio`、`detected_frame_disagreement` 和 `outer_edge_rms`。


## 当前尺度与质量门策略

测距仅使用 `inner_corners_float` 计算的内框宽高尺度几何平均值：

```text
measurement_scale_inner_geom = sqrt((inner_width_px / 17.0) * (inner_height_px / 25.7))
```

外框仅作身份验证和诊断，不再参与距离尺度平均。平行目标的质量门会检查：

- 内框 4 条边、鲁棒直线拟合 RMS、面积填充率、对边一致性和尺度各向异性；
- 外框至少 3 条独立有效边、外框拟合 RMS，以及实测内外框比例误差；
- 沿四边采样的白—黑环带对比度、总通过率和最差单边通过率。

结果分为三级：

- `STRONG`：全部身份与几何门通过，且内框 RMS 不大于 `1.0 px`；可参与距离标定和最终测量。
- `WEAK`：其他门通过，但内框 RMS 在 `1.0～1.5 px`；保留用于诊断或后续重试，不直接输出距离。
- `REJECT`：任一身份或几何门失败；只保留定位/诊断信息。

强透视倾斜场景仍可保留 `localization_valid` 定位结果，但不输出平行平面距离。
