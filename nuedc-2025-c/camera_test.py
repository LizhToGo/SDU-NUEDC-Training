import time
import os
import gc

from media.sensor import *
from media.display import *
from ybUtils.YbKey import YbKey


# ============================================================
# 配置
# ============================================================

SAVE_DIR = "/data/captures"

CAPTURE_CH = CAM_CHN_ID_0
CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080

PREVIEW_CH = CAM_CHN_ID_1
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

PREVIEW_X = 0
PREVIEW_Y = (DISPLAY_HEIGHT - PREVIEW_HEIGHT) // 2

IDE_PREVIEW_QUALITY = 70
JPG_QUALITY = 95

# True：同时保存RGB888原始数据
# False：只保存JPG
SAVE_RAW = False


# ============================================================
# 全局变量
# ============================================================

sensor = None
key = None
display_inited = False
photo_id = 0


# ============================================================
# 文件系统
# ============================================================

def ensure_dir(path):
    try:
        os.stat(path)
    except OSError:
        os.mkdir(path)


def find_last_photo_id():
    max_id = 0

    try:
        filenames = os.listdir(SAVE_DIR)
    except OSError:
        return 0

    for filename in filenames:
        if not filename.startswith("gc2093_"):
            continue

        dot_index = filename.find(".")

        if dot_index < 0:
            dot_index = len(filename)

        number_text = filename[7:dot_index]

        try:
            number = int(number_text)

            if number > max_id:
                max_id = number
        except Exception:
            pass

    return max_id


# ============================================================
# 拍照
# ============================================================

def capture_photo():
    global photo_id

    photo_id += 1

    print("")
    print("Capturing photo:", photo_id)

    img = None

    # 获取按键触发后的新画面
    for i in range(3):
        img = sensor.snapshot(chn=CAPTURE_CH)

    print(
        "Captured:",
        img.width(),
        "x",
        img.height(),
        "format:",
        img.format()
    )

    # --------------------------------------------------------
    # 可选：保存无损RGB888原始数据
    # --------------------------------------------------------

    if SAVE_RAW:
        raw_path = "%s/gc2093_%03d.rgb888" % (
            SAVE_DIR,
            photo_id
        )

        img.save(raw_path)
        print("RAW saved:", raw_path)

    # --------------------------------------------------------
    # 保存JPG
    # --------------------------------------------------------

    gc.collect()

    # RGB888不能直接保存成JPG
    img565 = img.to_rgb565()

    jpg_path = "%s/gc2093_%03d.jpg" % (
        SAVE_DIR,
        photo_id
    )

    img565.save(
        jpg_path,
        quality=JPG_QUALITY
    )

    print("JPG saved:", jpg_path)
    print("CAPTURE_DONE:", photo_id)
    print("Press KEY to capture again.")

    img565 = None
    gc.collect()


# ============================================================
# 主程序
# ============================================================

try:
    ensure_dir(SAVE_DIR)

    photo_id = find_last_photo_id()

    os.exitpoint(os.EXITPOINT_ENABLE)

    # 初始化亚博板载按键
    key = YbKey()

    # --------------------------------------------------------
    # 初始化GC2093
    # --------------------------------------------------------

    sensor = Sensor(
        id=2,
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        fps=30
    )

    sensor.reset()

    # --------------------------------------------------------
    # CH0：1920×1080拍照通道
    # --------------------------------------------------------

    sensor.set_framesize(
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        chn=CAPTURE_CH
    )

    sensor.set_pixformat(
        Sensor.RGB888,
        chn=CAPTURE_CH
    )

    # --------------------------------------------------------
    # CH1：640×360实时预览通道
    # --------------------------------------------------------

    sensor.set_framesize(
        width=PREVIEW_WIDTH,
        height=PREVIEW_HEIGHT,
        chn=PREVIEW_CH
    )

    sensor.set_pixformat(
        Sensor.YUV420SP,
        chn=PREVIEW_CH
    )

    # --------------------------------------------------------
    # 绑定预览视频层
    # --------------------------------------------------------

    bind_info = sensor.bind_info(
        x=PREVIEW_X,
        y=PREVIEW_Y,
        chn=PREVIEW_CH
    )

    Display.bind_layer(
        **bind_info,
        layer=Display.LAYER_VIDEO1
    )

    # --------------------------------------------------------
    # 输出到板载LCD和CanMV IDE
    # --------------------------------------------------------

    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True,
        quality=IDE_PREVIEW_QUALITY
    )

    display_inited = True

    sensor.run()

    print("Warming up camera...")

    for i in range(30):
        time.sleep_ms(50)
        os.exitpoint()

    print("")
    print("Camera ready.")
    print("CanMV IDE preview is running.")
    print("Save directory:", SAVE_DIR)
    print("Existing photo count:", photo_id)
    print("")
    print("Press the onboard KEY to capture.")

    # --------------------------------------------------------
    # 按键检测
    # --------------------------------------------------------

    while True:
        os.exitpoint()

        if key.is_pressed():
            # 消抖
            time.sleep_ms(30)

            if key.is_pressed():
                try:
                    capture_photo()
                except Exception as e:
                    print("CAPTURE_ERROR:", repr(e))

                # 等待按键释放，避免长按连续拍照
                while key.is_pressed():
                    os.exitpoint()
                    time.sleep_ms(20)

        time.sleep_ms(20)


# ============================================================
# 异常处理
# ============================================================

except KeyboardInterrupt:
    print("")
    print("Stopped by CanMV IDE.")


except Exception as e:
    print("")
    print("PROGRAM_ERROR:", repr(e))


# ============================================================
# 资源释放
# ============================================================

finally:
    if isinstance(sensor, Sensor):
        try:
            sensor.stop()
        except Exception as e:
            print("Sensor stop error:", repr(e))

    if display_inited:
        try:
            Display.deinit()
        except Exception as e:
            print("Display deinit error:", repr(e))

    gc.collect()

    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

    print("Camera stopped.")
