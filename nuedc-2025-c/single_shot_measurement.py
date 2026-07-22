"""Trigger-driven distance and basic-target size measurement.

Idle state only previews the camera.  Each UART1 trigger collects up to five
independent measurements until two STRONG 1080p refinements agree in scale.
The accepted high-resolution frame is then mapped to the physical A4 plane,
the central basic shape is classified/measured, and one combined D/x result is
saved under /data/captures.  Its outlines remain fixed until the next trigger.
"""

import gc
import os
import time

from media.sensor import *
from media.display import *
# Keep the vendor wrapper as a fallback, but initialize UART1 explicitly in
# the production path.  The wrapper shipped with the Yahboom image currently
# maps the same pins, however making the FPIOA mapping visible here prevents a
# future ybUtils update (or a cleaned sdcard) from silently moving the screen
# to another UART.
try:
    from machine import UART, FPIOA
except Exception:
    UART = None
    FPIOA = None

# ``machine.UART`` is the production path.  Keep the Yahboom wrapper
# optional: a cleaned deployment may retain the camera/display drivers while
# omitting ``ybUtils/YbUart.py``.  An unconditional import here would make the
# native UART path fail before ``create_uart1()`` gets a chance to run.
try:
    from ybUtils.YbUart import YbUart
except Exception:
    YbUart = None

from frame_detector import FrameDetector
from high_res_refiner import HighResRefiner
from inner_aperture_locator import InnerApertureLocator, scale_seed
from distance_estimator import (
    estimate_distance,
    fuse_distance_results,
    frame_scale,
)
from measurement_consistency import (
    relative_scale_difference,
    relative_center_shift,
)
from plane_mapper import PlaneMapper
from basic_shape_detector import (
    BasicShapeDetector,
    draw_shape_overlay,
    scale_shape_result,
)
from advanced_target_detector import (
    AdvancedTargetDetector,
    MODE_AUTO_MIN,
    MODE_DIGIT_SELECT,
    MODE_TILT_MIN,
    advanced_results_consistent,
    draw_advanced_overlay,
    scale_advanced_result,
)
from power_monitor import (
    PowerMonitor,
    create_ina226_monitor,
    parse_current_power_sample,
    parse_power_sample,
)
from tjc_screen import TJCScreen


SAVE_DIR = "/data/captures"
PIPELINE_VERSION = "2026-07-22-screen-fastdigit-tilt-v9-currentcal"

CAPTURE_CH = CAM_CHN_ID_0
CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080

PREVIEW_CH = CAM_CHN_ID_1
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 360

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
PREVIEW_X = (DISPLAY_WIDTH - PREVIEW_WIDTH) // 2
PREVIEW_Y = (DISPLAY_HEIGHT - PREVIEW_HEIGHT) // 2

IDE_PREVIEW_QUALITY = 55
SAVED_JPG_QUALITY = 95
PREVIEW_JPG_QUALITY = 90

RECT_THRESHOLD = 1800
FALLBACK_RECT_THRESHOLD = 1200
COARSE_HYPOTHESES_PER_ATTEMPT = 4
MAX_HIGH_RES_HYPOTHESES = 3
MAX_APERTURE_HYPOTHESES = 2
HIGH_RES_FLUSH_FRAMES = 2

# In competition the camera is adjusted so the target stays horizontally
# centred. The stand cannot guarantee its vertical position, so this is a
# strict full-height vertical strip: no off-centre full-frame fallback exists.
# Guides are drawn only after detection has finished.
CENTER_SEARCH_CENTER_X_RATIO = 0.54
CENTER_SEARCH_WIDTH_RATIO = 0.34
ALIGNMENT_CROSS_HALF_LENGTH = 10
CENTER_ROI_GUIDE_COLOR = (0, 220, 255)
CENTER_CROSS_COLOR = (255, 255, 0)

# Expected 640x360 inner-aperture envelope over the calibrated 100--200 cm
# range, with margin for perspective and print/camera tolerances.
CENTER_MIN_SHORT_PX = 24.0
CENTER_MAX_SHORT_PX = 88.0
CENTER_MIN_LONG_PX = 35.0
CENTER_MAX_LONG_PX = 135.0
CENTER_MIN_ASPECT = 1.15
CENTER_MAX_ASPECT = 1.90

# Describe a generous diagnostic ROI around the first STRONG frame.  The next
# independent frame in the same trigger uses its strict corners directly;
# preview-ROI find_rects() and cross-trigger tracking are deliberately disabled.
TRACKING_ROI_MARGIN_SCALE = 1.0
TRACKING_ROI_MIN_MARGIN = 24
TRACKING_ROI_MIN_SIZE = 48
MAX_COARSE_DIAGNOSTICS = 12

# One UART/touch-screen trigger starts independent captures until two STRONG
# frame scales agree.  No mechanical-key debounce delay is needed.
MAX_MEASUREMENT_ATTEMPTS = 5
MAX_TILT_MEASUREMENT_ATTEMPTS = 3
MAX_FRAME_SCALE_DISAGREEMENT = 0.0050
MAX_FRAME_CENTER_SHIFT_RATIO = 0.10
# The BASIC distance path keeps the strict parallel-sheet anisotropy gate in
# HighResRefiner.  For advanced targets, the subsequent plane/shape detector
# and two-frame identity gate provide an independent rejection mechanism; a
# slightly wider limit prevents harmless 0.09--0.13 perspective/noise values
# from blocking otherwise clear numbered/overlap targets.
ADVANCED_MAX_INNER_SCALE_ANISOTROPY = 0.14

# Header pin 3 is UART1_TXD and header pin 4 is UART1_RXD on the Yahboom K230.
UART_BAUDRATE = 115200
UART_TX_PIN = 9
UART_RX_PIN = 10
UART_RX_BUFFER_LIMIT = 64
USE_TJC_SCREEN = True

# INA226 R100 module supplied for the K230.  The driver configures IIC1 on
# GPIO34/35 and uses address 0x40 by default.  If the module is unplugged,
# startup falls back to the existing UART/manual power input path.
USE_INA226 = True
INA226_I2C_ID = 1
INA226_SCL_PIN = 34
INA226_SDA_PIN = 35
INA226_ADDRESS = 0x40
INA226_FREQ = 100000
INA226_R_SHUNT_OHM = 0.1
INA226_CURRENT_LSB_A = 25e-6
# Empirical correction measured against the external current reference.
# Applied only to INA226 hardware samples; UART/manual I/P commands remain
# unchanged.
INA226_CURRENT_OFFSET_A = 0.077
INA226_CONFIG = 0x4527
INA226_POLL_INTERVAL_MS = 100

# Power widgets on the external TJC screen are refreshed independently of
# measurement triggers.  One combined UART write updates I/P/Pmax at 5 Hz,
# which looks real-time without loading the 115200-baud touch-screen link.
SCREEN_POWER_UPDATE_INTERVAL_MS = 200

# Exact dimensions from the official target definition.
EXPECTED_SHORT_RATIO = 17.0 / 21.0
EXPECTED_LONG_RATIO = 25.7 / 29.7
EXPECTED_OUTER_ASPECT = 29.7 / 21.0
EXPECTED_INNER_ASPECT = 25.7 / 17.0
INNER_PLANE_WIDTH_CM = 17.0
INNER_PLANE_HEIGHT_CM = 25.7


sensor = None
display_inited = False
uart = None
uart_rx_buffer = ""
screen = None
screen_status_code = TJCScreen.STATUS_IDLE
measurement_id = 0
power_monitor = None
last_measurement_failure_reason = None
last_screen_power_update_ms = None


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def create_uart1():
    """Create the screen/debug UART with an explicit GPIO9/10 mapping.

    ``LCD_test.py`` supplied with the board uses the native ``machine`` API.
    Use that API first so the production program does not depend on an
    implementation detail of ``ybUtils.YbUart``.  Older Yahboom images have
    shipped slightly different UART constructor signatures, so a conservative
    fallback to the vendor wrapper keeps the deployment compatible.
    """
    if UART is not None and FPIOA is not None:
        try:
            fpioa = FPIOA()
            try:
                fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
                fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
            except TypeError:
                # Some CanMV builds require the input/output enable keywords.
                fpioa.set_function(
                    UART_TX_PIN,
                    FPIOA.UART1_TXD,
                    ie=0,
                    oe=1,
                    pu=1,
                )
                fpioa.set_function(
                    UART_RX_PIN,
                    FPIOA.UART1_RXD,
                    ie=1,
                    oe=0,
                    pu=1,
                )
            return UART(
                UART.UART1,
                baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE,
                timeout=0,
            )
        except TypeError:
            # A few firmware revisions accept positional baudrate only.
            try:
                return UART(UART.UART1, UART_BAUDRATE)
            except Exception as error:
                print("NATIVE_UART1_ERROR:", repr(error))
        except Exception as error:
            print("NATIVE_UART1_ERROR:", repr(error))
    if YbUart is None:
        raise RuntimeError(
            "UART1 unavailable: machine.UART/FPIOA failed and ybUtils.YbUart is missing"
        )
    # Wrapper signatures differ between Yahboom images.  Try the documented
    # keyword first, then the common positional forms without masking the
    # original native-UART diagnostic.
    try:
        return YbUart(baudrate=UART_BAUDRATE)
    except TypeError:
        try:
            return YbUart(UART_BAUDRATE)
        except TypeError:
            return YbUart()


def safe_screen_call(method_name, *args):
    """Call one screen method without allowing a cable fault to stop vision."""
    if screen is None:
        return False
    try:
        getattr(screen, method_name)(*args)
        return True
    except Exception as error:
        print("SCREEN_%s_ERROR:" % method_name.upper(), repr(error))
        return False


def screen_power_values(force=False):
    """Return current, power and peak values for the screen widgets."""
    if power_monitor is None:
        return 0.0, 0.0, 0.0
    power_monitor.poll_hardware(force=force)
    status = power_monitor.status()
    current_a = status.get("current_a")
    power_w = status.get("current_w")
    maximum_w = status.get("maximum_w")
    return (
        0.0 if current_a is None else float(current_a),
        0.0 if power_w is None else float(power_w),
        0.0 if maximum_w is None else float(maximum_w),
    )


def periodic_screen_power_update(force=False):
    """Refresh I/P/Pmax on the TJC screen at a bounded real-time rate."""
    global last_screen_power_update_ms
    if screen is None or power_monitor is None:
        return False

    now = time.ticks_ms()
    if (
        not force
        and last_screen_power_update_ms is not None
        and time.ticks_diff(now, last_screen_power_update_ms)
        < SCREEN_POWER_UPDATE_INTERVAL_MS
    ):
        return False

    current_a, power_w, pmax_w = screen_power_values(force=force)
    if not safe_screen_call(
        "update_power", current_a, power_w, pmax_w
    ):
        return False
    last_screen_power_update_ms = now
    return True


def service_power_telemetry(force=False):
    """Sample INA226 and refresh the screen between blocking vision stages."""
    if power_monitor is None:
        return False
    power_monitor.poll_hardware(force=force)
    periodic_screen_power_update(force=force)
    return True


def set_screen_status(status_code):
    global screen_status_code
    screen_status_code = int(status_code)
    safe_screen_call("set_status", screen_status_code)


def refresh_touch_screen(result=None):
    current_a, power_w, pmax_w = screen_power_values(force=True)
    if result is not None:
        distance_cm = result.get("distance_cm", 0.0)
        if result.get("task_mode") == MODE_TILT_MIN:
            distance_cm = 0.0
        safe_screen_call(
            "update_result",
            distance_cm,
            result.get("x_cm", 0.0),
            current_a,
            power_w,
            pmax_w,
            screen_status_code,
        )
    else:
        safe_screen_call("update_power", current_a, power_w, pmax_w)
        safe_screen_call("set_status", screen_status_code)


def set_screen_task_label(task_mode, target_digit=None):
    # The TJC guide deliberately keeps ``main.tTarget`` and ``main.tStatus``
    # under screen-side event/timer control.  K230 only writes the numeric
    # status/value controls, so a delayed screen timer cannot be overwritten by
    # a stale text command from the vision loop.
    return


def command_from_screen_frame(frame):
    """Map the guide's 7-byte touch-screen frame to an existing task."""
    if frame.command == TJCScreen.CMD_START_MEASURE:
        if frame.data0 == TJCScreen.MODE_AUTO:
            return "MEASURE"
        if frame.data0 == TJCScreen.MODE_MIN_SQUARE:
            return "ADVANCED_MIN"
        if frame.data0 == TJCScreen.MODE_NUMBERED_SQUARE:
            if 0 <= frame.data1 <= 9:
                return "DIGIT:%d" % frame.data1
            return "SCREEN_BAD_DIGIT"
        if frame.data0 == TJCScreen.MODE_TILTED:
            return "TILT_MIN"
        return "SCREEN_BAD_MODE"
    if frame.command == TJCScreen.CMD_RESET_PMAX:
        return "POWER_RESET"
    if frame.command == TJCScreen.CMD_SCREEN_READY:
        return "SCREEN_READY"
    if frame.command == TJCScreen.CMD_REQUEST_REFRESH:
        return "SCREEN_REFRESH"
    return "SCREEN_UNKNOWN_CMD"


def poll_screen_command():
    if screen is None:
        return None
    try:
        frames = screen.poll()
    except Exception as error:
        print("SCREEN_RX_ERROR:", repr(error))
        return None
    if not frames:
        return None
    # Return at most one frame per preview loop. Extra complete frames remain
    # semantically redundant (touch buttons are edge-triggered); dropping them
    # also prevents a queued second measurement after a blocking five-second
    # operation.
    command = None
    measurement_command = None
    for frame in frames:
        mapped = command_from_screen_frame(frame)
        print("SCREEN_RX", repr(frame), "mapped=%s" % mapped)
        if mapped in (
            "MEASURE",
            "ADVANCED_MIN",
            "TILT_MIN",
        ) or (
            mapped is not None and mapped.startswith("DIGIT:")
        ):
            # A screen-ready/refresh frame may share one UART read with a
            # touch event.  Prefer the actionable measurement instead of
            # silently dropping it as housekeeping.
            if measurement_command is None:
                measurement_command = mapped
        elif command is None:
            command = mapped
    return measurement_command if measurement_command is not None else command


def uart_send_line(message):
    # Production UART1 is dedicated to the TJC binary/ASCII screen protocol.
    # Sending terminal log lines into that link would be interpreted as screen
    # commands, so diagnostics stay on the CanMV IDE console.
    if screen is not None:
        print("UART_DEBUG:", message)
        return
    if uart is None:
        return
    payload = (str(message) + "\n").encode("ascii")
    try:
        writer = getattr(uart, "write", None)
        if writer is None:
            writer = getattr(uart, "send", None)
        if writer is None:
            raise AttributeError("UART object has no write/send method")
        try:
            writer(payload)
        except TypeError:
            # The vendor wrapper also accepts a text string on some builds.
            writer(str(message) + "\n")
    except Exception as error:
        print("UART_TX_ERROR:", repr(error))


def normalize_uart_command(raw_command):
    command = raw_command.strip().upper()
    if not command:
        return None
    if command in ("M", "MEASURE", "1"):
        return "MEASURE"
    if command in ("A", "AUTO", "MIN", "2"):
        return "ADVANCED_MIN"
    if command in ("T", "TILT", "4"):
        return "TILT_MIN"
    if command.startswith("N") and len(command) == 2 and command[1].isdigit():
        return "DIGIT:" + command[1]
    if command.startswith("DIGIT=") and command[6:].isdigit():
        return "DIGIT:" + command[6:]
    if command in ("S", "STATUS", "?"):
        return "STATUS"
    if command in ("P", "PING"):
        return "PING"
    if command in ("PW", "POWER", "POWER?"):
        return "POWER_STATUS"
    if command in ("R", "POWER_RESET", "RESET_POWER"):
        return "POWER_RESET"
    if command in ("H", "HELP"):
        return "HELP"
    if "I" in command:
        current_power = parse_current_power_sample(command)
        if current_power is not None:
            current_a, power_w = current_power
            return "POWER_SAMPLE_PAIR:%s:%s" % (
                "" if current_a is None else "%.6f" % current_a,
                "" if power_w is None else "%.6f" % power_w,
            )
    power_sample = parse_power_sample(command)
    if power_sample is not None:
        return "POWER_SAMPLE:%.6f" % power_sample
    return "UNKNOWN:" + command


def poll_uart_command():
    global uart_rx_buffer
    try:
        data = uart.read()
    except Exception as error:
        print("UART_RX_ERROR:", repr(error))
        return None
    if not data:
        return None
    try:
        text = data.decode()
    except Exception:
        text = str(data)
    uart_rx_buffer += text.replace("\r", "\n")

    while True:
        newline_index = uart_rx_buffer.find("\n")
        if newline_index < 0:
            break
        line = uart_rx_buffer[:newline_index]
        uart_rx_buffer = uart_rx_buffer[newline_index + 1:]
        command = normalize_uart_command(line)
        if command is not None:
            return command

    # Single-byte commands work immediately in a raw serial terminal.  Full
    # words require CR/LF so a fragmented word cannot trigger twice.
    if uart_rx_buffer in (
        "M", "m", "1", "A", "a", "2", "T", "t", "4",
        "S", "s", "P", "p", "R", "r", "H", "h", "?",
    ):
        command = normalize_uart_command(uart_rx_buffer)
        uart_rx_buffer = ""
        return command
    if (
        len(uart_rx_buffer) == 2
        and uart_rx_buffer[0] in ("N", "n")
        and uart_rx_buffer[1].isdigit()
    ):
        command = normalize_uart_command(uart_rx_buffer)
        uart_rx_buffer = ""
        return command
    if len(uart_rx_buffer) > UART_RX_BUFFER_LIMIT:
        command = "UNKNOWN:" + uart_rx_buffer[:UART_RX_BUFFER_LIMIT]
        uart_rx_buffer = ""
        return command
    return None


def drain_uart_input():
    """Discard commands queued while a blocking measurement was running."""
    global uart_rx_buffer
    uart_rx_buffer = ""
    if screen is not None:
        try:
            screen.discard_input(4)
        except Exception as error:
            print("SCREEN_DRAIN_ERROR:", repr(error))
        return
    for _ in range(4):
        try:
            data = uart.read()
        except Exception:
            return
        if not data:
            return
        time.sleep_ms(2)


def send_uart_status(state, current_id, result):
    if state == "LOCKED" and result is not None:
        if result.get("task_mode") == MODE_TILT_MIN:
            uart_send_line(
                "STATUS LOCKED ID=%03d TASK=%s X=%.2f TYPE=%s TIME=%d VER=%s"
                % (
                    current_id,
                    MODE_TILT_MIN,
                    result.get("x_cm", 0.0),
                    result.get("shape_type", "UNKNOWN"),
                    result.get("total_ms", 0),
                    PIPELINE_VERSION,
                )
            )
        else:
            uart_send_line(
                "STATUS LOCKED ID=%03d TASK=%s D=%.2f X=%.2f TYPE=%s METHOD=%s SCALE=%.6f TIME=%d VER=%s"
                % (
                    current_id,
                    result.get("task_mode", "BASIC"),
                    result.get("distance_cm", 0.0),
                    result.get("x_cm", 0.0),
                    result.get("shape_type", "UNKNOWN"),
                    result.get("method", "UNKNOWN"),
                    result.get("frame_scale", 0.0),
                    result.get("total_ms", 0),
                    PIPELINE_VERSION,
                )
            )
    elif state == "FAILED":
        uart_send_line(
            "STATUS FAILED ID=%03d NEXT=%03d VER=%s"
            % (current_id, current_id + 1, PIPELINE_VERSION)
        )
    else:
        uart_send_line(
            "STATUS IDLE NEXT=%03d VER=%s"
            % (current_id + 1, PIPELINE_VERSION)
        )


def find_last_measurement_id():
    maximum = 0
    try:
        filenames = os.listdir(SAVE_DIR)
    except OSError:
        return 0
    for filename in filenames:
        if not filename.startswith("measure_"):
            continue
        number_text = filename[8:11]
        if number_text.isdigit():
            maximum = max(maximum, int(number_text))
    return maximum


def draw_quad(image, corners, color, thickness=1):
    if corners is None:
        return
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        image.draw_line(
            int(first[0]),
            int(first[1]),
            int(second[0]),
            int(second[1]),
            color=color,
            thickness=thickness,
        )


def center_search_roi(image_width, image_height):
    """Return the full-height, slightly right-shifted coarse-search strip."""
    roi_width = max(
        2,
        int(image_width * CENTER_SEARCH_WIDTH_RATIO + 0.5),
    )
    roi_width = min(roi_width, image_width)
    center_x = int(image_width * CENTER_SEARCH_CENTER_X_RATIO + 0.5)
    x0 = center_x - roi_width // 2
    x0 = max(0, min(x0, image_width - roi_width))
    return (x0, 0, roi_width, image_height)


def draw_alignment_guides(image):
    """Draw the search-strip boundaries and the optical-center crosshair."""
    width = image.width()
    height = image.height()
    roi = center_search_roi(width, height)
    left_x = roi[0]
    right_x = roi[0] + roi[2] - 1
    image.draw_line(
        left_x,
        0,
        left_x,
        height - 1,
        color=CENTER_ROI_GUIDE_COLOR,
        thickness=1,
    )
    image.draw_line(
        right_x,
        0,
        right_x,
        height - 1,
        color=CENTER_ROI_GUIDE_COLOR,
        thickness=1,
    )

    # The crosshair uses the same calibrated optical centre as the search
    # strip. Its y coordinate is only an aiming aid, never a search gate.
    center_x = roi[0] + roi[2] // 2
    center_y = height // 2
    half = ALIGNMENT_CROSS_HALF_LENGTH
    image.draw_line(
        max(0, center_x - half),
        center_y,
        min(width - 1, center_x + half),
        center_y,
        color=CENTER_CROSS_COLOR,
        thickness=1,
    )
    image.draw_line(
        center_x,
        max(0, center_y - half),
        center_x,
        min(height - 1, center_y + half),
        color=CENTER_CROSS_COLOR,
        thickness=1,
    )


def draw_measurement_overlay(image, result, status_color, accepted=True):
    if result is not None:
        if accepted:
            outer_color = (255, 50, 50)
            inner_color = (0, 255, 80)
        else:
            outer_color = (255, 120, 0)
            inner_color = (255, 120, 0)
        draw_quad(image, result.get("outer_corners"), outer_color, 1)
        draw_quad(image, result.get("inner_corners"), inner_color, 1)
        shape_result = result.get("shape_result")
        if shape_result is not None and shape_result.get("mode") in (
            MODE_AUTO_MIN,
            MODE_DIGIT_SELECT,
            MODE_TILT_MIN,
        ):
            draw_advanced_overlay(
                image,
                shape_result,
                selected_color=(255, 40, 40),
                other_color=(0, 255, 80),
                thickness=1,
            )
        else:
            draw_shape_overlay(
                image,
                shape_result,
                color=(255, 255, 0),
                thickness=1,
            )
    image.draw_rectangle(4, 4, 12, 12, color=status_color, thickness=-1)


def scale_corners(corners, scale_x, scale_y):
    if corners is None:
        return None
    scaled = []
    for x, y in corners:
        scaled.append((
            int(round(x * scale_x)),
            int(round(y * scale_y)),
        ))
    return tuple(scaled)


def high_result_to_preview(high_result):
    if high_result is None:
        return None
    scale_x = PREVIEW_WIDTH / CAPTURE_WIDTH
    scale_y = PREVIEW_HEIGHT / CAPTURE_HEIGHT
    return {
        "mode": high_result["mode"],
        "outer_corners": scale_corners(
            high_result["outer_corners"], scale_x, scale_y
        ),
        "inner_corners": scale_corners(
            high_result["inner_corners"], scale_x, scale_y
        ),
    }


def tracking_seed_from_result(result):
    """Turn the last strict 1080p lock into a direct local-refinement seed."""
    if result is None:
        return None
    inner_corners = result.get("inner_corners")
    if inner_corners is None or len(inner_corners) < 4:
        return None
    return {
        "mode": "TRACK_HINT",
        "outer_corners": result.get("outer_corners"),
        "inner_corners": tuple(inner_corners),
        "outer_bbox": None,
        "inner_bbox": None,
        "score": 1.0,
        "contrast": 0.0,
        "short_ratio": EXPECTED_SHORT_RATIO,
        "long_ratio": EXPECTED_LONG_RATIO,
        "area_ratio": EXPECTED_SHORT_RATIO * EXPECTED_LONG_RATIO,
        "outer_valid_sides": 4 if result.get("outer_corners") else 0,
        "outer_edge_response": 0.0,
        "candidate_count": 1,
        "coarse_soft_reasons": (),
        "coarse_regularity": {},
        "seed_class": "TRACK_SEED",
        "location_seed_relaxed": False,
    }


def tracking_roi_from_result(result):
    """Build a clipped preview ROI around the last accepted target."""
    if result is None:
        return None
    corners = result.get("outer_corners")
    if corners is None:
        corners = result.get("inner_corners")
    if corners is None or len(corners) < 4:
        return None

    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    x0 = min(xs)
    y0 = min(ys)
    x1 = max(xs)
    y1 = max(ys)
    target_width = max(1, x1 - x0 + 1)
    target_height = max(1, y1 - y0 + 1)
    margin_x = max(
        TRACKING_ROI_MIN_MARGIN,
        int(round(target_width * TRACKING_ROI_MARGIN_SCALE)),
    )
    margin_y = max(
        TRACKING_ROI_MIN_MARGIN,
        int(round(target_height * TRACKING_ROI_MARGIN_SCALE)),
    )

    roi_x0 = max(0, int(x0) - margin_x)
    roi_y0 = max(0, int(y0) - margin_y)
    roi_x1 = min(PREVIEW_WIDTH - 1, int(x1) + margin_x)
    roi_y1 = min(PREVIEW_HEIGHT - 1, int(y1) + margin_y)
    roi_width = roi_x1 - roi_x0 + 1
    roi_height = roi_y1 - roi_y0 + 1
    if (
        roi_width < TRACKING_ROI_MIN_SIZE
        or roi_height < TRACKING_ROI_MIN_SIZE
    ):
        return None
    return (roi_x0, roi_y0, roi_width, roi_height)


def format_roi(roi):
    if roi is None:
        return "NONE"
    return "%d,%d,%d,%d" % roi


def merge_rejection_counts(destination, source):
    for reason in source:
        destination[reason] = (
            destination.get(reason, 0) + source[reason]
        )


def format_rejection_counts(counts):
    if not counts:
        return "NONE"
    parts = []
    for reason in sorted(counts):
        parts.append("%s:%d" % (reason, counts[reason]))
    return ",".join(parts)


def format_reason_list(reasons):
    if not reasons:
        return "NONE"
    return ",".join(reasons)


def has_usable_location_seed(results):
    """Return True when 1080p validation has a bounded place to inspect."""
    for result in results:
        if result.get("seed_class", "REJECT") != "REJECT":
            return True
    return False


def has_trusted_coarse_seed(results):
    """Return True only for a seed whose preview corners are directly trusted."""
    for result in results:
        if result.get("seed_class") in ("CLEAN_SEED", "TRACK_SEED"):
            return True
    return False


def _coarse_quad_dimensions(corners):
    if corners is None or len(corners) < 4:
        return None
    sides = []
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        delta_x = second[0] - first[0]
        delta_y = second[1] - first[1]
        sides.append((delta_x * delta_x + delta_y * delta_y) ** 0.5)
    dimension_a = (sides[0] + sides[2]) * 0.5
    dimension_b = (sides[1] + sides[3]) * 0.5
    short_side = min(dimension_a, dimension_b)
    long_side = max(dimension_a, dimension_b)
    return short_side, long_side


def filter_center_hypotheses(results, image, allow_projective=False):
    """Apply the calibrated distance envelope to central normal-view seeds."""
    if allow_projective:
        return list(results), 0
    scale = min(
        image.width() / PREVIEW_WIDTH,
        image.height() / PREVIEW_HEIGHT,
    )
    accepted = []
    rejected = 0
    for result in results:
        dimensions = _coarse_quad_dimensions(result.get("inner_corners"))
        if dimensions is None:
            rejected += 1
            continue
        short_side, long_side = dimensions
        aspect = long_side / max(short_side, 1.0)
        if (
            short_side < CENTER_MIN_SHORT_PX * scale
            or short_side > CENTER_MAX_SHORT_PX * scale
            or long_side < CENTER_MIN_LONG_PX * scale
            or long_side > CENTER_MAX_LONG_PX * scale
            or aspect < CENTER_MIN_ASPECT
            or aspect > CENTER_MAX_ASPECT
        ):
            rejected += 1
            continue
        accepted.append(result)
    return accepted, rejected


def run_aperture_scan(
    image,
    locator,
    result,
    scope,
    roi,
    source,
    allow_projective=False,
):
    """Find a bright inner aperture and record bounded diagnostics."""
    start = time.ticks_ms()
    hypotheses = locator.detect_hypotheses(
        image,
        roi=roi,
        max_results=MAX_APERTURE_HYPOTHESES,
        source=source,
        allow_projective=allow_projective,
    )
    elapsed = time.ticks_diff(time.ticks_ms(), start)
    result["coarse_ms"] += elapsed
    result["aperture_ms"] += elapsed
    counts = {}
    for reason in locator.last_rejection_counts:
        counts[reason] = locator.last_rejection_counts[reason]
    scan = {
        "scope": scope,
        "source": source,
        "allow_projective": bool(allow_projective),
        "roi": roi,
        "blobs": locator.last_blob_count,
        "candidates": len(locator.last_candidates),
        "elapsed_ms": elapsed,
        "reject_counts": counts,
        "error": locator.last_error,
    }
    result["aperture_scans"].append(scan)
    for candidate in locator.last_candidates:
        result["aperture_diagnostics"].append(candidate)
        if (
            image.width() == PREVIEW_WIDTH
            and image.height() == PREVIEW_HEIGHT
            and len(result["coarse_diagnostics"]) < MAX_COARSE_DIAGNOSTICS
        ):
            result["coarse_diagnostics"].append({
                "mode": candidate.get("mode", "INNER_APERTURE_SEED"),
                "inner_corners": candidate.get("inner_corners"),
                "outer_corners": None,
                "hard_reject_reason": "NONE",
                "reject_reason": "NONE",
                "soft_reasons": (),
                "seed_class": candidate.get("seed_class", "ROI_SEED"),
                "projective_seed": candidate.get(
                    "projective_seed", False
                ),
                "location_seed_relaxed": False,
                "area_ratio": candidate.get("area_ratio", -1.0),
                "aspect": candidate.get("aperture_aspect", -1.0),
                "contrast": candidate.get(
                    "aperture_border_contrast", -999.0
                ),
                "inner_score": candidate.get("score", -1.0),
                "ring_inside_contrast": candidate.get(
                    "aperture_border_contrast", -999.0
                ),
                "ring_outside_contrast": -999.0,
                "ring_inside_pass_ratio": candidate.get(
                    "aperture_border_pass_ratio", 0.0
                ),
                "ring_outside_pass_ratio": 0.0,
                "ring_min_side_pass_ratio": candidate.get(
                    "aperture_min_side_pass_ratio", 0.0
                ),
                "regularity_max_opposite_error": 0.0,
                "regularity_diagonal_midpoint_error": 0.0,
                "regularity_fill_ratio": 1.0,
                "regularity_min_corner_angle": 90.0,
                "regularity_max_corner_angle": 90.0,
            })
    print(
        "aperture scope=%s source=%s projective=%d blobs=%d candidates=%d returned=%d reject=%s error=%s time=%dms"
        % (
            scope,
            source,
            1 if allow_projective else 0,
            scan["blobs"],
            scan["candidates"],
            len(hypotheses),
            format_rejection_counts(counts),
            scan["error"] or "NONE",
            elapsed,
        )
    )
    return hypotheses


def tilt_localization_result(high_result):
    """Return a consistency record for a valid tilted A4 inner aperture.

    Normal distance measurement deliberately rejects large image anisotropy.
    For the 30--60 degree task that anisotropy is expected: only four genuine
    inner edges, bounds, ring contrast and repeatable projective geometry are
    required.  The returned scale is used solely for the two-frame gate and is
    never converted to D.
    """
    if high_result is None:
        return None
    if not high_result.get("localization_valid", False):
        return None
    if high_result.get("inner_valid_sides", 0) < 4:
        return None
    if not high_result.get("inner_geometry_valid", False):
        return None
    if not high_result.get("inner_in_bounds", True):
        return None
    inner_rms = high_result.get("inner_edge_rms", -1.0)
    if inner_rms < 0.0 or inner_rms > 5.0:
        return None
    if high_result.get("outer_valid_sides", 0) < 3:
        if high_result.get("ring_valid_samples", 0) < 16:
            return None
        if high_result.get("ring_inside_contrast", -999.0) < 8.0:
            return None
        if high_result.get("ring_inside_pass_ratio", 0.0) < 0.28:
            return None
        if high_result.get("ring_min_side_pass_ratio", 0.0) < 0.12:
            return None

    measurement = frame_scale(
        high_result.get(
            "inner_corners_float", high_result.get("inner_corners")
        ),
        INNER_PLANE_WIDTH_CM,
        INNER_PLANE_HEIGHT_CM,
    )
    if measurement is None:
        return None
    return {
        "distance_cm": 0.0,
        "method": "PLANE_HOMOGRAPHY_ONLY",
        "source": "TILT_INNER_APERTURE",
        "scale_method": measurement["scale_method"],
        "frame_scale": measurement["frame_scale"],
        "scale_x": measurement["scale_x"],
        "scale_y": measurement["scale_y"],
        "anisotropy": measurement["anisotropy"],
        "width_px": measurement["width_px"],
        "height_px": measurement["height_px"],
        "area_px2": measurement["area_px2"],
        "outer_inner_disagreement": 0.0,
        "in_official_range": True,
        "tilt_localization_only": True,
    }


def fuse_tilt_localizations(first, second):
    scale_first = first["frame_scale"]
    scale_second = second["frame_scale"]
    fused = dict(second)
    fused_scale = (scale_first + scale_second) * 0.5
    spread = abs(scale_first - scale_second) / max(fused_scale, 1.0e-6)
    fused["frame_scale"] = fused_scale
    fused["fusion_method"] = "TWO_FRAME_TILT_GEOMETRY_MEAN"
    fused["fusion_count"] = 2
    fused["component_scales"] = (scale_first, scale_second)
    fused["frame_scale_relative_spread"] = spread
    fused["frame_scale_spread_pct"] = spread * 100.0
    return fused


def record_coarse_scan(result, detector, pass_index, scope, elapsed_ms):
    counts = {}
    for reason in detector.last_rejection_counts:
        counts[reason] = detector.last_rejection_counts[reason]
    merge_rejection_counts(result["coarse_reject_counts"], counts)

    soft_counts = {}
    for reason in detector.last_soft_gate_counts:
        soft_counts[reason] = detector.last_soft_gate_counts[reason]
    merge_rejection_counts(result["coarse_soft_counts"], soft_counts)

    diagnostics = []
    for candidate_index in range(len(detector.last_candidates)):
        if len(diagnostics) >= MAX_COARSE_DIAGNOSTICS:
            break
        candidate = detector.last_candidates[candidate_index]
        ring = candidate.get("ring_metrics", {})
        regularity = candidate.get("quad_regularity", {})
        diagnostic = {
            "mode": "COARSE_CANDIDATE",
            "candidate_index": candidate_index + 1,
            "inner_corners": candidate.get("corners"),
            "outer_corners": candidate.get("predicted_outer_corners"),
            "hard_reject_reason": candidate.get(
                "inner_reject_reason"
            ) or "NONE",
            # Keep the old key for preview compatibility.
            "reject_reason": candidate.get("inner_reject_reason") or "NONE",
            "soft_reasons": candidate.get("inner_soft_reasons", ()),
            "seed_class": candidate.get("seed_class", "REJECT"),
            "location_seed_relaxed": candidate.get(
                "location_seed_relaxed", False
            ),
            "area_ratio": candidate.get("area_ratio", -1.0),
            "aspect": candidate.get("aspect", -1.0),
            "contrast": candidate.get("contrast", -999.0),
            "inner_score": candidate.get("coarse_inner_score", -1.0),
            "ring_inside_contrast": ring.get("inside_contrast", -999.0),
            "ring_outside_contrast": ring.get("outside_contrast", -999.0),
            "ring_inside_pass_ratio": ring.get("inside_pass_ratio", 0.0),
            "ring_outside_pass_ratio": ring.get("outside_pass_ratio", 0.0),
            "ring_min_side_pass_ratio": ring.get(
                "min_side_inside_pass_ratio", 0.0
            ),
            "regularity_max_opposite_error": regularity.get(
                "max_opposite_error", -1.0
            ),
            "regularity_diagonal_midpoint_error": regularity.get(
                "diagonal_midpoint_error", -1.0
            ),
            "regularity_fill_ratio": regularity.get("fill_ratio", -1.0),
            "regularity_min_corner_angle": regularity.get(
                "min_corner_angle", -1.0
            ),
            "regularity_max_corner_angle": regularity.get(
                "max_corner_angle", -1.0
            ),
        }
        diagnostics.append(diagnostic)
        result["coarse_diagnostics"].append(diagnostic)

    if len(result["coarse_diagnostics"]) > MAX_COARSE_DIAGNOSTICS:
        result["coarse_diagnostics"] = result["coarse_diagnostics"][
            -MAX_COARSE_DIAGNOSTICS:
        ]

    scan = {
        "pass_index": pass_index,
        "scope": scope,
        "threshold": detector.last_threshold,
        "raw_rects": detector.last_raw_rect_count,
        "candidates": len(detector.last_candidates),
        "pairs": detector.last_pair_count,
        "errors": detector.last_candidate_errors,
        "elapsed_ms": elapsed_ms,
        "reject_counts": counts,
        "soft_counts": soft_counts,
        "diagnostics": diagnostics,
    }
    result["coarse_scans"].append(scan)
    return scan


def run_coarse_scan(
    preview,
    detector,
    result,
    pass_index,
    scope,
    roi,
    threshold_override=None,
    scale_roi_threshold=True,
    allow_location_seed=False,
):
    start = time.ticks_ms()
    hypotheses = detector.detect_hypotheses(
        preview,
        roi=roi,
        max_results=COARSE_HYPOTHESES_PER_ATTEMPT,
        threshold_override=threshold_override,
        scale_roi_threshold=scale_roi_threshold,
        allow_location_seed=allow_location_seed,
    )
    elapsed = time.ticks_diff(time.ticks_ms(), start)
    result["coarse_ms"] += elapsed
    scan = record_coarse_scan(
        result,
        detector,
        pass_index,
        scope,
        elapsed,
    )

    if hypotheses:
        best = hypotheses[0]
        print(
            "coarse pass=%d scope=%s threshold=%d hypotheses=%d best=%s seed=%s score=%.3f raw=%d rects=%d pairs=%d errors=%d reject=%s soft=%s time=%dms"
            % (
                pass_index,
                scope,
                scan["threshold"],
                len(hypotheses),
                best["mode"],
                best.get("seed_class", "UNKNOWN"),
                best["score"],
                scan["raw_rects"],
                scan["candidates"],
                scan["pairs"],
                scan["errors"],
                format_rejection_counts(scan["reject_counts"]),
                format_rejection_counts(scan["soft_counts"]),
                elapsed,
            )
        )
    else:
        print(
            "coarse pass=%d scope=%s threshold=%d result=NONE raw=%d rects=%d pairs=%d errors=%d reject=%s soft=%s time=%dms"
            % (
                pass_index,
                scope,
                scan["threshold"],
                scan["raw_rects"],
                scan["candidates"],
                scan["pairs"],
                scan["errors"],
                format_rejection_counts(scan["reject_counts"]),
                format_rejection_counts(scan["soft_counts"]),
                elapsed,
            )
        )
    return hypotheses


def format_corners(corners):
    if corners is None:
        return "NONE"
    parts = []
    for x, y in corners:
        parts.append("(%d,%d)" % (int(x), int(y)))
    return " ".join(parts)


def collect_digit_diagnostics(shape_result):
    diagnostics = []
    if shape_result is None:
        return ()
    squares = shape_result.get("squares", ())
    for square_index in range(len(squares)):
        square = squares[square_index]
        digit_result = square.get("digit_result", {})
        center = square.get("center", (0.0, 0.0))
        diagnostics.append({
            "index": square_index,
            "side_cm": square.get("side_cm", 0.0),
            "center_x_cm": center[0],
            "center_y_cm": center[1],
            "square_score": square.get("score", 0.0),
            "digit": digit_result.get("digit"),
            "confidence": digit_result.get("confidence", 0.0),
            "margin": digit_result.get("margin", 0.0),
            "target_score": square.get("digit_target_score", 0.0),
            "target_gap": square.get("digit_target_gap", 0.0),
            "rotation_deg": digit_result.get("rotation_deg"),
            "valid": digit_result.get("valid", False),
            "reject_reason": digit_result.get(
                "reject_reason", "NOT_RUN"
            ),
        })
    return tuple(diagnostics)


def print_digit_diagnostics(attempt_index, diagnostics):
    for diagnostic in diagnostics:
        print(
            "digit_candidate attempt=%d square=%d side=%.3fcm center=%.2f/%.2f score=%.3f best=%s valid=%d confidence=%.3f margin=%.3f target=%.3f gap=%.3f rotation=%s reject=%s"
            % (
                attempt_index,
                diagnostic["index"],
                diagnostic["side_cm"],
                diagnostic["center_x_cm"],
                diagnostic["center_y_cm"],
                diagnostic["square_score"],
                str(diagnostic["digit"]),
                1 if diagnostic["valid"] else 0,
                diagnostic["confidence"],
                diagnostic["margin"],
                diagnostic["target_score"],
                diagnostic["target_gap"],
                str(diagnostic["rotation_deg"]),
                diagnostic["reject_reason"],
            )
        )


def save_metadata(
    path,
    coarse_result,
    coarse_candidate_count,
    high_result,
    distance_result,
    shape_result,
    coarse_ms,
    convert_ms,
    refine_ms,
    shape_ms,
    total_ms,
    attempt_count,
):
    with open(path, "w") as file:
        file.write("pipeline_version=%s\n" % PIPELINE_VERSION)
        file.write(
            "task_mode=%s\n"
            % (
                distance_result.get("task_mode", "BASIC")
                if distance_result is not None
                else "UNKNOWN"
            )
        )
        file.write(
            "target_digit=%s\n"
            % str(
                distance_result.get("target_digit")
                if distance_result is not None
                else None
            )
        )
        file.write("coarse_mode=%s\n" % coarse_result["mode"])
        file.write("coarse_score=%.6f\n" % coarse_result["score"])
        file.write(
            "coarse_seed_class=%s\n"
            % coarse_result.get("seed_class", "UNKNOWN")
        )
        file.write(
            "coarse_seed_source=%s\n"
            % coarse_result.get("seed_source", "FIND_RECTS")
        )
        file.write(
            "coarse_location_seed_relaxed=%s\n"
            % (
                "YES"
                if coarse_result.get("location_seed_relaxed", False)
                else "NO"
            )
        )
        coarse_regularity = coarse_result.get("coarse_regularity", {})
        file.write(
            "coarse_max_opposite_error=%.6f\n"
            % coarse_regularity.get("max_opposite_error", -1.0)
        )
        file.write(
            "coarse_diagonal_midpoint_error=%.6f\n"
            % coarse_regularity.get("diagonal_midpoint_error", -1.0)
        )
        file.write(
            "coarse_fill_ratio=%.6f\n"
            % coarse_regularity.get("fill_ratio", -1.0)
        )
        file.write(
            "coarse_corner_angle_range=%.3f,%.3f\n"
            % (
                coarse_regularity.get("min_corner_angle", -1.0),
                coarse_regularity.get("max_corner_angle", -1.0),
            )
        )
        file.write(
            "coarse_soft_reasons=%s\n"
            % format_reason_list(coarse_result.get("coarse_soft_reasons", ()))
        )
        file.write("coarse_candidate_count=%d\n" % coarse_candidate_count)
        file.write(
            "coarse_aperture_border_contrast=%.6f\n"
            % coarse_result.get("aperture_border_contrast", -1.0)
        )
        file.write(
            "coarse_aperture_border_pass_ratio=%.6f\n"
            % coarse_result.get("aperture_border_pass_ratio", -1.0)
        )
        file.write(
            "coarse_aperture_min_side_pass_ratio=%.6f\n"
            % coarse_result.get("aperture_min_side_pass_ratio", -1.0)
        )
        file.write("coarse_ms=%d\n" % coarse_ms)
        file.write("rgb888_to_rgb565_ms=%d\n" % convert_ms)
        file.write("refine_ms=%d\n" % refine_ms)
        file.write("shape_ms=%d\n" % shape_ms)
        file.write("total_ms=%d\n" % total_ms)
        file.write("measurement_attempts=%d\n" % attempt_count)
        file.write(
            "max_scale_disagreement=%.6f\n"
            % MAX_FRAME_SCALE_DISAGREEMENT
        )
        file.write(
            "max_center_shift_ratio=%.6f\n"
            % MAX_FRAME_CENTER_SHIFT_RATIO
        )
        file.write(
            "inner_scale_anisotropy_limit=%.6f\n"
            % high_result.get(
                "inner_scale_anisotropy_limit",
                0.08,
            )
        )
        file.write("high_mode=%s\n" % high_result["mode"])
        file.write(
            "measurement_valid=%s\n"
            % ("YES" if high_result["measurement_valid"] else "NO")
        )
        file.write(
            "measurement_reject_reason=%s\n"
            % high_result.get("measurement_reject_reason", "UNKNOWN")
        )
        file.write(
            "measurement_confidence=%s\n"
            % high_result.get("measurement_confidence", "UNKNOWN")
        )
        file.write("high_quality_score=%.3f\n" % high_result["quality_score"])
        file.write(
            "frame_model_disagreement=%.6f\n"
            % high_result["frame_model_disagreement"]
        )
        file.write(
            "detected_frame_disagreement=%.6f\n"
            % high_result.get("detected_frame_disagreement", 0.0)
        )
        file.write(
            "inner_valid_sides=%d\n" % high_result["inner_valid_sides"]
        )
        file.write(
            "outer_valid_sides=%d\n" % high_result["outer_valid_sides"]
        )
        file.write(
            "inner_raw_valid_sides=%d\n"
            % high_result.get("inner_raw_valid_sides", 0)
        )
        file.write(
            "outer_raw_valid_sides=%d\n"
            % high_result.get("outer_raw_valid_sides", 0)
        )
        file.write(
            "inner_geometry_valid=%s\n"
            % (
                "YES"
                if high_result.get("inner_geometry_valid", False)
                else "NO"
            )
        )
        file.write(
            "outer_geometry_valid=%s\n"
            % (
                "YES"
                if high_result.get("outer_geometry_valid", False)
                else "NO"
            )
        )
        file.write(
            "outer_independently_valid=%s\n"
            % (
                "YES"
                if high_result.get("outer_independently_valid", False)
                else "NO"
            )
        )
        file.write(
            "outer_wide_recovery_used=%s\n"
            % (
                "YES"
                if high_result.get("outer_wide_recovery_used", False)
                else "NO"
            )
        )
        file.write(
            "outer_wide_search_used=%s\n"
            % (
                "YES"
                if high_result.get("outer_wide_search_used", False)
                else "NO"
            )
        )
        file.write(
            "inner_wide_search_used=%s\n"
            % (
                "YES"
                if high_result.get("inner_wide_search_used", False)
                else "NO"
            )
        )
        file.write(
            "inner_refine_path=%s\n"
            % high_result.get("inner_refine_path", "UNKNOWN")
        )
        file.write(
            "inner_wide_search_reason=%s\n"
            % high_result.get("inner_wide_search_reason", "UNKNOWN")
        )
        file.write(
            "inner_narrow_edge_rms=%.6f\n"
            % high_result.get("inner_narrow_edge_rms", -1.0)
        )
        file.write(
            "inner_wide_edge_rms=%.6f\n"
            % high_result.get("inner_wide_edge_rms", -1.0)
        )
        file.write(
            "seed_regularized=%s\n"
            % (
                "YES"
                if high_result.get("seed_regularized", False)
                else "NO"
            )
        )
        file.write(
            "relocalize_roi=%s\n"
            % format_roi(high_result.get("relocalize_roi"))
        )
        file.write(
            "seed_regularization_angle_degrees=%.6f\n"
            % high_result.get("seed_regularization_angle_degrees", 0.0)
        )
        file.write(
            "seed_regularization_max_shift=%.6f\n"
            % high_result.get("seed_regularization_max_shift", 0.0)
        )
        file.write(
            "large_seed_shift_threshold=%.6f\n"
            % high_result.get("large_seed_shift_threshold", 0.0)
        )
        file.write(
            "large_seed_shift=%s\n"
            % (
                "YES"
                if high_result.get("large_seed_shift", False)
                else "NO"
            )
        )
        file.write(
            "outer_conflict_demoted=%s\n"
            % (
                "YES"
                if high_result.get("outer_conflict_demoted", False)
                else "NO"
            )
        )
        file.write(
            "outer_conflict_inner_ring_valid=%s\n"
            % (
                "YES"
                if high_result.get(
                    "outer_conflict_inner_ring_valid", False
                )
                else "NO"
            )
        )
        file.write(
            "inner_only_accepted=%s\n"
            % (
                "YES"
                if high_result.get("inner_only_accepted", False)
                else "NO"
            )
        )
        file.write(
            "inner_edge_response=%.3f\n"
            % high_result["inner_edge_response"]
        )
        file.write(
            "outer_edge_response=%.3f\n"
            % high_result["outer_edge_response"]
        )
        file.write(
            "inner_edge_rms=%.6f\n"
            % high_result.get("inner_edge_rms", -1.0)
        )
        file.write(
            "inner_raw_edge_rms=%.6f\n"
            % high_result.get("inner_raw_edge_rms", -1.0)
        )
        file.write(
            "outer_edge_rms=%.6f\n"
            % high_result.get("outer_edge_rms", -1.0)
        )
        file.write(
            "detected_outer_edge_rms=%.6f\n"
            % high_result.get("detected_outer_edge_rms", -1.0)
        )
        file.write(
            "inner_fill_ratio=%.6f\n"
            % high_result.get("inner_fill_ratio", 0.0)
        )
        file.write(
            "inner_max_opposite_error=%.6f\n"
            % high_result.get("inner_max_opposite_error", 1.0)
        )
        file.write(
            "inner_scale_anisotropy=%.6f\n"
            % high_result.get("inner_scale_anisotropy", 1.0)
        )
        file.write(
            "ring_inside_contrast=%.6f\n"
            % high_result.get("ring_inside_contrast", -999.0)
        )
        file.write(
            "ring_inside_pass_ratio=%.6f\n"
            % high_result.get("ring_inside_pass_ratio", 0.0)
        )
        file.write(
            "ring_outside_contrast=%.6f\n"
            % high_result.get("ring_outside_contrast", -999.0)
        )
        file.write(
            "ring_outside_pass_ratio=%.6f\n"
            % high_result.get("ring_outside_pass_ratio", 0.0)
        )
        file.write(
            "ring_min_side_pass_ratio=%.6f\n"
            % high_result.get("ring_min_side_pass_ratio", 0.0)
        )
        file.write(
            "ring_mean_thickness=%.6f\n"
            % high_result.get("ring_mean_thickness", 0.0)
        )
        file.write(
            "inner_search_radius=%d\n"
            % high_result["inner_search_radius"]
        )
        file.write(
            "outer_search_radius=%d\n"
            % high_result["outer_search_radius"]
        )
        file.write(
            "inner_corners=%s\n"
            % format_corners(high_result["inner_corners"])
        )
        file.write(
            "outer_corners=%s\n"
            % format_corners(high_result["outer_corners"])
        )
        file.write(
            "detected_outer_corners=%s\n"
            % format_corners(high_result.get("detected_outer_corners"))
        )
        file.write(
            "model_outer_corners=%s\n"
            % format_corners(high_result.get("model_outer_corners"))
        )
        if distance_result is None:
            file.write("distance_cm=NONE\n")
        else:
            file.write(
                "distance_cm=%.3f\n" % distance_result["distance_cm"]
            )
            file.write("distance_method=%s\n" % distance_result["method"])
            file.write(
                "distance_calibration_version=%s\n"
                % distance_result.get("calibration_version", "UNKNOWN")
            )
            file.write("distance_source=%s\n" % distance_result["source"])
            file.write(
                "frame_scale=%.6f\n" % distance_result["frame_scale"]
            )
            file.write(
                "scale_anisotropy=%.6f\n"
                % distance_result["anisotropy"]
            )
            file.write(
                "outer_inner_disagreement=%.6f\n"
                % distance_result["outer_inner_disagreement"]
            )
            file.write(
                "fusion_method=%s\n"
                % distance_result.get("fusion_method", "NONE")
            )
            file.write(
                "fusion_count=%d\n"
                % distance_result.get("fusion_count", 1)
            )
            component_parts = []
            for scale in distance_result.get(
                "component_scales",
                (distance_result["frame_scale"],),
            ):
                component_parts.append("%.6f" % scale)
            file.write(
                "component_frame_scales=%s\n"
                % ",".join(component_parts)
            )
            file.write(
                "frame_scale_relative_spread=%.6f\n"
                % distance_result.get("frame_scale_relative_spread", 0.0)
            )
            file.write(
                "frame_scale_spread_pct=%.6f\n"
                % distance_result.get("frame_scale_spread_pct", 0.0)
            )
            file.write(
                "frame_center_shift_ratio=%.6f\n"
                % distance_result.get("frame_center_shift_ratio", 0.0)
            )
        if shape_result is None:
            file.write("shape_valid=NO\n")
            file.write("shape_type=UNKNOWN\n")
            file.write("shape_x_cm=NONE\n")
        else:
            file.write(
                "shape_valid=%s\n"
                % ("YES" if shape_result.get("shape_valid", False) else "NO")
            )
            file.write(
                "shape_reject_reason=%s\n"
                % shape_result.get("reject_reason", "UNKNOWN")
            )
            file.write(
                "shape_digit_match_mode=%s\n"
                % shape_result.get("digit_match_mode", "NONE")
            )
            file.write(
                "shape_type=%s\n"
                % shape_result.get("shape_type", "UNKNOWN")
            )
            file.write(
                "shape_x_cm=%.6f\n" % shape_result.get("x_cm", 0.0)
            )
            file.write(
                "shape_confidence=%.6f\n"
                % shape_result.get("confidence", 0.0)
            )
            file.write(
                "shape_fill_ratio=%.6f\n"
                % shape_result.get("fill_ratio", 0.0)
            )
            file.write(
                "shape_width_cm=%.6f\n"
                % shape_result.get("width_cm", 0.0)
            )
            file.write(
                "shape_height_cm=%.6f\n"
                % shape_result.get("height_cm", 0.0)
            )
            file.write(
                "shape_area_cm2=%.6f\n"
                % shape_result.get("area_cm2", 0.0)
            )
            file.write(
                "shape_radial_cv=%.6f\n"
                % shape_result.get("radial_cv", 0.0)
            )
            file.write(
                "shape_harmonic_3=%.6f\n"
                % shape_result.get("harmonic_3", 0.0)
            )
            file.write(
                "shape_harmonic_4=%.6f\n"
                % shape_result.get("harmonic_4", 0.0)
            )
            file.write(
                "shape_black_white_contrast=%.6f\n"
                % shape_result.get("contrast", 0.0)
            )
            file.write(
                "shape_valid_rays=%d\n"
                % shape_result.get("valid_rays", 0)
            )
            file.write(
                "shape_corner_refined=%s\n"
                % (
                    "YES"
                    if shape_result.get("corner_refined", False)
                    else "NO"
                )
            )
            file.write(
                "shape_corner_geometry_valid=%s\n"
                % (
                    "YES"
                    if shape_result.get("corner_geometry_valid", False)
                    else "NO"
                )
            )
            file.write(
                "shape_corner_x_cm=%.6f\n"
                % shape_result.get("corner_x_cm", 0.0)
            )
            file.write(
                "shape_corner_side_cv=%.6f\n"
                % shape_result.get("corner_side_cv", 1.0)
            )
            file.write(
                "shape_corner_max_line_rms_cm=%.6f\n"
                % shape_result.get("corner_max_line_rms_cm", -1.0)
            )
            file.write(
                "shape_corner_area_disagreement=%.6f\n"
                % shape_result.get("corner_area_disagreement", 1.0)
            )
            side_parts = []
            for side in shape_result.get("corner_side_lengths_cm", ()):
                side_parts.append("%.6f" % side)
            file.write(
                "shape_corner_side_lengths_cm=%s\n"
                % ",".join(side_parts)
            )
            file.write(
                "shape_image_corners=%s\n"
                % format_corners(shape_result.get("image_corners"))
            )
            plane_center = shape_result.get("plane_center")
            if plane_center is not None:
                file.write(
                    "shape_plane_center_cm=%.6f,%.6f\n"
                    % (plane_center[0], plane_center[1])
                )
            if shape_result.get("mode") in (
                MODE_AUTO_MIN,
                MODE_DIGIT_SELECT,
                MODE_TILT_MIN,
            ):
                file.write(
                    "advanced_square_count=%d\n"
                    % len(shape_result.get("squares", ()))
                )
                file.write(
                    "advanced_candidate_count=%d\n"
                    % shape_result.get("candidate_count", 0)
                )
                file.write(
                    "advanced_raw_candidate_count=%d\n"
                    % shape_result.get("raw_candidate_count", 0)
                )
                file.write(
                    "advanced_raw_generated_count=%d\n"
                    % shape_result.get("raw_generated_count", 0)
                )
                file.write(
                    "advanced_raw_complete_count=%d\n"
                    % shape_result.get("raw_complete_count", 0)
                )
                file.write(
                    "advanced_raw_partial_count=%d\n"
                    % shape_result.get("raw_partial_count", 0)
                )
                file.write(
                    "advanced_scored_candidate_count=%d\n"
                    % shape_result.get("scored_candidate_count", 0)
                )
                file.write(
                    "advanced_selection_candidate_count=%d\n"
                    % shape_result.get("selection_candidate_count", 0)
                )
                file.write(
                    "advanced_selection_subset_count=%d\n"
                    % shape_result.get("selection_subset_count", 0)
                )
                file.write(
                    "advanced_selection_backend=%s\n"
                    % shape_result.get("selection_backend", "UNKNOWN")
                )
                file.write(
                    "advanced_fast_path=%s\n"
                    % (
                        "YES"
                        if shape_result.get("square_detection", {}).get(
                            "fast_path", False
                        )
                        else "NO"
                    )
                )
                file.write(
                    "advanced_localization_only=%s\n"
                    % (
                        "YES"
                        if shape_result.get("localization_only", False)
                        else "NO"
                    )
                )
                file.write(
                    "advanced_score_sample_count=%d\n"
                    % shape_result.get("score_sample_count", 0)
                )
                file.write(
                    "advanced_orientation_count=%d\n"
                    % shape_result.get("orientation_count", 0)
                )
                file.write(
                    "advanced_explanation_score=%.6f\n"
                    % shape_result.get("explanation_score", 0.0)
                )
                file.write(
                    "advanced_coverage_recall=%.6f\n"
                    % shape_result.get("coverage_recall", 0.0)
                )
                file.write(
                    "advanced_coverage_precision=%.6f\n"
                    % shape_result.get("coverage_precision", 0.0)
                )
                file.write(
                    "advanced_mask_threshold=%d\n"
                    % shape_result.get("mask_threshold", -1)
                )
                file.write(
                    "advanced_selected_index=%s\n"
                    % str(shape_result.get("selected_index"))
                )
                file.write(
                    "advanced_selected_digit=%s\n"
                    % str(shape_result.get("selected_digit"))
                )
                file.write(
                    "advanced_plane_mask_path=%s\n"
                    % str(shape_result.get("plane_mask_path"))
                )
                advanced_timing = shape_result.get("timing_ms", {})
                for stage_name in (
                    "boundary",
                    "orientation",
                    "candidate",
                    "selection",
                    "total",
                ):
                    file.write(
                        "advanced_timing_%s_ms=%d\n"
                        % (
                            stage_name,
                            advanced_timing.get(stage_name, 0),
                        )
                    )
                squares = shape_result.get("squares", ())
                for square_index in range(len(squares)):
                    square = squares[square_index]
                    center = square.get("center", (0.0, 0.0))
                    file.write(
                        "advanced_square_%d=side:%.6f,center:%.6f/%.6f,angle_rad:%.6f,score:%.6f,edge:%.6f,digit:%s,digit_conf:%.6f,digit_margin:%.6f,target_score:%.6f,target_gap:%.6f,digit_rotation:%s,digit_valid:%s,digit_reject:%s\n"
                        % (
                            square_index,
                            square.get("side_cm", 0.0),
                            center[0],
                            center[1],
                            square.get("angle_rad", 0.0),
                            square.get("score", 0.0),
                            square.get("edge_support", 0.0),
                            str(square.get("digit")),
                            square.get("digit_confidence", 0.0),
                            square.get("digit_margin", 0.0),
                            square.get("digit_target_score", 0.0),
                            square.get("digit_target_gap", 0.0),
                            str(square.get("digit_rotation_deg")),
                            str(square.get("digit_result", {}).get("valid")),
                            square.get("digit_result", {}).get(
                                "reject_reason", "NOT_RUN"
                            ),
                        )
                    )


def save_failed_preview(
    preview,
    current_id,
    coarse_diagnostics=None,
    tracking_roi=None,
):
    path = "%s/measure_%03d_failed.jpg" % (SAVE_DIR, current_id)
    if tracking_roi is not None:
        preview.draw_rectangle(
            tracking_roi[0],
            tracking_roi[1],
            tracking_roi[2],
            tracking_roi[3],
            color=(0, 220, 255),
            thickness=1,
        )
    if coarse_diagnostics is not None:
        for diagnostic in coarse_diagnostics:
            # Red is a hard-rejected seed, orange is a recoverable soft-ring
            # seed, and green passed every coarse gate.  Blue remains the
            # official-ratio predicted outer edge.
            hard_reason = diagnostic.get("hard_reject_reason")
            if hard_reason is None:
                hard_reason = diagnostic.get("reject_reason", "NONE")
            if hard_reason != "NONE":
                inner_color = (255, 40, 40)
            elif diagnostic.get("soft_reasons", ()):
                inner_color = (255, 120, 0)
            else:
                inner_color = (0, 255, 80)
            draw_quad(
                preview,
                diagnostic.get("inner_corners"),
                inner_color,
                1,
            )
            draw_quad(
                preview,
                diagnostic.get("outer_corners"),
                (0, 120, 255),
                1,
            )
    preview.save(path, quality=PREVIEW_JPG_QUALITY)
    print("Failed preview saved:", path)
    return path


def save_failed_high_diagnostic(image, high_results, current_id):
    """Save one rejected 1080p frame with every refined hypothesis visible."""
    path = "%s/measure_%03d_failed_1080.jpg" % (SAVE_DIR, current_id)
    if isinstance(high_results, dict):
        high_results = (high_results,)
    if high_results is None:
        high_results = ()
    colors = (
        ((0, 255, 80), (255, 50, 50), (0, 120, 255)),
        ((255, 220, 0), (255, 0, 220), (0, 220, 255)),
        ((150, 255, 0), (255, 120, 0), (120, 120, 255)),
    )
    for index in range(len(high_results)):
        high_result = high_results[index]
        inner_color, detected_color, model_color = colors[
            index % len(colors)
        ]
        draw_quad(
            image,
            high_result.get("inner_corners"),
            inner_color,
            2,
        )
        draw_quad(
            image,
            high_result.get("detected_outer_corners"),
            detected_color,
            2,
        )
        draw_quad(
            image,
            high_result.get("model_outer_corners"),
            model_color,
            1,
        )
    image.draw_rectangle(4, 4, 12, 12, color=(255, 50, 50), thickness=-1)
    image.save(path, quality=SAVED_JPG_QUALITY)
    print("Failed 1080p diagnostic saved:", path)
    return path


def remove_file_if_exists(path):
    if path is None:
        return
    try:
        os.remove(path)
    except OSError:
        pass


def capture_measurement_attempt(
    first_preview,
    detector,
    refiner,
    aperture_locator,
    attempt_index,
    tracking_hint=None,
    allow_localization_only=False,
    relaxed_geometry=False,
):
    attempt_start = time.ticks_ms()
    tracking_roi = tracking_roi_from_result(tracking_hint)
    result = {
        "attempt_index": attempt_index,
        "relaxed_geometry": bool(relaxed_geometry),
        "preview": first_preview,
        "coarse_result": None,
        "coarse_candidate_count": 0,
        "coarse_reject_counts": {},
        "coarse_soft_counts": {},
        "coarse_scans": [],
        "coarse_diagnostics": [],
        "aperture_scans": [],
        "aperture_diagnostics": [],
        "center_search_roi": center_search_roi(
            first_preview.width(), first_preview.height()
        ),
        "tracking_roi": tracking_roi,
        "high_image": None,
        "high_result": None,
        "high_diagnostics": [],
        "distance_result": None,
        "preview_result": None,
        "coarse_ms": 0,
        "aperture_ms": 0,
        "convert_ms": 0,
        "refine_ms": 0,
        "attempt_ms": 0,
        "error": "",
    }
    print(
        "ATTEMPT_START index=%d tracking_roi=%s"
        % (attempt_index, format_roi(tracking_roi))
    )
    service_power_telemetry()

    best_result = None
    coarse_results = []
    preview = first_preview
    tracking_seed = tracking_seed_from_result(tracking_hint)
    if tracking_seed is not None:
        # The former preview-ROI find_rects() path returned raw_rects=0 in all
        # observed 160 cm tracking scans. A strict result from the first STRONG
        # frame of this same trigger is a much stronger seed: refine it directly
        # on the next independent 1080p frame. A rejection drops the hint and
        # the following attempt returns to the central cold-start search.
        coarse_results.append(tracking_seed)
        print(
            "coarse direct_track seed=TRACK_SEED roi=%s"
            % format_roi(tracking_roi)
        )

    if tracking_seed is None:
        # Cold start searches one full-height central strip on the trigger
        # frame. The horizontal placement guide is a hard competition
        # constraint; no off-centre full-preview scan is allowed.
        search_roi = center_search_roi(preview.width(), preview.height())
        result["center_search_roi"] = search_roi
        center_results = run_coarse_scan(
            preview,
            detector,
            result,
            1,
            "CENTER",
            search_roi,
            threshold_override=None,
            scale_roi_threshold=False,
            allow_location_seed=True,
        )
        center_results, physical_rejects = filter_center_hypotheses(
            center_results,
            preview,
            allow_projective=allow_localization_only,
        )
        if physical_rejects:
            result["coarse_reject_counts"]["PHYSICAL_RANGE"] = (
                result["coarse_reject_counts"].get("PHYSICAL_RANGE", 0)
                + physical_rejects
            )
        for candidate in center_results:
            coarse_results.append(candidate)

        if not has_trusted_coarse_seed(center_results):
            # At 175--200 cm the frame is only about 30x50 preview pixels.
            # Retry the same bounded strip at 1200. No extra frame is captured.
            center_low_results = run_coarse_scan(
                preview,
                detector,
                result,
                2,
                "CENTER_LOW",
                search_roi,
                threshold_override=FALLBACK_RECT_THRESHOLD,
                scale_roi_threshold=False,
                allow_location_seed=True,
            )
            center_low_results, physical_rejects = filter_center_hypotheses(
                center_low_results,
                preview,
                allow_projective=allow_localization_only,
            )
            if physical_rejects:
                result["coarse_reject_counts"]["PHYSICAL_RANGE"] = (
                    result["coarse_reject_counts"].get(
                        "PHYSICAL_RANGE", 0
                    )
                    + physical_rejects
                )
            for candidate in center_low_results:
                coarse_results.append(candidate)

            combined_rect_results = list(center_results)
            for candidate in center_low_results:
                combined_rect_results.append(candidate)
            if not has_trusted_coarse_seed(combined_rect_results):
                # find_rects() is shape-biased at long range. Recover the same
                # physical target from its connected bright inner aperture and
                # four dark border sides, still inside the central strip.
                aperture_results = run_aperture_scan(
                    preview,
                    aperture_locator,
                    result,
                    "CENTER_APERTURE",
                    search_roi,
                    "PREVIEW_INNER_APERTURE",
                    allow_projective=allow_localization_only,
                )
                for candidate in aperture_results:
                    coarse_results.append(candidate)
            else:
                print(
                    "coarse center_low trusted=%d aperture=SKIPPED"
                    % (1 if has_trusted_coarse_seed(combined_rect_results) else 0)
                )
        else:
            print(
                "coarse center_hit hypotheses=%d center_low=SKIPPED aperture=SKIPPED"
                % len(center_results)
            )

    coarse_results = detector.select_distinct_hypotheses(
        coarse_results,
        max_results=MAX_HIGH_RES_HYPOTHESES,
    )
    for candidate in coarse_results:
        if best_result is None or candidate["score"] > best_result["score"]:
            best_result = candidate
    result["preview"] = preview
    result["coarse_candidate_count"] = len(coarse_results)
    if coarse_results:
        print(
            "coarse distinct=%d refine_limit=%d"
            % (len(coarse_results), MAX_HIGH_RES_HYPOTHESES)
        )
    need_high_aperture = (
        tracking_seed is None
        and not has_trusted_coarse_seed(coarse_results)
    )

    service_power_telemetry()
    gc.collect()
    high_capture = None
    for _ in range(HIGH_RES_FLUSH_FRAMES):
        high_capture = sensor.snapshot(chn=CAPTURE_CH)

    # On this Yahboom CanMV firmware RGB888 can be captured and converted, but
    # get_pixel() on that buffer returns None.  The refiner performs sparse
    # pixel reads, so convert once *before* refinement.  RGB565 still retains
    # much more spatial detail than the 640x360 coarse image and can be saved
    # directly as JPG afterwards.
    convert_start = time.ticks_ms()
    high_image = high_capture.to_rgb565()
    high_capture = None
    result["convert_ms"] = time.ticks_diff(
        time.ticks_ms(), convert_start
    )
    service_power_telemetry()
    gc.collect()
    if high_image is None:
        result["error"] = "CONVERT_NONE"
        result["attempt_ms"] = time.ticks_diff(
            time.ticks_ms(), attempt_start
        )
        print("ATTEMPT_REJECT index=%d reason=CONVERT_NONE" % attempt_index)
        return result
    result["high_image"] = high_image

    print(
        "high image=%dx%d format=%s convert=%dms"
        % (
            high_image.width(),
            high_image.height(),
            str(high_image.format()),
            result["convert_ms"],
        )
    )

    if need_high_aperture:
        # A preview ROI_SEED can still mix a triangle edge with the A4 frame.
        # Reacquire the connected inner aperture on the same 1080p frame before
        # trusting any large local seed shift. The result is converted back to
        # preview coordinates because HighResRefiner owns the only final gate.
        high_roi = center_search_roi(high_image.width(), high_image.height())
        high_aperture_results = run_aperture_scan(
            high_image,
            aperture_locator,
            result,
            "HIGH_RES_CENTER_APERTURE",
            high_roi,
            "HIGH_RES_INNER_APERTURE",
            allow_projective=allow_localization_only,
        )
        scale_x = PREVIEW_WIDTH / high_image.width()
        scale_y = PREVIEW_HEIGHT / high_image.height()
        for candidate in high_aperture_results:
            scaled_candidate = scale_seed(
                candidate,
                scale_x,
                scale_y,
                source="HIGH_RES_INNER_APERTURE",
            )
            # Prefer the higher-resolution location when it duplicates the
            # preview aperture. It is still only an ROI_SEED and must pass the
            # unchanged strict 1080p quality gates below.
            scaled_candidate["score"] += 0.08
            coarse_results.append(scaled_candidate)

        coarse_results = detector.select_distinct_hypotheses(
            coarse_results,
            max_results=MAX_HIGH_RES_HYPOTHESES,
        )
        best_result = None
        for candidate in coarse_results:
            if (
                best_result is None
                or candidate["score"] > best_result["score"]
            ):
                best_result = candidate
        result["coarse_candidate_count"] = len(coarse_results)

    if best_result is None:
        result["error"] = "CENTER_TARGET_NOT_FOUND"
        result["attempt_ms"] = time.ticks_diff(
            time.ticks_ms(), attempt_start
        )
        print(
            "ATTEMPT_REJECT index=%d reason=CENTER_TARGET_NOT_FOUND reject=%s"
            % (
                attempt_index,
                format_rejection_counts(result["coarse_reject_counts"]),
            )
        )
        return result
    result["coarse_result"] = best_result

    # Coarse confidence mainly describes rectangle plausibility; it does not
    # guarantee that all four low-resolution corners are equally accurate.
    # Refine every successful coarse hypothesis against the same 1080p frame
    # and let high-resolution edge evidence select the final result.
    high_result = None
    selected_coarse_result = best_result
    for candidate_index in range(len(coarse_results)):
        candidate = coarse_results[candidate_index]
        refine_start = time.ticks_ms()
        candidate_high = refiner.refine(
            high_image,
            candidate,
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            preserve_projective_seed=allow_localization_only,
            localization_only=allow_localization_only,
            max_inner_scale_anisotropy=(
                ADVANCED_MAX_INNER_SCALE_ANISOTROPY
                if relaxed_geometry and not allow_localization_only
                else None
            ),
        )
        candidate_refine_ms = time.ticks_diff(
            time.ticks_ms(), refine_start
        )
        result["refine_ms"] += candidate_refine_ms
        service_power_telemetry()
        if candidate_high is None:
            print(
                "refine candidate=%d result=NONE time=%dms"
                % (candidate_index + 1, candidate_refine_ms)
            )
            continue

        # Keep every refined hypothesis (small dictionaries only) so a failed
        # 1080p diagnostic shows competing locations instead of just the last
        # selected wrong rectangle.
        result["high_diagnostics"].append(candidate_high)

        if candidate_high.get("seed_regularized", False):
            print(
                "relocalize candidate=%d seed=%s roi=%s angle=%.2fdeg shift=%.1fpx force_threshold=%.1fpx large_shift=%d inner_wide=%d selected=%s"
                % (
                    candidate_index + 1,
                    candidate_high.get("coarse_seed_class", "UNKNOWN"),
                    format_roi(candidate_high.get("relocalize_roi")),
                    candidate_high.get(
                        "seed_regularization_angle_degrees", 0.0
                    ),
                    candidate_high.get(
                        "seed_regularization_max_shift", 0.0
                    ),
                    candidate_high.get("large_seed_shift_threshold", 0.0),
                    1
                    if candidate_high.get("large_seed_shift", False)
                    else 0,
                    1
                    if candidate_high.get("inner_wide_search_used", False)
                    else 0,
                    candidate_high.get("inner_refine_path", "UNKNOWN"),
                )
            )

        print(
            "refine candidate=%d coarse=%.3f high=%s valid=%d confidence=%s reject=%s quality=%.1f sides=%d/%d raw=%d/%d inner_only=%d demoted=%d inner_path=%s inner_wide=%d inner_reason=%s branch_rms=%.2f/%.2f wide_outer=%d rms=%.2f/%.2f ring=%.2f/%.2f/%.2f radius=%d/%d error=%.3f/%.3f time=%dms"
            % (
                candidate_index + 1,
                candidate["score"],
                candidate_high["mode"],
                1 if candidate_high["measurement_valid"] else 0,
                candidate_high.get("measurement_confidence", "UNKNOWN"),
                candidate_high.get("measurement_reject_reason", "UNKNOWN"),
                candidate_high["quality_score"],
                candidate_high["inner_valid_sides"],
                candidate_high["outer_valid_sides"],
                candidate_high.get("inner_raw_valid_sides", 0),
                candidate_high.get("outer_raw_valid_sides", 0),
                1 if candidate_high.get("inner_only_accepted", False) else 0,
                1
                if candidate_high.get("outer_conflict_demoted", False)
                else 0,
                candidate_high.get("inner_refine_path", "UNKNOWN"),
                1
                if candidate_high.get("inner_wide_search_used", False)
                else 0,
                candidate_high.get(
                    "inner_wide_search_reason", "UNKNOWN"
                ),
                candidate_high.get("inner_narrow_edge_rms", -1.0),
                candidate_high.get("inner_wide_edge_rms", -1.0),
                1
                if candidate_high.get("outer_wide_search_used", False)
                else 0,
                candidate_high.get("inner_edge_rms", -1.0),
                candidate_high.get("outer_edge_rms", -1.0),
                candidate_high.get("ring_inside_pass_ratio", 0.0),
                candidate_high.get("ring_min_side_pass_ratio", 0.0),
                candidate_high.get("ring_outside_pass_ratio", 0.0),
                candidate_high["inner_search_radius"],
                candidate_high["outer_search_radius"],
                candidate_high.get("detected_frame_disagreement", 0.0),
                candidate_high["frame_model_disagreement"],
                candidate_refine_ms,
            )
        )
        candidate_rank = candidate_high.get(
            "measurement_confidence_rank",
            2 if candidate_high["measurement_valid"] else 0,
        )
        selected_rank = (
            high_result.get(
                "measurement_confidence_rank",
                2 if high_result["measurement_valid"] else 0,
            )
            if high_result is not None
            else -1
        )
        if (
            high_result is None
            or candidate_rank > selected_rank
            or (
                candidate_rank == selected_rank
                and candidate_high["quality_score"]
                > high_result["quality_score"]
            )
        ):
            high_result = candidate_high
            selected_coarse_result = candidate
        if candidate_high.get("measurement_valid", False) or (
            allow_localization_only
            and tilt_localization_result(candidate_high) is not None
        ):
            # Candidates are already ordered by coarse plausibility.  Once a
            # STRONG 1080p result passes every gate, refining lower-ranked
            # hypotheses cannot improve the two-frame acceptance decision.
            print(
                "refine early_stop candidate=%d reason=%s"
                % (
                    candidate_index + 1,
                    (
                        "STRONG"
                        if candidate_high.get("measurement_valid", False)
                        else "TILT_LOCALIZATION"
                    ),
                )
            )
            break

    best_result = selected_coarse_result
    result["coarse_result"] = best_result
    if high_result is None:
        result["error"] = "REFINE_NONE"
        result["attempt_ms"] = time.ticks_diff(
            time.ticks_ms(), attempt_start
        )
        print("ATTEMPT_REJECT index=%d reason=REFINE_NONE" % attempt_index)
        return result

    distance_result = estimate_distance(high_result)
    if distance_result is None and allow_localization_only:
        distance_result = tilt_localization_result(high_result)
    result["high_result"] = high_result
    result["distance_result"] = distance_result
    result["preview_result"] = (
        high_result_to_preview(high_result)
        if distance_result is not None
        else None
    )
    print(
        "high mode=%s valid=%d confidence=%s reject=%s quality=%.1f inner_sides=%d outer_sides=%d raw=%d/%d inner_only=%d demoted=%d inner_path=%s inner_wide=%d inner_reason=%s branch_rms=%.2f/%.2f wide_outer=%d inner_edge=%.1f outer_edge=%.1f rms=%.2f/%.2f ring=%.2f/%.2f/%.2f radius=%d/%d error=%.3f/%.3f convert=%dms refine_total=%dms"
        % (
            high_result["mode"],
            1 if high_result["measurement_valid"] else 0,
            high_result.get("measurement_confidence", "UNKNOWN"),
            high_result.get("measurement_reject_reason", "UNKNOWN"),
            high_result["quality_score"],
            high_result["inner_valid_sides"],
            high_result["outer_valid_sides"],
            high_result.get("inner_raw_valid_sides", 0),
            high_result.get("outer_raw_valid_sides", 0),
            1 if high_result.get("inner_only_accepted", False) else 0,
            1 if high_result.get("outer_conflict_demoted", False) else 0,
            high_result.get("inner_refine_path", "UNKNOWN"),
            1 if high_result.get("inner_wide_search_used", False) else 0,
            high_result.get("inner_wide_search_reason", "UNKNOWN"),
            high_result.get("inner_narrow_edge_rms", -1.0),
            high_result.get("inner_wide_edge_rms", -1.0),
            1
            if high_result.get("outer_wide_search_used", False)
            else 0,
            high_result["inner_edge_response"],
            high_result["outer_edge_response"],
            high_result.get("inner_edge_rms", -1.0),
            high_result.get("outer_edge_rms", -1.0),
            high_result.get("ring_inside_pass_ratio", 0.0),
            high_result.get("ring_min_side_pass_ratio", 0.0),
            high_result.get("ring_outside_pass_ratio", 0.0),
            high_result["inner_search_radius"],
            high_result["outer_search_radius"],
            high_result.get("detected_frame_disagreement", 0.0),
            high_result["frame_model_disagreement"],
            result["convert_ms"],
            result["refine_ms"],
        )
    )
    if distance_result is None:
        print(
            "ATTEMPT_REJECT index=%d reason=%s"
            % (
                attempt_index,
                high_result.get("measurement_reject_reason", "UNKNOWN"),
            )
        )
    elif distance_result.get("tilt_localization_only", False):
        print(
            "ATTEMPT_TILT_STRONG index=%d frame_scale=%.6f anisotropy=%.3f original_reject=%s"
            % (
                attempt_index,
                distance_result["frame_scale"],
                distance_result["anisotropy"],
                high_result.get("measurement_reject_reason", "UNKNOWN"),
            )
        )
    else:
        print(
            "ATTEMPT_STRONG index=%d frame_scale=%.6f distance=%.2fcm anisotropy=%.3f disagreement=%.3f"
            % (
                attempt_index,
                distance_result["frame_scale"],
                distance_result["distance_cm"],
                distance_result["anisotropy"],
                distance_result["outer_inner_disagreement"],
            )
        )
    result["attempt_ms"] = time.ticks_diff(
        time.ticks_ms(), attempt_start
    )
    service_power_telemetry()
    return result


def derive_failure_reason(summaries, strong_count):
    if strong_count > 0:
        return "NO_CONSISTENT_PAIR"
    if summaries:
        coarse_only = True
        for summary in summaries:
            if summary.get("reject") != "COARSE_NONE":
                coarse_only = False
                break
        if coarse_only:
            return "COARSE_LOCALIZATION_FAILED"
        for summary in summaries:
            if summary.get("status") == "SHAPE_REJECT":
                return "NO_VALID_SHAPE"
    return "NO_STRONG_FRAME"


def derive_detailed_failure_reason(summaries, strong_count):
    """Prefer a screen-actionable reason over the generic metadata class."""
    for summary in reversed(summaries):
        reason = str(summary.get("reject", ""))
        shape_reason = str(summary.get("shape_reject", ""))
        combined = reason + ":" + shape_reason
        if "TARGET_DIGIT_NOT_FOUND" in combined:
            return "TARGET_DIGIT_NOT_FOUND"
        if "CENTER_TARGET_NOT_FOUND" in combined or "COARSE" in combined:
            return "TARGET_NOT_FOUND"
        if "SHAPE" in combined or "SQUARE_" in combined:
            return "SHAPE_RECOGNITION_FAILED"
    return derive_failure_reason(summaries, strong_count)


def screen_failure_status(reason, task_mode, measurement_error=False):
    if measurement_error:
        return TJCScreen.STATUS_SYSTEM_ERROR
    reason = str(reason or "")
    if task_mode == MODE_DIGIT_SELECT and "DIGIT" in reason:
        return TJCScreen.STATUS_NUMBER_NOT_FOUND
    if "TARGET_NOT_FOUND" in reason or "COARSE" in reason:
        return TJCScreen.STATUS_NO_TARGET
    if "SHAPE" in reason or "SQUARE" in reason:
        return TJCScreen.STATUS_SHAPE_FAILED
    return TJCScreen.STATUS_FAILED


def save_failed_metadata(
    path,
    summaries,
    total_ms,
    strong_count=0,
    task_mode=None,
    target_digit=None,
):
    with open(path, "w") as file:
        file.write("pipeline_version=%s\n" % PIPELINE_VERSION)
        file.write("task_mode=%s\n" % str(task_mode))
        file.write("target_digit=%s\n" % str(target_digit))
        file.write("measurement_valid=NO\n")
        file.write(
            "failure_reason=%s\n"
            % derive_failure_reason(summaries, strong_count)
        )
        file.write("measurement_attempts=%d\n" % len(summaries))
        file.write("strong_count=%d\n" % strong_count)
        file.write("total_ms=%d\n" % total_ms)
        file.write(
            "max_scale_disagreement=%.6f\n"
            % MAX_FRAME_SCALE_DISAGREEMENT
        )
        file.write(
            "max_center_shift_ratio=%.6f\n"
            % MAX_FRAME_CENTER_SHIFT_RATIO
        )
        for summary in summaries:
            index = summary["attempt_index"]
            file.write(
                "attempt_%d_status=%s\n" % (index, summary["status"])
            )
            file.write(
                "attempt_%d_geometry_profile=%s\n"
                % (
                    index,
                    (
                        "ADVANCED_RELAXED"
                        if summary.get("relaxed_geometry", False)
                        else "BASIC_STRICT"
                    ),
                )
            )
            file.write(
                "attempt_%d_reject=%s\n" % (index, summary["reject"])
            )
            digit_diagnostics = summary.get("digit_diagnostics", ())
            file.write(
                "attempt_%d_digit_candidates=%d\n"
                % (index, len(digit_diagnostics))
            )
            for diagnostic_index in range(len(digit_diagnostics)):
                diagnostic = digit_diagnostics[diagnostic_index]
                prefix = "attempt_%d_digit_%d_" % (
                    index,
                    diagnostic_index,
                )
                file.write(
                    prefix
                    + "side_cm=%.6f\n"
                    % diagnostic.get("side_cm", 0.0)
                )
                file.write(
                    prefix
                    + "center_cm=%.6f,%.6f\n"
                    % (
                        diagnostic.get("center_x_cm", 0.0),
                        diagnostic.get("center_y_cm", 0.0),
                    )
                )
                file.write(
                    prefix
                    + "square_score=%.6f\n"
                    % diagnostic.get("square_score", 0.0)
                )
                file.write(
                    prefix + "best=%s\n" % str(diagnostic.get("digit"))
                )
                file.write(
                    prefix
                    + "valid=%s\n"
                    % ("YES" if diagnostic.get("valid", False) else "NO")
                )
                file.write(
                    prefix
                    + "confidence=%.6f\n"
                    % diagnostic.get("confidence", 0.0)
                )
                file.write(
                    prefix
                    + "margin=%.6f\n"
                    % diagnostic.get("margin", 0.0)
                )
                file.write(
                    prefix
                    + "target_score=%.6f\n"
                    % diagnostic.get("target_score", 0.0)
                )
                file.write(
                    prefix
                    + "target_gap=%.6f\n"
                    % diagnostic.get("target_gap", 0.0)
                )
                file.write(
                    prefix
                    + "rotation_deg=%s\n"
                    % str(diagnostic.get("rotation_deg"))
                )
                file.write(
                    prefix
                    + "reject=%s\n"
                    % diagnostic.get("reject_reason", "UNKNOWN")
                )
            file.write(
                "attempt_%d_coarse_candidates=%d\n"
                % (index, summary.get("coarse_candidate_count", 0))
            )
            file.write(
                "attempt_%d_tracking_roi=%s\n"
                % (index, format_roi(summary.get("tracking_roi")))
            )
            reject_counts = summary.get("coarse_reject_counts", {})
            file.write(
                "attempt_%d_coarse_rejects=%s\n"
                % (index, format_rejection_counts(reject_counts))
            )
            for reason in sorted(reject_counts):
                file.write(
                    "attempt_%d_coarse_reject_%s=%d\n"
                    % (index, reason, reject_counts[reason])
                )
            soft_counts = summary.get("coarse_soft_counts", {})
            file.write(
                "attempt_%d_coarse_soft=%s\n"
                % (index, format_rejection_counts(soft_counts))
            )
            for reason in sorted(soft_counts):
                file.write(
                    "attempt_%d_coarse_soft_%s=%d\n"
                    % (index, reason, soft_counts[reason])
                )
            scans = summary.get("coarse_scans", ())
            file.write(
                "attempt_%d_coarse_scans=%d\n" % (index, len(scans))
            )
            for scan_index in range(len(scans)):
                scan = scans[scan_index]
                prefix = "attempt_%d_scan_%d_" % (index, scan_index + 1)
                file.write(prefix + "scope=%s\n" % scan["scope"])
                file.write(
                    prefix + "threshold=%d\n" % scan["threshold"]
                )
                file.write(
                    prefix + "raw_rects=%d\n" % scan["raw_rects"]
                )
                file.write(
                    prefix + "candidates=%d\n" % scan["candidates"]
                )
                file.write(prefix + "pairs=%d\n" % scan["pairs"])
                file.write(prefix + "errors=%d\n" % scan["errors"])
                file.write(
                    prefix + "elapsed_ms=%d\n" % scan["elapsed_ms"]
                )
                file.write(
                    prefix
                    + "rejects=%s\n"
                    % format_rejection_counts(scan["reject_counts"])
                )
                file.write(
                    prefix
                    + "soft=%s\n"
                    % format_rejection_counts(scan.get("soft_counts", {}))
                )
                diagnostics = scan.get("diagnostics", ())
                file.write(
                    prefix + "diagnostics=%d\n" % len(diagnostics)
                )
                for diagnostic_index in range(len(diagnostics)):
                    diagnostic = diagnostics[diagnostic_index]
                    candidate_prefix = (
                        prefix + "candidate_%d_" % (diagnostic_index + 1)
                    )
                    file.write(
                        candidate_prefix
                        + "hard_reject=%s\n"
                        % diagnostic.get("hard_reject_reason", "NONE")
                    )
                    file.write(
                        candidate_prefix
                        + "soft=%s\n"
                        % format_reason_list(
                            diagnostic.get("soft_reasons", ())
                        )
                    )
                    file.write(
                        candidate_prefix
                        + "seed_class=%s\n"
                        % diagnostic.get("seed_class", "REJECT")
                    )
                    file.write(
                        candidate_prefix
                        + "location_seed_relaxed=%s\n"
                        % (
                            "YES"
                            if diagnostic.get(
                                "location_seed_relaxed", False
                            )
                            else "NO"
                        )
                    )
                    file.write(
                        candidate_prefix
                        + "inner_corners=%s\n"
                        % format_corners(diagnostic.get("inner_corners"))
                    )
                    file.write(
                        candidate_prefix
                        + "predicted_outer_corners=%s\n"
                        % format_corners(diagnostic.get("outer_corners"))
                    )
                    for key in (
                        "area_ratio",
                        "aspect",
                        "contrast",
                        "inner_score",
                        "ring_inside_contrast",
                        "ring_outside_contrast",
                        "ring_inside_pass_ratio",
                        "ring_outside_pass_ratio",
                        "ring_min_side_pass_ratio",
                        "regularity_max_opposite_error",
                        "regularity_diagonal_midpoint_error",
                        "regularity_fill_ratio",
                        "regularity_min_corner_angle",
                        "regularity_max_corner_angle",
                    ):
                        file.write(
                            candidate_prefix
                            + "%s=%.6f\n"
                            % (key, diagnostic.get(key, -1.0))
                        )
            aperture_scans = summary.get("aperture_scans", ())
            file.write(
                "attempt_%d_aperture_scans=%d\n"
                % (index, len(aperture_scans))
            )
            file.write(
                "attempt_%d_aperture_ms=%d\n"
                % (index, summary.get("aperture_ms", 0))
            )
            for scan_index in range(len(aperture_scans)):
                scan = aperture_scans[scan_index]
                prefix = "attempt_%d_aperture_scan_%d_" % (
                    index,
                    scan_index + 1,
                )
                file.write(prefix + "scope=%s\n" % scan.get("scope", "UNKNOWN"))
                file.write(prefix + "source=%s\n" % scan.get("source", "UNKNOWN"))
                file.write(
                    prefix + "roi=%s\n" % format_roi(scan.get("roi"))
                )
                file.write(
                    prefix + "blobs=%d\n" % scan.get("blobs", 0)
                )
                file.write(
                    prefix
                    + "candidates=%d\n"
                    % scan.get("candidates", 0)
                )
                file.write(
                    prefix
                    + "elapsed_ms=%d\n"
                    % scan.get("elapsed_ms", 0)
                )
                aperture_rejects = scan.get("reject_counts", {})
                file.write(
                    prefix
                    + "rejects=%s\n"
                    % format_rejection_counts(aperture_rejects)
                )
                for reason in sorted(aperture_rejects):
                    file.write(
                        prefix
                        + "reject_%s=%d\n"
                        % (reason, aperture_rejects[reason])
                    )
                file.write(
                    prefix + "error=%s\n" % (scan.get("error") or "NONE")
                )
            if summary["frame_scale"] is not None:
                file.write(
                    "attempt_%d_frame_scale=%.6f\n"
                    % (index, summary["frame_scale"])
                )
            high_result = summary.get("high_result")
            if high_result is None:
                continue
            prefix = "attempt_%d_" % index
            file.write(prefix + "high_mode=%s\n" % high_result["mode"])
            file.write(
                prefix
                + "coarse_seed_class=%s\n"
                % high_result.get("coarse_seed_class", "UNKNOWN")
            )
            file.write(
                prefix
                + "seed_regularized=%s\n"
                % (
                    "YES"
                    if high_result.get("seed_regularized", False)
                    else "NO"
                )
            )
            file.write(
                prefix
                + "relocalize_roi=%s\n"
                % format_roi(high_result.get("relocalize_roi"))
            )
            file.write(
                prefix
                + "seed_regularization_angle_degrees=%.6f\n"
                % high_result.get(
                    "seed_regularization_angle_degrees", 0.0
                )
            )
            file.write(
                prefix
                + "seed_regularization_max_shift=%.6f\n"
                % high_result.get("seed_regularization_max_shift", 0.0)
            )
            file.write(
                prefix
                + "large_seed_shift_threshold=%.6f\n"
                % high_result.get("large_seed_shift_threshold", 0.0)
            )
            file.write(
                prefix
                + "large_seed_shift=%s\n"
                % (
                    "YES"
                    if high_result.get("large_seed_shift", False)
                    else "NO"
                )
            )
            file.write(
                prefix
                + "inner_refine_path=%s\n"
                % high_result.get("inner_refine_path", "UNKNOWN")
            )
            file.write(
                prefix
                + "inner_wide_search_used=%s\n"
                % (
                    "YES"
                    if high_result.get("inner_wide_search_used", False)
                    else "NO"
                )
            )
            file.write(
                prefix
                + "inner_wide_search_reason=%s\n"
                % high_result.get(
                    "inner_wide_search_reason", "UNKNOWN"
                )
            )
            file.write(
                prefix
                + "inner_search_radius=%d\n"
                % high_result.get("inner_search_radius", 0)
            )
            file.write(
                prefix
                + "inner_edge_rms=%.6f\n"
                % high_result.get("inner_edge_rms", -1.0)
            )
            file.write(
                prefix
                + "inner_raw_edge_rms=%.6f\n"
                % high_result.get("inner_raw_edge_rms", -1.0)
            )
            file.write(
                prefix
                + "inner_narrow_edge_rms=%.6f\n"
                % high_result.get("inner_narrow_edge_rms", -1.0)
            )
            file.write(
                prefix
                + "inner_wide_edge_rms=%.6f\n"
                % high_result.get("inner_wide_edge_rms", -1.0)
            )
            file.write(
                prefix
                + "high_quality_score=%.3f\n"
                % high_result.get("quality_score", 0.0)
            )
            file.write(
                prefix
                + "detected_frame_disagreement=%.6f\n"
                % high_result.get("detected_frame_disagreement", 0.0)
            )
            file.write(
                prefix
                + "inner_fill_ratio=%.6f\n"
                % high_result.get("inner_fill_ratio", 0.0)
            )
            file.write(
                prefix
                + "inner_max_opposite_error=%.6f\n"
                % high_result.get("inner_max_opposite_error", 1.0)
            )
            file.write(
                prefix
                + "inner_scale_anisotropy=%.6f\n"
                % high_result.get("inner_scale_anisotropy", 1.0)
            )
            file.write(
                prefix
                + "inner_valid_sides=%d\n"
                % high_result.get("inner_valid_sides", 0)
            )
            file.write(
                prefix
                + "outer_valid_sides=%d\n"
                % high_result.get("outer_valid_sides", 0)
            )
            file.write(
                prefix
                + "detected_outer_edge_rms=%.6f\n"
                % high_result.get("detected_outer_edge_rms", -1.0)
            )
            file.write(
                prefix
                + "ring_inside_contrast=%.6f\n"
                % high_result.get("ring_inside_contrast", -999.0)
            )
            file.write(
                prefix
                + "ring_inside_pass_ratio=%.6f\n"
                % high_result.get("ring_inside_pass_ratio", 0.0)
            )
            file.write(
                prefix
                + "ring_min_side_pass_ratio=%.6f\n"
                % high_result.get("ring_min_side_pass_ratio", 0.0)
            )
            file.write(
                prefix
                + "ring_outside_contrast=%.6f\n"
                % high_result.get("ring_outside_contrast", -999.0)
            )
            file.write(
                prefix
                + "ring_outside_pass_ratio=%.6f\n"
                % high_result.get("ring_outside_pass_ratio", 0.0)
            )
            file.write(
                prefix
                + "outer_conflict_demoted=%s\n"
                % (
                    "YES"
                    if high_result.get("outer_conflict_demoted", False)
                    else "NO"
                )
            )
            file.write(
                prefix
                + "outer_conflict_inner_ring_valid=%s\n"
                % (
                    "YES"
                    if high_result.get(
                        "outer_conflict_inner_ring_valid", False
                    )
                    else "NO"
                )
            )
            file.write(
                prefix
                + "inner_corners=%s\n"
                % format_corners(high_result.get("inner_corners"))
            )
            file.write(
                prefix
                + "detected_outer_corners=%s\n"
                % format_corners(high_result.get("detected_outer_corners"))
            )
            file.write(
                prefix
                + "model_outer_corners=%s\n"
                % format_corners(high_result.get("model_outer_corners"))
            )


def measure_once(
    first_preview,
    detector,
    refiner,
    aperture_locator,
    shape_detector,
    advanced_detector=None,
    task_mode="BASIC",
    target_digit=None,
    tracking_hint=None,
):
    global measurement_id, last_measurement_failure_reason
    measurement_id += 1
    last_measurement_failure_reason = None
    current_id = measurement_id
    total_start = time.ticks_ms()
    strong_records = []
    summaries = []
    last_preview = first_preview
    coarse_total_ms = 0
    convert_total_ms = 0
    refine_total_ms = 0
    failed_high_path = None
    last_coarse_diagnostics = []
    last_tracking_roi = tracking_roi_from_result(tracking_hint)
    current_tracking_hint = tracking_hint
    max_attempts = (
        MAX_TILT_MEASUREMENT_ATTEMPTS
        if task_mode == MODE_TILT_MIN
        else MAX_MEASUREMENT_ATTEMPTS
    )

    print("")
    print(
        "MEASURE_START id=%03d task=%s target_digit=%s pipeline=%s consistency=2 max_attempts=%d scale_threshold=%.3f%% center_threshold=%.1f%% center_roi=%s tracking_roi=%s geometry=%s"
        % (
            current_id,
            task_mode,
            str(target_digit),
            PIPELINE_VERSION,
            max_attempts,
            MAX_FRAME_SCALE_DISAGREEMENT * 100.0,
            MAX_FRAME_CENTER_SHIFT_RATIO * 100.0,
            format_roi(center_search_roi(PREVIEW_WIDTH, PREVIEW_HEIGHT)),
            format_roi(last_tracking_roi),
            (
                "ADVANCED_RELAXED"
                if task_mode in (MODE_AUTO_MIN, MODE_DIGIT_SELECT)
                else "BASIC_STRICT"
            ),
        )
    )

    for attempt_index in range(1, max_attempts + 1):
        if attempt_index == 1:
            attempt_preview = first_preview
        else:
            attempt_preview = sensor.snapshot(chn=PREVIEW_CH)
        try:
            attempt = capture_measurement_attempt(
                attempt_preview,
                detector,
                refiner,
                aperture_locator,
                attempt_index,
                tracking_hint=current_tracking_hint,
                allow_localization_only=(task_mode == MODE_TILT_MIN),
                relaxed_geometry=(
                    task_mode in (MODE_AUTO_MIN, MODE_DIGIT_SELECT)
                ),
            )
        except Exception as error:
            print(
                "ATTEMPT_ERROR index=%d error=%s"
                % (attempt_index, repr(error))
            )
            summaries.append({
                "attempt_index": attempt_index,
                "status": "EXCEPTION",
                "reject": repr(error),
                "frame_scale": None,
            })
            current_tracking_hint = None
            gc.collect()
            continue

        last_preview = attempt.get("preview", last_preview)
        current_coarse_diagnostics = attempt.get("coarse_diagnostics", ())
        if current_coarse_diagnostics:
            last_coarse_diagnostics = current_coarse_diagnostics
        last_tracking_roi = attempt.get("tracking_roi", last_tracking_roi)
        coarse_total_ms += attempt["coarse_ms"]
        convert_total_ms += attempt["convert_ms"]
        refine_total_ms += attempt["refine_ms"]
        high_result = attempt.get("high_result")
        distance_result = attempt.get("distance_result")
        reject_reason = attempt.get("error", "")
        if high_result is not None:
            reject_reason = high_result.get(
                "measurement_reject_reason", reject_reason
            )
        summaries.append({
            "attempt_index": attempt_index,
            "status": "STRONG" if distance_result is not None else "REJECT",
            "reject": reject_reason or "UNKNOWN",
            "frame_scale": (
                distance_result["frame_scale"]
                if distance_result is not None
                else None
            ),
            # This dictionary contains only geometry/quality scalars and small
            # corner tuples, never the high-resolution image buffer.
            "high_result": high_result,
            "coarse_candidate_count": attempt.get(
                "coarse_candidate_count", 0
            ),
            "coarse_reject_counts": attempt.get(
                "coarse_reject_counts", {}
            ),
            "coarse_soft_counts": attempt.get("coarse_soft_counts", {}),
            "coarse_scans": attempt.get("coarse_scans", ()),
            "aperture_scans": attempt.get("aperture_scans", ()),
            "aperture_ms": attempt.get("aperture_ms", 0),
            "tracking_roi": attempt.get("tracking_roi"),
            "relaxed_geometry": attempt.get("relaxed_geometry", False),
        })

        if (
            distance_result is None
            and high_result is not None
            and attempt.get("high_image") is not None
            and failed_high_path is None
        ):
            # Preserve the first real 1080p rejection, including all refined
            # hypotheses from that frame.  If a later attempt succeeds, this
            # temporary diagnostic is removed before returning.
            failed_high_path = save_failed_high_diagnostic(
                attempt["high_image"],
                attempt.get("high_diagnostics") or high_result,
                current_id,
            )

        if distance_result is not None:
            current_tracking_hint = attempt.get("preview_result")
            current_scale = distance_result["frame_scale"]
            shape_result = None
            shape_ms = 0

            # Advanced tasks run on every accepted localization.  Only the
            # small geometry result is retained, so two-frame target identity
            # and x can be checked without keeping two 1080p buffers alive.
            if task_mode != "BASIC":
                service_power_telemetry()
                shape_start = time.ticks_ms()
                try:
                    if advanced_detector is None:
                        raise ValueError("advanced detector is not configured")
                    mapper = PlaneMapper(
                        high_result["inner_corners_float"],
                        INNER_PLANE_WIDTH_CM,
                        INNER_PLANE_HEIGHT_CM,
                    )
                    shape_result = advanced_detector.detect(
                        attempt["high_image"],
                        mapper,
                        high_result["inner_corners_float"],
                        mode=task_mode,
                        target_digit=target_digit,
                    )
                except Exception as error:
                    shape_result = {
                        "shape_valid": False,
                        "shape_type": "UNKNOWN",
                        "x_cm": 0.0,
                        "confidence": 0.0,
                        "reject_reason": "ADVANCED_EXCEPTION:%s" % repr(error),
                        "mode": task_mode,
                    }
                shape_ms = time.ticks_diff(time.ticks_ms(), shape_start)
                shape_result["shape_ms"] = shape_ms
                service_power_telemetry()
                detector_timing = shape_result.get("timing_ms", {})
                print(
                    "advanced_detector boundary=%dms orientation=%dms candidate=%dms selection=%dms total=%dms raw=%d/%d full=%d partial=%d scored=%d final=%d subsets=%d backend=%s"
                    % (
                        detector_timing.get("boundary", 0),
                        detector_timing.get("orientation", 0),
                        detector_timing.get("candidate", 0),
                        detector_timing.get("selection", 0),
                        detector_timing.get("total", shape_ms),
                        shape_result.get("raw_candidate_count", 0),
                        shape_result.get("raw_generated_count", 0),
                        shape_result.get("raw_complete_count", 0),
                        shape_result.get("raw_partial_count", 0),
                        shape_result.get("scored_candidate_count", 0),
                        len(shape_result.get("squares", ())),
                        shape_result.get("selection_subset_count", 0),
                        shape_result.get("selection_backend", "UNKNOWN"),
                    )
                )
                digit_diagnostics = collect_digit_diagnostics(shape_result)
                summaries[-1]["digit_diagnostics"] = digit_diagnostics
                if task_mode == MODE_DIGIT_SELECT:
                    print_digit_diagnostics(
                        attempt_index, digit_diagnostics
                    )
                summaries[-1]["shape_status"] = (
                    "VALID"
                    if shape_result.get("shape_valid", False)
                    else "REJECT"
                )
                summaries[-1]["shape_reject"] = shape_result.get(
                    "reject_reason", "UNKNOWN"
                )
                if not shape_result.get("shape_valid", False):
                    summaries[-1]["status"] = "SHAPE_REJECT"
                    summaries[-1]["reject"] = shape_result.get(
                        "reject_reason", "UNKNOWN_SHAPE_REJECT"
                    )
                    print(
                        "ADVANCED_REJECT index=%d task=%s reason=%s candidates=%d squares=%d time=%dms"
                        % (
                            attempt_index,
                            task_mode,
                            shape_result.get("reject_reason", "UNKNOWN"),
                            shape_result.get("candidate_count", 0),
                            len(shape_result.get("squares", ())),
                            shape_ms,
                        )
                    )
                    attempt["high_image"] = None
                    gc.collect()
                    continue

            matched = None
            matched_difference = None
            matched_center_shift_ratio = None
            matched_shape_gate = None
            last_shape_gate = None
            for previous in strong_records:
                difference = relative_scale_difference(
                    previous["distance_result"]["frame_scale"],
                    current_scale,
                )
                center_shift_ratio = relative_center_shift(
                    previous["high_result"]["inner_corners_float"],
                    high_result["inner_corners_float"],
                )
                if (
                    difference > MAX_FRAME_SCALE_DISAGREEMENT
                    or center_shift_ratio > MAX_FRAME_CENTER_SHIFT_RATIO
                ):
                    continue
                if task_mode != "BASIC":
                    last_shape_gate = advanced_results_consistent(
                        previous.get("shape_result"),
                        shape_result,
                    )
                    if not last_shape_gate.get("valid", False):
                        continue
                if (
                    matched_difference is None
                    or difference < matched_difference
                ):
                    matched = previous
                    matched_difference = difference
                    matched_center_shift_ratio = center_shift_ratio
                    matched_shape_gate = last_shape_gate

            if matched is not None:
                if task_mode == "BASIC":
                    service_power_telemetry()
                    shape_start = time.ticks_ms()
                    try:
                        mapper = PlaneMapper(
                            high_result["inner_corners_float"],
                            INNER_PLANE_WIDTH_CM,
                            INNER_PLANE_HEIGHT_CM,
                        )
                        shape_result = shape_detector.detect(
                            attempt["high_image"],
                            mapper,
                            high_result["inner_corners_float"],
                        )
                    except Exception as error:
                        shape_result = {
                            "shape_valid": False,
                            "shape_type": "UNKNOWN",
                            "x_cm": 0.0,
                            "confidence": 0.0,
                            "reject_reason": "SHAPE_EXCEPTION:%s" % repr(error),
                        }
                    shape_ms = time.ticks_diff(time.ticks_ms(), shape_start)
                    shape_result["shape_ms"] = shape_ms
                    service_power_telemetry()

                if not shape_result.get("shape_valid", False):
                    summaries[-1]["status"] = "SHAPE_REJECT"
                    summaries[-1]["reject"] = shape_result.get(
                        "reject_reason", "UNKNOWN_SHAPE_REJECT"
                    )
                    strong_records.append({
                        "attempt_index": attempt_index,
                        "distance_result": distance_result,
                        "high_result": high_result,
                        "shape_result": shape_result,
                    })
                    print(
                        "SHAPE_REJECT index=%d reason=%s type=%s x=%.2fcm confidence=%.3f time=%dms"
                        % (
                            attempt_index,
                            shape_result.get("reject_reason", "UNKNOWN"),
                            shape_result.get("shape_type", "UNKNOWN"),
                            shape_result.get("x_cm", 0.0),
                            shape_result.get("confidence", 0.0),
                            shape_ms,
                        )
                    )
                    print("Continue with a fresh 1080p attempt.")
                    attempt["high_image"] = None
                    gc.collect()
                    continue

                if task_mode == MODE_TILT_MIN:
                    fused_distance = fuse_tilt_localizations(
                        matched["distance_result"],
                        distance_result,
                    )
                else:
                    fused_distance = fuse_distance_results((
                        matched["distance_result"],
                        distance_result,
                    ))
                fused_distance["frame_center_shift_ratio"] = (
                    matched_center_shift_ratio
                )
                fused_distance["x_cm"] = shape_result["x_cm"]
                fused_distance["shape_type"] = shape_result["shape_type"]
                fused_distance["shape_confidence"] = shape_result[
                    "confidence"
                ]
                fused_distance["task_mode"] = task_mode
                fused_distance["target_digit"] = target_digit
                high_path = "%s/measure_%03d_1080.jpg" % (
                    SAVE_DIR,
                    current_id,
                )
                preview_path = "%s/measure_%03d_preview.jpg" % (
                    SAVE_DIR,
                    current_id,
                )
                metadata_path = "%s/measure_%03d.txt" % (
                    SAVE_DIR,
                    current_id,
                )
                plane_mask_path = None
                if (
                    task_mode != "BASIC"
                    and advanced_detector is not None
                    and advanced_detector.last_mask is not None
                ):
                    plane_mask_path = "%s/measure_%03d_plane.pgm" % (
                        SAVE_DIR,
                        current_id,
                    )
                    advanced_detector.last_mask.save_pgm(plane_mask_path)
                    shape_result["plane_mask_path"] = plane_mask_path
                    advanced_detector.last_mask = None
                preview_result = attempt["preview_result"]
                if task_mode == "BASIC":
                    preview_result["shape_result"] = scale_shape_result(
                        shape_result,
                        PREVIEW_WIDTH / CAPTURE_WIDTH,
                        PREVIEW_HEIGHT / CAPTURE_HEIGHT,
                    )
                else:
                    preview_result["shape_result"] = scale_advanced_result(
                        shape_result,
                        PREVIEW_WIDTH / CAPTURE_WIDTH,
                        PREVIEW_HEIGHT / CAPTURE_HEIGHT,
                    )
                status_color = (
                    (0, 255, 80)
                    if fused_distance["method"] == "CALIBRATION_TABLE"
                    else (0, 220, 255)
                )

                high_image = attempt["high_image"]
                draw_measurement_overlay(
                    high_image,
                    high_result,
                    status_color,
                    accepted=True,
                )
                if task_mode == "BASIC":
                    draw_shape_overlay(
                        high_image,
                        shape_result,
                        color=(255, 255, 0),
                        thickness=2,
                    )
                else:
                    draw_advanced_overlay(
                        high_image,
                        shape_result,
                        selected_color=(255, 40, 40),
                        other_color=(0, 255, 80),
                        thickness=2,
                    )
                high_image.save(high_path, quality=SAVED_JPG_QUALITY)
                attempt["high_image"] = None
                high_image = None
                gc.collect()

                draw_measurement_overlay(
                    last_preview,
                    preview_result,
                    status_color,
                )
                last_preview.save(
                    preview_path,
                    quality=PREVIEW_JPG_QUALITY,
                )
                total_ms = time.ticks_diff(time.ticks_ms(), total_start)
                fused_distance["measurement_id"] = current_id
                fused_distance["total_ms"] = total_ms
                save_metadata(
                    metadata_path,
                    attempt["coarse_result"],
                    attempt["coarse_candidate_count"],
                    high_result,
                    fused_distance,
                    shape_result,
                    coarse_total_ms,
                    convert_total_ms,
                    refine_total_ms,
                    shape_ms,
                    total_ms,
                    attempt_index,
                )

                print(
                    "CONSISTENCY_OK pair=%d/%d scales=%.6f,%.6f delta=%.4f%% center_shift=%.3f%%"
                    % (
                        matched["attempt_index"],
                        attempt_index,
                        matched["distance_result"]["frame_scale"],
                        current_scale,
                        matched_difference * 100.0,
                        matched_center_shift_ratio * 100.0,
                    )
                )
                print(
                    "distance=%.2fcm method=%s source=%s fused_scale=%.6f spread=%.4f%%"
                    % (
                        fused_distance["distance_cm"],
                        fused_distance["method"],
                        fused_distance["source"],
                        fused_distance["frame_scale"],
                        fused_distance["frame_scale_spread_pct"],
                    )
                )
                print(
                    "shape=%s x=%.2fcm confidence=%.3f fill=%.3f time=%dms"
                    % (
                        shape_result["shape_type"],
                        shape_result["x_cm"],
                        shape_result["confidence"],
                        shape_result["fill_ratio"],
                        shape_ms,
                    )
                )
                if matched_shape_gate is not None:
                    print(
                        "advanced_pair count=%d/%d side_delta=%.3fcm center_shift=%.3fcm"
                        % (
                            matched_shape_gate.get("count_first", 0),
                            matched_shape_gate.get("count_second", 0),
                            matched_shape_gate.get(
                                "side_difference_cm", 999.0
                            ),
                            matched_shape_gate.get("center_shift_cm", 999.0),
                        )
                    )
                if shape_result.get("image_corners") is not None:
                    print(
                        "corners=%d corner_x=%.2fcm side_cv=%.3f line_rms=%.3fcm area_delta=%.3f"
                        % (
                            len(shape_result["image_corners"]),
                            shape_result.get("corner_x_cm", 0.0),
                            shape_result.get("corner_side_cv", 1.0),
                            shape_result.get(
                                "corner_max_line_rms_cm", -1.0
                            ),
                            shape_result.get(
                                "corner_area_disagreement", 1.0
                            ),
                        )
                    )
                if task_mode == MODE_TILT_MIN:
                    print(
                        "RESULT task=TILT x=%.2fcm type=%s D=NOT_REQUIRED"
                        % (
                            shape_result["x_cm"],
                            shape_result["shape_type"],
                        )
                    )
                else:
                    print(
                        "RESULT task=%s D=%.2fcm x=%.2fcm type=%s"
                        % (
                            task_mode,
                            fused_distance["distance_cm"],
                            shape_result["x_cm"],
                            shape_result["shape_type"],
                        )
                    )
                print("High-resolution image:", high_path)
                print("Preview image:", preview_path)
                if plane_mask_path is not None:
                    print("Rectified plane mask:", plane_mask_path)
                print("Metadata:", metadata_path)
                print(
                    "MEASURE_DONE id=%03d attempts=%d total=%dms"
                    % (current_id, attempt_index, total_ms)
                )
                remove_file_if_exists(failed_high_path)
                last_measurement_failure_reason = None
                print("Send the next UART task command.")
                return preview_result, fused_distance, status_color

            strong_records.append({
                "attempt_index": attempt_index,
                "distance_result": distance_result,
                "high_result": high_result,
                "shape_result": shape_result,
                "shape_ms": shape_ms,
            })
            if last_shape_gate is not None:
                print(
                    "ADVANCED_CONSISTENCY_WAIT reason=%s side_delta=%.3fcm center_shift=%.3fcm"
                    % (
                        last_shape_gate.get("reason", "UNKNOWN"),
                        last_shape_gate.get("side_difference_cm", 999.0),
                        last_shape_gate.get("center_shift_cm", 999.0),
                    )
                )
            print(
                "CONSISTENCY_WAIT strong=%d/%d scale=%.6f"
                % (len(strong_records), 2, current_scale)
            )

        elif current_tracking_hint is not None:
            # A high-resolution rejection means the old spatial hint was not
            # trustworthy. The next independent attempt returns to a cold
            # full-height central-strip search.
            print("tracking hint dropped after rejected refinement")
            current_tracking_hint = None

        attempt["high_image"] = None
        gc.collect()

    total_ms = time.ticks_diff(time.ticks_ms(), total_start)
    failed_path = save_failed_preview(
        last_preview,
        current_id,
        coarse_diagnostics=last_coarse_diagnostics,
        tracking_roi=last_tracking_roi,
    )
    failed_plane_path = None
    if (
        task_mode != "BASIC"
        and advanced_detector is not None
        and advanced_detector.last_mask is not None
    ):
        failed_plane_path = "%s/measure_%03d_failed_plane.pgm" % (
            SAVE_DIR,
            current_id,
        )
        advanced_detector.last_mask.save_pgm(failed_plane_path)
        advanced_detector.last_mask = None
    metadata_path = "%s/measure_%03d.txt" % (SAVE_DIR, current_id)
    save_failed_metadata(
        metadata_path,
        summaries,
        total_ms,
        strong_count=len(strong_records),
        task_mode=task_mode,
        target_digit=target_digit,
    )
    last_measurement_failure_reason = derive_detailed_failure_reason(
        summaries, len(strong_records)
    )
    print(
        "MEASURE_FAILED id=%03d reason=NO_CONSISTENT_PAIR_OR_VALID_SHAPE strong=%d attempts=%d total=%dms"
        % (
            current_id,
            len(strong_records),
            max_attempts,
            total_ms,
        )
    )
    print("Failed preview:", failed_path)
    if failed_high_path is not None:
        print("Failed 1080p diagnostic:", failed_high_path)
    if failed_plane_path is not None:
        print("Failed rectified plane mask:", failed_plane_path)
    print("Metadata:", metadata_path)
    print("Send the same UART task command to retry.")
    return None, None, (255, 50, 50)


try:
    ensure_dir(SAVE_DIR)
    measurement_id = find_last_measurement_id()
    os.exitpoint(os.EXITPOINT_ENABLE)

    uart = create_uart1()
    if USE_TJC_SCREEN:
        screen = TJCScreen(uart)
    detector = FrameDetector(
        rect_threshold=RECT_THRESHOLD,
        expected_short_ratio=EXPECTED_SHORT_RATIO,
        expected_long_ratio=EXPECTED_LONG_RATIO,
        expected_outer_aspect=EXPECTED_OUTER_ASPECT,
        expected_inner_aspect=EXPECTED_INNER_ASPECT,
        allow_inner_only=True,
    )
    refiner = HighResRefiner(
        expected_short_ratio=EXPECTED_SHORT_RATIO,
        expected_long_ratio=EXPECTED_LONG_RATIO,
        expected_inner_aspect=EXPECTED_INNER_ASPECT,
        expected_outer_aspect=EXPECTED_OUTER_ASPECT,
    )
    aperture_locator = InnerApertureLocator(
        expected_aspect=EXPECTED_INNER_ASPECT,
    )
    shape_detector = BasicShapeDetector()
    advanced_detector = AdvancedTargetDetector(pixels_per_cm=8.0)
    power_monitor = PowerMonitor(
        hardware_poll_interval_ms=INA226_POLL_INTERVAL_MS,
        current_offset_a=INA226_CURRENT_OFFSET_A,
    )
    if USE_INA226:
        try:
            ina226 = create_ina226_monitor(
                i2c_id=INA226_I2C_ID,
                scl_pin=INA226_SCL_PIN,
                sda_pin=INA226_SDA_PIN,
                address=INA226_ADDRESS,
                freq=INA226_FREQ,
                r_shunt=INA226_R_SHUNT_OHM,
                current_lsb=INA226_CURRENT_LSB_A,
                config=INA226_CONFIG,
                verify=True,
            )
            power_monitor.attach_hardware(ina226)
            print(
                "INA226 ready: IIC%d SCL=GPIO%d SDA=GPIO%d addr=0x%02X"
                % (
                    INA226_I2C_ID,
                    INA226_SCL_PIN,
                    INA226_SDA_PIN,
                    INA226_ADDRESS,
                )
            )
            print("INA226 range:", ina226.range_info())
            print(
                "INA226 current correction: +%.3f A (hardware only)"
                % INA226_CURRENT_OFFSET_A
            )
            print("INA226 sample:", power_monitor.status_line())
        except Exception as error:
            # Camera measurement must remain usable if the power module is
            # disconnected during development.  UART/manual W=... input is
            # still accepted when the screen is disabled.
            print("INA226_INIT_ERROR:", repr(error))
            print(
                "INA226_HINT: check scan in the error; [] means no I2C ACK. "
                "Verify 3.3V/GND, GPIO34=SCL, GPIO35=SDA and module address."
            )
            print("Power monitor fallback: UART/manual samples")
    if screen is not None:
        safe_screen_call("reset_display")

    sensor = Sensor(
        id=2,
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        fps=30,
    )
    sensor.reset()
    sensor.set_framesize(
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        chn=CAPTURE_CH,
    )
    sensor.set_pixformat(Sensor.RGB888, chn=CAPTURE_CH)
    sensor.set_framesize(
        width=PREVIEW_WIDTH,
        height=PREVIEW_HEIGHT,
        chn=PREVIEW_CH,
    )
    sensor.set_pixformat(Sensor.RGB565, chn=PREVIEW_CH)

    Display.init(
        Display.ST7701,
        width=DISPLAY_WIDTH,
        height=DISPLAY_HEIGHT,
        to_ide=True,
        quality=IDE_PREVIEW_QUALITY,
    )
    display_inited = True
    sensor.run()

    print("Warming up camera...")
    for _ in range(30):
        time.sleep_ms(50)
        os.exitpoint()

    print("")
    print("Two-frame-consistent D/x measurement ready.")
    print("Pipeline version:", PIPELINE_VERSION)
    print("UART1: 115200 8N1, header pin 3 TXD / pin 4 RXD")
    print("Commands:")
    print("  M  = basic CIRCLE/TRIANGLE/SQUARE D+x")
    print("  A  = advanced random target D+minimum-square x")
    print("  N1..N9 = numbered square D+selected x")
    print("  T  = 30-60 degree tilted plane, x only")
    print("  W=1.23 = update measured input power; I=0.823,W=4.12 = current+power")
    print("  PW = P/Pmax status, R = reset Pmax, S = status, P = ping")
    print(
        "Power backend: INA226 IIC1 GPIO34/35 when detected; UART/manual fallback"
    )
    print(
        "Preview/coarse: %dx%d RGB565, high capture: %dx%d RGB888 -> RGB565 refinement"
        % (PREVIEW_WIDTH, PREVIEW_HEIGHT, CAPTURE_WIDTH, CAPTURE_HEIGHT)
    )
    print(
        "Coarse: center_roi=%s thresholds=%d->%d (unscaled); aperture_fallback=PREVIEW/1080P; refine_limit=%d"
        % (
            format_roi(center_search_roi(PREVIEW_WIDTH, PREVIEW_HEIGHT)),
            RECT_THRESHOLD,
            FALLBACK_RECT_THRESHOLD,
            MAX_HIGH_RES_HYPOTHESES,
        )
    )
    print(
        "Each new trigger searches only the full-height central strip: CENTER, CENTER_LOW, then shape-independent inner-aperture recovery."
    )
    print(
        "After the first STRONG frame, the second frame uses direct high-resolution tracking only within the same trigger."
    )
    print(
        "A STRONG inner quad may pass without independent outer sides when all ring/geometry gates pass."
    )
    print(
        "Each task requires 2 consistent localizations within %.3f%% scale and %.1f%% center shift, max %d attempts (%d for TILT)."
        % (
            MAX_FRAME_SCALE_DISAGREEMENT * 100.0,
            MAX_FRAME_CENTER_SHIFT_RATIO * 100.0,
            MAX_MEASUREMENT_ATTEMPTS,
            MAX_TILT_MEASUREMENT_ATTEMPTS,
        )
    )
    print("After localization: homography maps the 17.0x25.7cm plane.")
    print("Advanced modes reconstruct overlapping/rotated squares at 8 pixels/cm.")
    print("A/N/T require the advanced target result to agree in two frames.")
    print("No localization runs while waiting for a UART command.")
    if screen is not None:
        print(
            "TJC screen enabled: GPIO9=UART1_TXD, GPIO10=UART1_RXD, 115200 8N1"
        )
        print(
            "Screen modes: 0=BASIC, 1=MIN_SQUARE, 2=NUMBERED, 3=TILTED"
        )
        print(
            "Screen live power refresh: %dms"
            % SCREEN_POWER_UPDATE_INTERVAL_MS
        )

    locked_preview_result = None
    locked_distance_result = None
    locked_status_color = (80, 120, 255)
    measurement_state = "IDLE"
    frame_count = 0
    clock = time.clock()
    uart_send_line(
        "READY SINGLE_SHOT_MEASUREMENT BAUD=%d VER=%s"
        % (UART_BAUDRATE, PIPELINE_VERSION)
    )
    send_uart_status(measurement_state, measurement_id, None)

    while True:
        os.exitpoint()
        clock.tick()
        preview = sensor.snapshot(chn=PREVIEW_CH)
        # INA226 sampling is throttled internally, so this does not run an
        # I2C transaction on every 27--30 FPS preview frame.
        power_monitor.poll_hardware()

        command = (
            poll_screen_command()
            if screen is not None
            else poll_uart_command()
        )
        if command == "PING":
            uart_send_line("PONG VER=%s" % PIPELINE_VERSION)
        elif command == "HELP":
            uart_send_line(
                "HELP M=BASIC A=MIN N1..N9=DIGIT T=TILT S=STATUS P=PING PW=POWER R=RESET_POWER W=<watts> I=<amps>,W=<watts>"
            )
        elif command == "POWER_STATUS":
            power_monitor.poll_hardware(force=True)
            uart_send_line(power_monitor.status_line())
        elif command == "POWER_RESET":
            power_monitor.reset_maximum()
            power_monitor.poll_hardware(force=True)
            uart_send_line("ACK POWER_RESET " + power_monitor.status_line())
            refresh_touch_screen(locked_distance_result)
        elif command in ("SCREEN_READY", "SCREEN_REFRESH"):
            if command == "SCREEN_READY":
                safe_screen_call("reset_display")
                set_screen_status(TJCScreen.STATUS_IDLE)
            refresh_touch_screen(locked_distance_result)
        elif command == "SCREEN_BAD_DIGIT":
            set_screen_status(TJCScreen.STATUS_NUMBER_NOT_FOUND)
        elif command in ("SCREEN_BAD_MODE", "SCREEN_UNKNOWN_CMD"):
            set_screen_status(TJCScreen.STATUS_SYSTEM_ERROR)
        elif command is not None and command.startswith("POWER_SAMPLE:"):
            try:
                watts = float(command[13:])
                power_monitor.update_watts(watts)
                uart_send_line("ACK " + power_monitor.status_line())
                refresh_touch_screen(locked_distance_result)
            except Exception:
                uart_send_line("ERROR POWER_SAMPLE EXPECTED=W=<watts>")
        elif command is not None and command.startswith("POWER_SAMPLE_PAIR:"):
            try:
                fields = command[len("POWER_SAMPLE_PAIR:"):].split(":")
                current_a = None if not fields[0] else float(fields[0])
                power_w = None if not fields[1] else float(fields[1])
                power_monitor.update_sample(
                    current_a=current_a,
                    watts=power_w,
                )
                uart_send_line("ACK " + power_monitor.status_line())
                refresh_touch_screen(locked_distance_result)
            except Exception:
                uart_send_line(
                    "ERROR POWER_SAMPLE EXPECTED=I=<amps>,W=<watts>"
                )
        elif command == "STATUS":
            send_uart_status(
                measurement_state,
                measurement_id,
                locked_distance_result,
            )
            uart_send_line(power_monitor.status_line())
        elif command is not None and command.startswith("UNKNOWN:"):
            print("UART_UNKNOWN_COMMAND:", command[8:])
            uart_send_line(
                "ERROR UNKNOWN_COMMAND EXPECTED=M|A|N1..N9|T|S|P|PW|R|W=<watts>|I=<amps>,W=<watts>"
            )
        elif command in (
            "MEASURE",
            "ADVANCED_MIN",
            "TILT_MIN",
        ) or (command is not None and command.startswith("DIGIT:")):
            task_mode = "BASIC"
            target_digit = None
            if command == "ADVANCED_MIN":
                task_mode = MODE_AUTO_MIN
            elif command == "TILT_MIN":
                task_mode = MODE_TILT_MIN
            elif command.startswith("DIGIT:"):
                task_mode = MODE_DIGIT_SELECT
                try:
                    target_digit = int(command[6:])
                except Exception:
                    target_digit = -1
                if target_digit < 0 or target_digit > 9:
                    uart_send_line("ERROR DIGIT_RANGE EXPECTED=N0..N9")
                    continue
            next_id = measurement_id + 1
            measurement_state = "BUSY"
            set_screen_task_label(task_mode, target_digit)
            set_screen_status(TJCScreen.STATUS_MEASURING)
            safe_screen_call("clear_measurement_values")
            current_a, power_w, pmax_w = screen_power_values()
            safe_screen_call("update_power", current_a, power_w, pmax_w)
            uart_send_line(
                "ACK MEASURE ID=%03d TASK=%s DIGIT=%s VER=%s"
                % (
                    next_id,
                    task_mode,
                    str(target_digit),
                    PIPELINE_VERSION,
                )
            )
            locked_preview_result = None
            locked_distance_result = None
            locked_status_color = (255, 210, 0)
            measurement_error = False
            power_monitor.poll_hardware(force=True)
            try:
                (
                    locked_preview_result,
                    locked_distance_result,
                    locked_status_color,
                ) = measure_once(
                    preview,
                    detector,
                    refiner,
                    aperture_locator,
                    shape_detector,
                    advanced_detector=advanced_detector,
                    task_mode=task_mode,
                    target_digit=target_digit,
                )
            except Exception as error:
                # One failed request must not tear down the camera.  Keep
                # preview/UART alive so the host can send M again.
                print("MEASURE_ERROR:", repr(error))
                measurement_error = True
                locked_preview_result = None
                locked_distance_result = None
                locked_status_color = (255, 50, 50)
                gc.collect()

            # Do not queue extra measurements typed while image processing
            # was blocking.  The host must wait for RESULT before sending M.
            drain_uart_input()
            if locked_distance_result is not None:
                measurement_state = "LOCKED"
                set_screen_status(TJCScreen.STATUS_SUCCESS)
                refresh_touch_screen(locked_distance_result)
                if task_mode == MODE_TILT_MIN:
                    uart_send_line(
                        "RESULT OK ID=%03d TASK=%s X=%.2f TYPE=%s CONF=%.3f D=NOT_REQUIRED TIME=%d VER=%s"
                        % (
                            measurement_id,
                            task_mode,
                            locked_distance_result.get("x_cm", 0.0),
                            locked_distance_result.get(
                                "shape_type", "UNKNOWN"
                            ),
                            locked_distance_result.get(
                                "shape_confidence", 0.0
                            ),
                            locked_distance_result.get("total_ms", 0),
                            PIPELINE_VERSION,
                        )
                    )
                else:
                    uart_send_line(
                        "RESULT OK ID=%03d TASK=%s DIGIT=%s D=%.2f X=%.2f TYPE=%s CONF=%.3f METHOD=%s SCALE=%.6f TIME=%d VER=%s"
                        % (
                            measurement_id,
                            task_mode,
                            str(target_digit),
                            locked_distance_result.get("distance_cm", 0.0),
                            locked_distance_result.get("x_cm", 0.0),
                            locked_distance_result.get(
                                "shape_type", "UNKNOWN"
                            ),
                            locked_distance_result.get(
                                "shape_confidence", 0.0
                            ),
                            locked_distance_result.get("method", "UNKNOWN"),
                            locked_distance_result.get("frame_scale", 0.0),
                            locked_distance_result.get("total_ms", 0),
                            PIPELINE_VERSION,
                        )
                    )
            else:
                measurement_state = "FAILED"
                failure_status = screen_failure_status(
                    last_measurement_failure_reason,
                    task_mode,
                    measurement_error=measurement_error,
                )
                set_screen_status(failure_status)
                refresh_touch_screen(None)
                if measurement_error:
                    uart_send_line(
                        "RESULT ERROR ID=%03d REASON=MEASURE_EXCEPTION"
                        " VER=%s" % (measurement_id, PIPELINE_VERSION)
                    )
                else:
                    uart_send_line(
                        "RESULT RETRY ID=%03d REASON=NO_CONSISTENT_PAIR_OR_VALID_SHAPE VER=%s"
                        % (measurement_id, PIPELINE_VERSION)
                    )
            preview = sensor.snapshot(chn=PREVIEW_CH)

        # Keep the external screen's electrical measurements live even while
        # the vision system is idle.  This updates only I/P/Pmax and therefore
        # does not overwrite the locked distance, size or status fields.
        periodic_screen_power_update()

        draw_measurement_overlay(
            preview,
            locked_preview_result,
            locked_status_color,
        )
        # Keep these synthetic lines out of every find_rects() input.  The
        # command is handled before this display-only decoration is applied.
        draw_alignment_guides(preview)
        power_monitor.draw_overlay(preview)
        Display.show_image(preview, x=PREVIEW_X, y=PREVIEW_Y)

        frame_count += 1
        if frame_count % 120 == 0:
            print(
                "preview fps=%.1f state=%s"
                % (
                    clock.fps(),
                    measurement_state,
                )
            )
            gc.collect()
        time.sleep_ms(1)


except KeyboardInterrupt:
    print("")
    print("Stopped by CanMV IDE.")


except Exception as error:
    print("")
    print("PROGRAM_ERROR:", repr(error))


finally:
    if sensor is not None:
        try:
            sensor.stop()
        except Exception:
            pass
    if display_inited:
        try:
            Display.deinit()
        except Exception:
            pass
    if uart is not None:
        try:
            uart.deinit()
        except Exception:
            pass
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("Camera stopped.")
