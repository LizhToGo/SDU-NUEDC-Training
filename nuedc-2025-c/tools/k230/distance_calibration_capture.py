"""Collect statistically useful distance-scale calibration data on K230.

Run this script from CanMV IDE after a soft reboot.  Put the target marker at
the distance printed in the IDE console, then send ``M\n`` (or the single byte
``1``) to UART1.  One command continuously collects all five accepted samples
for the current point.  Every sample still requires two independent STRONG
frame scales to pass both the scale and center-motion consistency gates.

The v3 collector reuses only the last accepted target geometry as a spatial
seed.  Every scale is still remeasured from a fresh 1920x1080 frame.  A failed
tracked refinement drops the seed and returns to the bounded cold-start coarse
search.  The last strict seed is also carried between the five samples and to
the next 5 cm point, where the small scale change is recovered by the existing
high-resolution edge search.

The v3 tracked run uses a new CSV filename, so it cannot mix with either the
old v1 10 cm table or the unreliable v2 cold-start calibration samples.  It
resumes that new CSV by default; set RESET_EXISTING_DATA=True for exactly one
run only when the v3 data itself must be restarted.
"""

import gc
import os
import time

from media.sensor import *
from media.display import *
from ybUtils.YbUart import YbUart

from frame_detector import FrameDetector
from high_res_refiner import HighResRefiner
from distance_estimator import estimate_distance, fuse_distance_results
from measurement_consistency import (
    relative_scale_difference,
    relative_center_shift,
)


CALIBRATION_CAPTURE_VERSION = "2026-07-21-v3-tracked-batch"

SAVE_DIR = "/data/captures"
# Use new filenames so tracked v3 samples can never silently mix with the
# former v1 table or the v2 cold-start batches collected at 115/130 cm.
CALIBRATION_CSV = SAVE_DIR + "/distance_calibration_5cm_v3.csv"
CALIBRATION_SUMMARY = (
    SAVE_DIR + "/distance_calibration_5cm_v3_summary.txt"
)

# Change this to True for exactly one run when a fresh data set is required.
RESET_EXISTING_DATA = False
DISTANCE_POINTS_CM = (
    100.0,
    105.0,
    110.0,
    115.0,
    120.0,
    125.0,
    130.0,
    135.0,
    140.0,
    145.0,
    150.0,
    155.0,
    160.0,
    165.0,
    170.0,
    175.0,
    180.0,
    185.0,
    190.0,
    195.0,
    200.0,
)
SAMPLES_PER_DISTANCE = 5
# One UART trigger fills the current distance point.  Each batch collection
# already has its own bounded two-frame consistency search; this outer bound
# prevents a badly placed target from keeping the UART command busy forever.
MAX_BATCH_SAMPLE_ATTEMPTS = 15

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

CENTER_SEARCH_CENTER_X_RATIO = 0.54
CENTER_SEARCH_WIDTH_RATIO = 0.42

RECT_THRESHOLD = 1800
FALLBACK_RECT_THRESHOLD = 1200
COARSE_HYPOTHESES_PER_ATTEMPT = 4
MAX_HIGH_RES_HYPOTHESES = 3
HIGH_RES_FLUSH_FRAMES = 2

MAX_MEASUREMENT_ATTEMPTS = 5
MAX_FRAME_SCALE_DISAGREEMENT = 0.0050
MAX_FRAME_CENTER_SHIFT_RATIO = 0.10

# Header pin 3 is UART1_TXD and header pin 4 is UART1_RXD on the Yahboom K230.
# YbUart is the vendor wrapper already shipped in /sdcard/ybUtils.
UART_BAUDRATE = 115200
UART_RX_BUFFER_LIMIT = 64
TRIGGER_SETTLE_MS = 0
POST_TRIGGER_PREVIEW_FLUSH_FRAMES = 2

EXPECTED_SHORT_RATIO = 17.0 / 21.0
EXPECTED_LONG_RATIO = 25.7 / 29.7
EXPECTED_OUTER_ASPECT = 29.7 / 21.0
EXPECTED_INNER_ASPECT = 25.7 / 17.0

CSV_HEADER = (
    "distance_cm,sample_index,frame_scale,provisional_distance_cm,"
    "component_scale_1,component_scale_2,scale_spread_pct,"
    "center_shift_ratio,measurement_attempts,total_ms,coarse_ms,"
    "convert_ms,refine_ms,mode_1,mode_2,demoted_1,demoted_2,"
    "inner_rms_1,inner_rms_2,ring_inside_1,ring_inside_2,"
    "ring_outside_1,ring_outside_2\n"
)


sensor = None
display_inited = False
uart = None
uart_rx_buffer = ""


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def remove_if_exists(path):
    try:
        os.remove(path)
    except OSError:
        pass


def uart_send_line(message):
    if uart is None:
        return
    try:
        uart.send(message + "\n")
    except Exception as error:
        print("UART_TX_ERROR:", repr(error))


def normalize_uart_command(raw_command):
    command = raw_command.strip().upper()
    if not command:
        return None
    if command in ("M", "MEASURE", "1"):
        return "MEASURE"
    if command in ("S", "STATUS", "?"):
        return "STATUS"
    if command in ("P", "PING"):
        return "PING"
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

    # A single-byte command is accepted without a line ending.  Full words
    # deliberately require CR/LF so a fragmented "MEASURE" cannot trigger
    # again when its remaining bytes arrive after a measurement.
    if uart_rx_buffer in ("M", "m", "1", "S", "s", "P", "p", "?"):
        command = normalize_uart_command(uart_rx_buffer)
        uart_rx_buffer = ""
        return command
    if len(uart_rx_buffer) > UART_RX_BUFFER_LIMIT:
        command = "UNKNOWN:" + uart_rx_buffer[:UART_RX_BUFFER_LIMIT]
        uart_rx_buffer = ""
        return command
    return None


def drain_uart_input():
    global uart_rx_buffer
    uart_rx_buffer = ""
    for _ in range(4):
        try:
            data = uart.read()
        except Exception:
            return
        if not data:
            return
        time.sleep_ms(2)


def send_uart_status(current_index, scales_by_distance):
    if current_index >= len(DISTANCE_POINTS_CM):
        uart_send_line("STATUS COMPLETE")
        return
    uart_send_line(
        "STATUS D=%.1f COUNT=%d/%d"
        % (
            DISTANCE_POINTS_CM[current_index],
            len(scales_by_distance[current_index]),
            SAMPLES_PER_DISTANCE,
        )
    )


def median(values):
    if not values:
        return 0.0
    ordered = list(values)
    ordered.sort()
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def mad(values):
    if not values:
        return 0.0
    center = median(values)
    deviations = []
    for value in values:
        deviations.append(abs(value - center))
    return median(deviations)


def distance_index(distance_cm):
    for index in range(len(DISTANCE_POINTS_CM)):
        if abs(DISTANCE_POINTS_CM[index] - distance_cm) < 0.01:
            return index
    return -1


def initialize_storage():
    if RESET_EXISTING_DATA:
        remove_if_exists(CALIBRATION_CSV)
        remove_if_exists(CALIBRATION_SUMMARY)
    try:
        with open(CALIBRATION_CSV, "r") as file:
            first_line = file.readline()
        if first_line.startswith("distance_cm,"):
            return
    except OSError:
        pass
    with open(CALIBRATION_CSV, "w") as file:
        file.write(CSV_HEADER)


def load_scales_by_distance():
    scales = []
    for _ in DISTANCE_POINTS_CM:
        scales.append([])
    try:
        with open(CALIBRATION_CSV, "r") as file:
            # Keep the resume path streaming so it also remains cheap after
            # repeated calibration runs on MicroPython's small heap.
            file.readline()
            while True:
                line = file.readline()
                if not line:
                    break
                parts = line.strip().split(",")
                if len(parts) < 3:
                    continue
                try:
                    measured_distance = float(parts[0])
                    measured_scale = float(parts[2])
                except (ValueError, TypeError):
                    continue
                index = distance_index(measured_distance)
                if (
                    index >= 0
                    and len(scales[index]) < SAMPLES_PER_DISTANCE
                ):
                    scales[index].append(measured_scale)
    except OSError:
        return scales
    return scales


def first_incomplete_index(scales_by_distance):
    for index in range(len(DISTANCE_POINTS_CM)):
        if len(scales_by_distance[index]) < SAMPLES_PER_DISTANCE:
            return index
    return len(DISTANCE_POINTS_CM)


def write_summary(scales_by_distance):
    with open(CALIBRATION_SUMMARY, "w") as file:
        file.write(
            "calibration_capture_version=%s\n"
            % CALIBRATION_CAPTURE_VERSION
        )
        file.write("distance_point_count=%d\n" % len(DISTANCE_POINTS_CM))
        file.write("distance_step_cm=5.0\n")
        file.write("samples_per_distance=%d\n" % SAMPLES_PER_DISTANCE)
        file.write(
            "max_scale_disagreement=%.6f\n"
            % MAX_FRAME_SCALE_DISAGREEMENT
        )
        file.write(
            "max_center_shift_ratio=%.6f\n"
            % MAX_FRAME_CENTER_SHIFT_RATIO
        )
        file.write("\nstatistics:\n")
        for index in range(len(DISTANCE_POINTS_CM)):
            values = scales_by_distance[index]
            if not values:
                continue
            file.write(
                "distance=%.1f count=%d median_scale=%.6f mad=%.6f min=%.6f max=%.6f\n"
                % (
                    DISTANCE_POINTS_CM[index],
                    len(values),
                    median(values),
                    mad(values),
                    min(values),
                    max(values),
                )
            )
        file.write("\nDISTANCE_CALIBRATION_POINTS = (\n")
        for index in range(len(DISTANCE_POINTS_CM)):
            values = scales_by_distance[index]
            if values:
                file.write(
                    "    (%.1f, %.6f),\n"
                    % (DISTANCE_POINTS_CM[index], median(values))
                )
        file.write(")\n")


def append_record(record):
    with open(CALIBRATION_CSV, "a") as file:
        file.write(
            "%.1f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%d,%s,%s,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"
            % (
                record["distance_cm"],
                record["sample_index"],
                record["frame_scale"],
                record["provisional_distance_cm"],
                record["component_scale_1"],
                record["component_scale_2"],
                record["scale_spread_pct"],
                record["center_shift_ratio"],
                record["measurement_attempts"],
                record["total_ms"],
                record["coarse_ms"],
                record["convert_ms"],
                record["refine_ms"],
                record["mode_1"],
                record["mode_2"],
                record["demoted_1"],
                record["demoted_2"],
                record["inner_rms_1"],
                record["inner_rms_2"],
                record["ring_inside_1"],
                record["ring_inside_2"],
                record["ring_outside_1"],
                record["ring_outside_2"],
            )
        )


def center_search_roi(image_width, image_height):
    """Match the production v2 full-height central search strip."""
    roi_width = max(
        2,
        int(image_width * CENTER_SEARCH_WIDTH_RATIO + 0.5),
    )
    roi_width = min(roi_width, image_width)
    center_x = int(image_width * CENTER_SEARCH_CENTER_X_RATIO + 0.5)
    x0 = center_x - roi_width // 2
    x0 = max(0, min(x0, image_width - roi_width))
    return (x0, 0, roi_width, image_height)


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
    """Keep only preview-space geometry; never retain the 1080p image."""
    if high_result is None:
        return None
    scale_x = PREVIEW_WIDTH / CAPTURE_WIDTH
    scale_y = PREVIEW_HEIGHT / CAPTURE_HEIGHT
    return {
        "mode": high_result["mode"],
        "outer_corners": scale_corners(
            high_result.get("outer_corners"), scale_x, scale_y
        ),
        "inner_corners": scale_corners(
            high_result.get("inner_corners"), scale_x, scale_y
        ),
    }


def tracking_seed_from_result(result):
    """Turn the last strict 1080p lock into a location-only refine seed."""
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


def has_usable_location_seed(results):
    for result in results:
        if result.get("seed_class", "REJECT") != "REJECT":
            return True
    return False


def run_calibration_coarse_scan(
    preview,
    detector,
    scope,
    roi,
    threshold_override,
    allow_location_seed,
):
    start = time.ticks_ms()
    results = detector.detect_hypotheses(
        preview,
        roi=roi,
        max_results=COARSE_HYPOTHESES_PER_ATTEMPT,
        threshold_override=threshold_override,
        scale_roi_threshold=False,
        allow_location_seed=allow_location_seed,
    )
    elapsed = time.ticks_diff(time.ticks_ms(), start)
    if results:
        print(
            "cal coarse scope=%s threshold=%d hypotheses=%d best=%s seed=%s score=%.3f raw=%d rects=%d time=%dms"
            % (
                scope,
                detector.last_threshold,
                len(results),
                results[0]["mode"],
                results[0].get("seed_class", "UNKNOWN"),
                results[0]["score"],
                detector.last_raw_rect_count,
                len(detector.last_candidates),
                elapsed,
            )
        )
    else:
        print(
            "cal coarse scope=%s threshold=%d NONE raw=%d rects=%d time=%dms"
            % (
                scope,
                detector.last_threshold,
                detector.last_raw_rect_count,
                len(detector.last_candidates),
                elapsed,
            )
        )
    return results, elapsed


def capture_coarse_candidates(first_preview, detector):
    """Use the production v2 CENTER -> CENTER_LOW -> FULL cascade."""
    candidates = []
    preview = first_preview
    coarse_ms = 0
    search_roi = center_search_roi(preview.width(), preview.height())
    center_results, elapsed = run_calibration_coarse_scan(
        preview,
        detector,
        "CENTER",
        search_roi,
        None,
        True,
    )
    coarse_ms += elapsed
    candidates.extend(center_results)

    if not has_usable_location_seed(center_results):
        center_low_results, elapsed = run_calibration_coarse_scan(
            preview,
            detector,
            "CENTER_LOW",
            search_roi,
            FALLBACK_RECT_THRESHOLD,
            True,
        )
        coarse_ms += elapsed
        candidates.extend(center_low_results)

        if not has_usable_location_seed(center_low_results):
            full_results, elapsed = run_calibration_coarse_scan(
                preview,
                detector,
                "FULL_FALLBACK",
                None,
                FALLBACK_RECT_THRESHOLD,
                False,
            )
            coarse_ms += elapsed
            candidates.extend(full_results)
        else:
            print("cal coarse CENTER_LOW hit; FULL_FALLBACK skipped")
    else:
        print("cal coarse CENTER hit; lower/full scans skipped")

    candidates = detector.select_distinct_hypotheses(
        candidates,
        max_results=MAX_HIGH_RES_HYPOTHESES,
    )
    return preview, candidates, coarse_ms


def capture_strong_attempt(
    first_preview,
    detector,
    refiner,
    attempt_index,
    tracking_hint=None,
):
    attempt_start = time.ticks_ms()
    result = {
        "attempt_index": attempt_index,
        "preview": first_preview,
        "high_result": None,
        "distance_result": None,
        "coarse_ms": 0,
        "convert_ms": 0,
        "refine_ms": 0,
        "attempt_ms": 0,
        "reject": "",
        "used_tracking": False,
    }
    tracking_seed = tracking_seed_from_result(tracking_hint)
    if tracking_seed is not None:
        # The seed carries position only.  The following 1080p snapshot and
        # edge refinement remain independent measurements of all four sides.
        preview = first_preview
        candidates = (tracking_seed,)
        coarse_ms = 0
        result["used_tracking"] = True
        print(
            "cal coarse scope=TRACK_DIRECT hypotheses=1 seed=TRACK_SEED time=0ms"
        )
    else:
        preview, candidates, coarse_ms = capture_coarse_candidates(
            first_preview,
            detector,
        )
    result["preview"] = preview
    result["coarse_ms"] = coarse_ms
    if not candidates:
        result["reject"] = "COARSE_NONE"
        result["attempt_ms"] = time.ticks_diff(
            time.ticks_ms(), attempt_start
        )
        return result

    gc.collect()
    high_capture = None
    for _ in range(HIGH_RES_FLUSH_FRAMES):
        high_capture = sensor.snapshot(chn=CAPTURE_CH)
    convert_start = time.ticks_ms()
    high_image = high_capture.to_rgb565()
    high_capture = None
    result["convert_ms"] = time.ticks_diff(
        time.ticks_ms(), convert_start
    )
    gc.collect()
    if high_image is None:
        result["reject"] = "CONVERT_NONE"
        result["attempt_ms"] = time.ticks_diff(
            time.ticks_ms(), attempt_start
        )
        return result

    selected_high = None
    selected_rank = -1
    for candidate_index in range(len(candidates)):
        refine_start = time.ticks_ms()
        candidate_high = refiner.refine(
            high_image,
            candidates[candidate_index],
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
        )
        elapsed = time.ticks_diff(time.ticks_ms(), refine_start)
        result["refine_ms"] += elapsed
        if candidate_high is None:
            continue
        candidate_rank = candidate_high.get(
            "measurement_confidence_rank",
            2 if candidate_high.get("measurement_valid", False) else 0,
        )
        if (
            selected_high is None
            or candidate_rank > selected_rank
            or (
                candidate_rank == selected_rank
                and candidate_high["quality_score"]
                > selected_high["quality_score"]
            )
        ):
            selected_high = candidate_high
            selected_rank = candidate_rank
        print(
            "cal refine candidate=%d mode=%s valid=%d reject=%s quality=%.1f demoted=%d ring=%.2f/%.2f time=%dms path=%s"
            % (
                candidate_index + 1,
                candidate_high["mode"],
                1 if candidate_high.get("measurement_valid", False) else 0,
                candidate_high.get("measurement_reject_reason", "UNKNOWN"),
                candidate_high["quality_score"],
                1
                if candidate_high.get("outer_conflict_demoted", False)
                else 0,
                candidate_high.get("ring_inside_pass_ratio", 0.0),
                candidate_high.get("ring_outside_pass_ratio", 0.0),
                elapsed,
                "TRACK" if result["used_tracking"] else "COLD",
            )
        )
        if candidate_high.get("measurement_valid", False):
            break

    high_image = None
    gc.collect()
    result["high_result"] = selected_high
    if selected_high is None:
        result["reject"] = "REFINE_NONE"
    else:
        result["distance_result"] = estimate_distance(selected_high)
        result["reject"] = selected_high.get(
            "measurement_reject_reason", "UNKNOWN"
        )
    result["attempt_ms"] = time.ticks_diff(
        time.ticks_ms(), attempt_start
    )
    return result


def collect_consistent_sample(
    first_preview,
    detector,
    refiner,
    tracking_hint=None,
):
    group_start = time.ticks_ms()
    strong_records = []
    coarse_ms = 0
    convert_ms = 0
    refine_ms = 0
    last_preview = first_preview
    current_tracking_hint = tracking_hint

    for attempt_index in range(1, MAX_MEASUREMENT_ATTEMPTS + 1):
        if attempt_index == 1:
            attempt_preview = first_preview
        else:
            attempt_preview = sensor.snapshot(chn=PREVIEW_CH)
        attempt = capture_strong_attempt(
            attempt_preview,
            detector,
            refiner,
            attempt_index,
            tracking_hint=current_tracking_hint,
        )
        last_preview = attempt["preview"]
        coarse_ms += attempt["coarse_ms"]
        convert_ms += attempt["convert_ms"]
        refine_ms += attempt["refine_ms"]
        high_result = attempt["high_result"]
        distance_result = attempt["distance_result"]
        if distance_result is None or high_result is None:
            print(
                "CAL_ATTEMPT_REJECT index=%d reason=%s"
                % (attempt_index, attempt["reject"])
            )
            if attempt.get("used_tracking", False):
                # Never keep forcing a stale geometry hint.  The next frame
                # returns to the complete CENTER -> CENTER_LOW -> FULL search.
                current_tracking_hint = None
                print("cal tracking hint dropped after rejected refinement")
            continue

        # Only a strict, independently refined 1080p result may update the
        # location hint used by the following frame or sample.
        current_tracking_hint = high_result_to_preview(high_result)

        best_match = None
        best_scale_difference = None
        best_center_shift = None
        for previous in strong_records:
            scale_difference = relative_scale_difference(
                previous["distance_result"]["frame_scale"],
                distance_result["frame_scale"],
            )
            center_shift = relative_center_shift(
                previous["high_result"]["inner_corners_float"],
                high_result["inner_corners_float"],
            )
            print(
                "cal pair=%d/%d scale_delta=%.4f%% center_shift=%.3f%% path=%s/%s"
                % (
                    previous["attempt_index"],
                    attempt_index,
                    scale_difference * 100.0,
                    center_shift * 100.0,
                    (
                        "TRACK"
                        if previous.get("used_tracking", False)
                        else "COLD"
                    ),
                    "TRACK" if attempt.get("used_tracking", False) else "COLD",
                )
            )
            if (
                scale_difference <= MAX_FRAME_SCALE_DISAGREEMENT
                and center_shift <= MAX_FRAME_CENTER_SHIFT_RATIO
                and (
                    best_scale_difference is None
                    or scale_difference < best_scale_difference
                )
            ):
                best_match = previous
                best_scale_difference = scale_difference
                best_center_shift = center_shift

        if best_match is not None:
            fused = fuse_distance_results((
                best_match["distance_result"],
                distance_result,
            ))
            total_ms = time.ticks_diff(time.ticks_ms(), group_start)
            first_high = best_match["high_result"]
            return {
                "frame_scale": fused["frame_scale"],
                "provisional_distance_cm": fused["distance_cm"],
                "component_scale_1": best_match["distance_result"][
                    "frame_scale"
                ],
                "component_scale_2": distance_result["frame_scale"],
                "scale_spread_pct": fused.get(
                    "frame_scale_spread_pct", 0.0
                ),
                "center_shift_ratio": best_center_shift,
                "measurement_attempts": attempt_index,
                "total_ms": total_ms,
                "coarse_ms": coarse_ms,
                "convert_ms": convert_ms,
                "refine_ms": refine_ms,
                "mode_1": first_high["mode"],
                "mode_2": high_result["mode"],
                "demoted_1": 1
                if first_high.get("outer_conflict_demoted", False)
                else 0,
                "demoted_2": 1
                if high_result.get("outer_conflict_demoted", False)
                else 0,
                "inner_rms_1": first_high.get("inner_edge_rms", -1.0),
                "inner_rms_2": high_result.get("inner_edge_rms", -1.0),
                "ring_inside_1": first_high.get(
                    "ring_inside_pass_ratio", 0.0
                ),
                "ring_inside_2": high_result.get(
                    "ring_inside_pass_ratio", 0.0
                ),
                "ring_outside_1": first_high.get(
                    "ring_outside_pass_ratio", 0.0
                ),
                "ring_outside_2": high_result.get(
                    "ring_outside_pass_ratio", 0.0
                ),
            }, last_preview, current_tracking_hint

        strong_records.append({
            "attempt_index": attempt_index,
            "distance_result": distance_result,
            "high_result": high_result,
            "used_tracking": attempt.get("used_tracking", False),
        })

    print(
        "CAL_SAMPLE_FAILED strong=%d attempts=%d total=%dms"
        % (
            len(strong_records),
            MAX_MEASUREMENT_ATTEMPTS,
            time.ticks_diff(time.ticks_ms(), group_start),
        )
    )
    return None, last_preview, current_tracking_hint


def draw_progress(image, current_index, current_count, status_color):
    image.draw_rectangle(4, 4, 12, 12, color=status_color, thickness=-1)
    margin = 22
    gap = 3
    segment_width = (
        PREVIEW_WIDTH - margin * 2 - gap * (len(DISTANCE_POINTS_CM) - 1)
    ) // len(DISTANCE_POINTS_CM)
    y = 8
    for index in range(len(DISTANCE_POINTS_CM)):
        x = margin + index * (segment_width + gap)
        if index < current_index:
            color = (0, 220, 80)
        elif index == current_index:
            color = (255, 210, 0)
        else:
            color = (70, 90, 130)
        image.draw_rectangle(
            x,
            y,
            segment_width,
            10,
            color=color,
            thickness=-1,
        )
    if current_index < len(DISTANCE_POINTS_CM):
        x = margin + current_index * (segment_width + gap)
        unit_width = max(2, (segment_width - 2) // SAMPLES_PER_DISTANCE)
        for sample_index in range(current_count):
            image.draw_rectangle(
                x + 1 + sample_index * unit_width,
                y + 12,
                max(1, unit_width - 1),
                4,
                color=(0, 255, 100),
                thickness=-1,
            )


try:
    ensure_dir(SAVE_DIR)
    initialize_storage()
    scales_by_distance = load_scales_by_distance()
    current_index = first_incomplete_index(scales_by_distance)
    write_summary(scales_by_distance)
    os.exitpoint(os.EXITPOINT_ENABLE)

    uart = YbUart(baudrate=UART_BAUDRATE)
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
    print("Distance calibration collector ready.")
    print("Version:", CALIBRATION_CAPTURE_VERSION)
    print("CSV:", CALIBRATION_CSV)
    print("Summary:", CALIBRATION_SUMMARY)
    print("UART1: 115200 8N1, header pin 3 TXD / pin 4 RXD")
    print("Commands: M or 1=collect current 5-sample point, S=status, P=ping")
    print(
        "points=100..200cm step=5cm count=%d samples=%d batch_attempts=%d scale_gate=%.3f%% center_gate=%.1f%% settle=%dms"
        % (
            len(DISTANCE_POINTS_CM),
            SAMPLES_PER_DISTANCE,
            MAX_BATCH_SAMPLE_ATTEMPTS,
            MAX_FRAME_SCALE_DISAGREEMENT * 100.0,
            MAX_FRAME_CENTER_SHIFT_RATIO * 100.0,
            TRIGGER_SETTLE_MS,
        )
    )
    if current_index < len(DISTANCE_POINTS_CM):
        print(
            "CAL_POINT distance=%.1fcm accepted=%d/%d -- place target, then send one M to fill the point"
            % (
                DISTANCE_POINTS_CM[current_index],
                len(scales_by_distance[current_index]),
                SAMPLES_PER_DISTANCE,
            )
        )
    else:
        print("CALIBRATION_COMPLETE -- existing CSV already has every point.")
    uart_send_line(
        "READY DISTANCE_CALIBRATION VER=%s BAUD=%d POINTS=%d STEP=5 SAMPLES=%d"
        % (
            CALIBRATION_CAPTURE_VERSION,
            UART_BAUDRATE,
            len(DISTANCE_POINTS_CM),
            SAMPLES_PER_DISTANCE,
        )
    )
    send_uart_status(current_index, scales_by_distance)

    status_color = (80, 120, 255)
    frame_count = 0
    clock = time.clock()
    # Preview-space corners only; no image buffer is retained.  The hint is
    # allowed to survive between samples and adjacent 5 cm points.
    tracking_hint = None

    while True:
        os.exitpoint()
        clock.tick()
        preview = sensor.snapshot(chn=PREVIEW_CH)

        command = poll_uart_command()
        if command == "PING":
            uart_send_line("PONG")
        elif command == "STATUS":
            send_uart_status(current_index, scales_by_distance)
        elif command is not None and command.startswith("UNKNOWN:"):
            print("UART_UNKNOWN_COMMAND:", command[8:])
            uart_send_line("ERROR UNKNOWN_COMMAND EXPECTED=M|S|P")
        elif command == "MEASURE":
            if current_index >= len(DISTANCE_POINTS_CM):
                print("CALIBRATION_COMPLETE -- delete/reset CSV for a new run.")
                status_color = (0, 220, 80)
                uart_send_line("COMPLETE")
            else:
                active_distance = DISTANCE_POINTS_CM[current_index]
                starting_count = len(scales_by_distance[current_index])
                status_color = (255, 210, 0)
                uart_send_line(
                    "ACK MEASURE BATCH D=%.1f START=%d/%d MAX_ATTEMPTS=%d"
                    % (
                        active_distance,
                        starting_count,
                        SAMPLES_PER_DISTANCE,
                        MAX_BATCH_SAMPLE_ATTEMPTS,
                    )
                )
                if TRIGGER_SETTLE_MS > 0:
                    time.sleep_ms(TRIGGER_SETTLE_MS)
                print("")
                print(
                    "CAL_BATCH_START distance=%.1fcm accepted=%d/%d trigger=UART max_attempts=%d seed=%s"
                    % (
                        active_distance,
                        starting_count,
                        SAMPLES_PER_DISTANCE,
                        MAX_BATCH_SAMPLE_ATTEMPTS,
                        (
                            "TRACK"
                            if tracking_seed_from_result(tracking_hint)
                            is not None
                            else "COLD"
                        ),
                    )
                )

                batch_attempts = 0
                accepted_this_batch = 0
                while (
                    len(scales_by_distance[current_index])
                    < SAMPLES_PER_DISTANCE
                    and batch_attempts < MAX_BATCH_SAMPLE_ATTEMPTS
                ):
                    batch_attempts += 1
                    next_sample = (
                        len(scales_by_distance[current_index]) + 1
                    )
                    for _ in range(POST_TRIGGER_PREVIEW_FLUSH_FRAMES):
                        preview = sensor.snapshot(chn=PREVIEW_CH)
                    print("")
                    print(
                        "CAL_SAMPLE_START distance=%.1fcm next=%d/%d batch_attempt=%d/%d"
                        % (
                            active_distance,
                            next_sample,
                            SAMPLES_PER_DISTANCE,
                            batch_attempts,
                            MAX_BATCH_SAMPLE_ATTEMPTS,
                        )
                    )
                    try:
                        (
                            record,
                            preview,
                            tracking_hint,
                        ) = collect_consistent_sample(
                            preview,
                            detector,
                            refiner,
                            tracking_hint=tracking_hint,
                        )
                    except Exception as error:
                        record = None
                        tracking_hint = None
                        print("CAL_SAMPLE_ERROR:", repr(error))
                        gc.collect()

                    # Ignore commands queued while this batch owns the camera.
                    # The host sends only one M and waits for the final RESULT.
                    drain_uart_input()
                    if record is None:
                        status_color = (255, 50, 50)
                        print(
                            "CAL_SAMPLE_RETRY distance=%.1fcm count=%d/%d -- automatic retry"
                            % (
                                active_distance,
                                len(scales_by_distance[current_index]),
                                SAMPLES_PER_DISTANCE,
                            )
                        )
                        uart_send_line(
                            "SAMPLE RETRY D=%.1f COUNT=%d/%d ATTEMPT=%d/%d"
                            % (
                                active_distance,
                                len(scales_by_distance[current_index]),
                                SAMPLES_PER_DISTANCE,
                                batch_attempts,
                                MAX_BATCH_SAMPLE_ATTEMPTS,
                            )
                        )
                        gc.collect()
                        continue

                    sample_index = len(scales_by_distance[current_index]) + 1
                    record["distance_cm"] = active_distance
                    record["sample_index"] = sample_index
                    append_record(record)
                    scales_by_distance[current_index].append(
                        record["frame_scale"]
                    )
                    write_summary(scales_by_distance)
                    accepted_this_batch += 1
                    status_color = (0, 220, 80)
                    print(
                        "CAL_SAMPLE_OK distance=%.1fcm sample=%d/%d scale=%.6f spread=%.4f%% center=%.3f%% provisional_D=%.2fcm total=%dms batch_attempt=%d/%d"
                        % (
                            record["distance_cm"],
                            sample_index,
                            SAMPLES_PER_DISTANCE,
                            record["frame_scale"],
                            record["scale_spread_pct"],
                            record["center_shift_ratio"] * 100.0,
                            record["provisional_distance_cm"],
                            record["total_ms"],
                            batch_attempts,
                            MAX_BATCH_SAMPLE_ATTEMPTS,
                        )
                    )
                    uart_send_line(
                        "SAMPLE OK D=%.1f SAMPLE=%d/%d SCALE=%.6f SPREAD=%.4f CENTER=%.3f TIME=%d"
                        % (
                            active_distance,
                            sample_index,
                            SAMPLES_PER_DISTANCE,
                            record["frame_scale"],
                            record["scale_spread_pct"],
                            record["center_shift_ratio"] * 100.0,
                            record["total_ms"],
                        )
                    )
                    draw_progress(
                        preview,
                        current_index,
                        sample_index,
                        status_color,
                    )
                    Display.show_image(
                        preview,
                        x=PREVIEW_X,
                        y=PREVIEW_Y,
                    )
                    gc.collect()

                completed_count = len(scales_by_distance[current_index])
                if completed_count >= SAMPLES_PER_DISTANCE:
                    values = scales_by_distance[current_index]
                    point_median = median(values)
                    point_mad = mad(values)
                    print(
                        "CAL_POINT_DONE distance=%.1fcm median_scale=%.6f mad=%.6f accepted_in_batch=%d attempts=%d"
                        % (
                            active_distance,
                            point_median,
                            point_mad,
                            accepted_this_batch,
                            batch_attempts,
                        )
                    )
                    uart_send_line(
                        "POINT DONE D=%.1f MEDIAN=%.6f MAD=%.6f"
                        % (active_distance, point_median, point_mad)
                    )
                    uart_send_line(
                        "RESULT OK D=%.1f COUNT=%d/%d ACCEPTED=%d ATTEMPTS=%d"
                        % (
                            active_distance,
                            completed_count,
                            SAMPLES_PER_DISTANCE,
                            accepted_this_batch,
                            batch_attempts,
                        )
                    )
                    current_index = first_incomplete_index(
                        scales_by_distance
                    )
                    if current_index < len(DISTANCE_POINTS_CM):
                        print(
                            "MOVE_TARGET distance=%.1fcm, then send one M"
                            % DISTANCE_POINTS_CM[current_index]
                        )
                        uart_send_line(
                            "MOVE D=%.1f"
                            % DISTANCE_POINTS_CM[current_index]
                        )
                    else:
                        print("CALIBRATION_COMPLETE")
                        print(
                            "Copy CSV + summary to PC before editing distance_estimator.py."
                        )
                        uart_send_line("COMPLETE")
                else:
                    status_color = (255, 50, 50)
                    print(
                        "CAL_BATCH_PARTIAL distance=%.1fcm count=%d/%d attempts=%d -- send M to continue"
                        % (
                            active_distance,
                            completed_count,
                            SAMPLES_PER_DISTANCE,
                            batch_attempts,
                        )
                    )
                    uart_send_line(
                        "RESULT RETRY D=%.1f COUNT=%d/%d ATTEMPTS=%d"
                        % (
                            active_distance,
                            completed_count,
                            SAMPLES_PER_DISTANCE,
                            batch_attempts,
                        )
                    )

        current_count = (
            len(scales_by_distance[current_index])
            if current_index < len(DISTANCE_POINTS_CM)
            else SAMPLES_PER_DISTANCE
        )
        draw_progress(preview, current_index, current_count, status_color)
        Display.show_image(preview, x=PREVIEW_X, y=PREVIEW_Y)

        frame_count += 1
        if frame_count % 120 == 0:
            print(
                "preview fps=%.1f point=%s count=%d/%d"
                % (
                    clock.fps(),
                    (
                        "DONE"
                        if current_index >= len(DISTANCE_POINTS_CM)
                        else "%.1fcm" % DISTANCE_POINTS_CM[current_index]
                    ),
                    current_count,
                    SAMPLES_PER_DISTANCE,
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
