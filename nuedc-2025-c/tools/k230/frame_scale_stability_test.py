"""Fixed-scene repeatability test for the 2025-C A4 reference frame.

Usage on the Yahboom K230 / CanMV MicroPython firmware:

1. Put this file, frame_detector.py and high_res_refiner.py in /sdcard.
2. Set KNOWN_DISTANCE_CM below to the measured camera-to-target distance.
3. Keep the camera and A4 target completely fixed.
4. Press the onboard key once.  The script performs SAMPLE_COUNT independent
   measurements without saving JPEG files.
5. Copy the CSV and summary TXT from /data/stability for PC analysis.

The test deliberately follows the same path as the current single-shot
program: three 512x288 coarse searches, one 1920x1080 RGB888 capture converted
to RGB565, then high-resolution refinement of every successful coarse result.
"""

import gc
import math
import os
import time

from media.sensor import *
from media.display import *
from ybUtils.YbKey import YbKey

from frame_detector import FrameDetector
from high_res_refiner import HighResRefiner


# ---------------------------------------------------------------------------
# Test configuration -- change the distance before each fixed-scene batch.
# ---------------------------------------------------------------------------

KNOWN_DISTANCE_CM = 100.0
SAMPLE_COUNT = 30
SAVE_DIR = "/data/stability"

CAPTURE_CH = CAM_CHN_ID_0
CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080

PREVIEW_CH = CAM_CHN_ID_1
PREVIEW_WIDTH = 512
PREVIEW_HEIGHT = 288

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
PREVIEW_X = (DISPLAY_WIDTH - PREVIEW_WIDTH) // 2
PREVIEW_Y = (DISPLAY_HEIGHT - PREVIEW_HEIGHT) // 2
IDE_PREVIEW_QUALITY = 55

RECT_THRESHOLD = 1800
COARSE_MAX_ATTEMPTS = 3
COARSE_HYPOTHESES_PER_ATTEMPT = 4
MAX_HIGH_RES_HYPOTHESES = 6
HIGH_RES_FLUSH_FRAMES = 2

OUTER_WIDTH_CM = 21.0
OUTER_HEIGHT_CM = 29.7
INNER_WIDTH_CM = 17.0
INNER_HEIGHT_CM = 25.7

EXPECTED_SHORT_RATIO = INNER_WIDTH_CM / OUTER_WIDTH_CM
EXPECTED_LONG_RATIO = INNER_HEIGHT_CM / OUTER_HEIGHT_CM
EXPECTED_OUTER_ASPECT = OUTER_HEIGHT_CM / OUTER_WIDTH_CM
EXPECTED_INNER_ASPECT = INNER_HEIGHT_CM / INNER_WIDTH_CM


CSV_FIELDS = (
    "sample",
    "known_distance_cm",
    "valid",
    "pair",
    "localization_valid",
    "mode",
    "error",
    "measurement_reject_reason",
    "measurement_confidence",
    "measurement_confidence_rank",
    "coarse_attempts",
    "coarse_candidates",
    "coarse_score",
    "quality_score",
    "inner_valid_sides",
    "outer_valid_sides",
    "inner_edge_response",
    "outer_edge_response",
    "inner_edge_rms",
    "outer_edge_rms",
    "inner_point_count",
    "outer_point_count",
    "inner_search_radius",
    "outer_search_radius",
    "frame_model_disagreement",
    "detected_frame_disagreement",
    "inner_fill_ratio",
    "outer_fill_ratio",
    "inner_max_opposite_error",
    "outer_max_opposite_error",
    "ring_valid_samples",
    "ring_inside_contrast",
    "ring_outside_contrast",
    "ring_inside_pass_ratio",
    "ring_outside_pass_ratio",
    "ring_min_side_pass_ratio",
    "ring_mean_thickness",
    "outer_top_px",
    "outer_right_px",
    "outer_bottom_px",
    "outer_left_px",
    "outer_width_px",
    "outer_height_px",
    "outer_area_px2",
    "inner_top_px",
    "inner_right_px",
    "inner_bottom_px",
    "inner_left_px",
    "inner_width_px",
    "inner_height_px",
    "inner_area_px2",
    "outer_scale_w",
    "outer_scale_h",
    "outer_scale_area",
    "inner_scale_w",
    "inner_scale_h",
    "inner_scale_area",
    "measurement_scale_inner_geom",
    "effective_scale_geom4",
    "effective_scale_int_geom4",
    "float_int_scale_delta_pct",
    "inner_outer_scale_disagreement",
    "inner_outer_width_ratio_error",
    "inner_outer_height_ratio_error",
    "outer_scale_anisotropy",
    "inner_scale_anisotropy",
    "coarse_ms",
    "high_capture_ms",
    "convert_ms",
    "refine_ms",
    "total_ms",
)


sensor = None
display_inited = False
key = None


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


def distance_tag(distance_cm):
    rounded = int(round(distance_cm))
    if abs(distance_cm - rounded) < 0.001:
        return "%dcm" % rounded
    return ("%.1fcm" % distance_cm).replace(".", "p")


def find_next_batch_id(tag):
    prefix = "stability_%s_" % tag
    maximum = 0
    try:
        filenames = os.listdir(SAVE_DIR)
    except OSError:
        return 1

    for filename in filenames:
        if not filename.startswith(prefix) or not filename.endswith(".csv"):
            continue
        number_text = filename[len(prefix):-4]
        if number_text.isdigit():
            maximum = max(maximum, int(number_text))
    return maximum + 1


def draw_quad(image, corners, color, thickness=1):
    if corners is None:
        return
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        image.draw_line(
            int(round(first[0])),
            int(round(first[1])),
            int(round(second[0])),
            int(round(second[1])),
            color=color,
            thickness=thickness,
        )


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
            high_result.get("outer_corners_float", high_result["outer_corners"]),
            scale_x,
            scale_y,
        ),
        "inner_corners": scale_corners(
            high_result.get("inner_corners_float", high_result["inner_corners"]),
            scale_x,
            scale_y,
        ),
    }


def coarse_result_to_preview(coarse_result):
    if coarse_result is None:
        return None
    return {
        "mode": coarse_result["mode"],
        "outer_corners": coarse_result.get("outer_corners"),
        "inner_corners": coarse_result.get("inner_corners"),
    }


def draw_progress_overlay(image, result, sample_index, valid, running):
    if result is not None:
        draw_quad(image, result.get("outer_corners"), (255, 50, 50), 1)
        draw_quad(image, result.get("inner_corners"), (0, 255, 80), 1)

    if running:
        status_color = (255, 210, 0)
    elif sample_index <= 0:
        status_color = (80, 120, 255)
    elif valid:
        status_color = (0, 255, 80)
    else:
        status_color = (255, 50, 50)
    image.draw_rectangle(4, 4, 12, 12, color=status_color, thickness=-1)

    # Font files are intentionally absent on the minimal SD-card image.  A
    # simple bar gives progress feedback without draw_string()/FreeType.
    bar_x = 4
    bar_y = image.height() - 8
    bar_width = image.width() - 8
    image.draw_rectangle(
        bar_x,
        bar_y,
        bar_width,
        4,
        color=(80, 80, 80),
        thickness=1,
    )
    if sample_index > 0:
        fill_width = int(round(
            (bar_width - 2) * min(sample_index, SAMPLE_COUNT) / SAMPLE_COUNT
        ))
        if fill_width > 0:
            image.draw_rectangle(
                bar_x + 1,
                bar_y + 1,
                fill_width,
                2,
                color=status_color,
                thickness=-1,
            )


def point_distance(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return math.sqrt(dx * dx + dy * dy)


def polygon_area(corners):
    total = 0.0
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        total += first[0] * second[1] - second[0] * first[1]
    return abs(total) * 0.5


def quad_geometry(corners):
    sides = []
    for index in range(4):
        sides.append(point_distance(corners[index], corners[(index + 1) % 4]))
    return {
        "top": sides[0],
        "right": sides[1],
        "bottom": sides[2],
        "left": sides[3],
        "axis_a": (sides[0] + sides[2]) * 0.5,
        "axis_b": (sides[1] + sides[3]) * 0.5,
        "area": polygon_area(corners),
    }


def effective_scale_from_geometry(outer, inner):
    # Determine which visual side pair is the physical 21 cm A4 width, then
    # apply the same orientation to the inner aperture.  This also works when
    # the sheet is rotated 90 degrees in the image.
    if outer["axis_a"] <= outer["axis_b"]:
        outer_width = outer["axis_a"]
        outer_height = outer["axis_b"]
        inner_width = inner["axis_a"]
        inner_height = inner["axis_b"]
    else:
        outer_width = outer["axis_b"]
        outer_height = outer["axis_a"]
        inner_width = inner["axis_b"]
        inner_height = inner["axis_a"]

    outer_scale_w = outer_width / OUTER_WIDTH_CM
    outer_scale_h = outer_height / OUTER_HEIGHT_CM
    inner_scale_w = inner_width / INNER_WIDTH_CM
    inner_scale_h = inner_height / INNER_HEIGHT_CM
    outer_scale_area = math.sqrt(
        outer["area"] / (OUTER_WIDTH_CM * OUTER_HEIGHT_CM)
    )
    inner_scale_area = math.sqrt(
        inner["area"] / (INNER_WIDTH_CM * INNER_HEIGHT_CM)
    )
    effective = (
        outer_scale_w
        * outer_scale_h
        * inner_scale_w
        * inner_scale_h
    ) ** 0.25

    mean_outer_scale = (outer_scale_w + outer_scale_h) * 0.5
    mean_inner_scale = (inner_scale_w + inner_scale_h) * 0.5
    mean_area_scale = (outer_scale_area + inner_scale_area) * 0.5

    return {
        "outer_width": outer_width,
        "outer_height": outer_height,
        "inner_width": inner_width,
        "inner_height": inner_height,
        "outer_scale_w": outer_scale_w,
        "outer_scale_h": outer_scale_h,
        "outer_scale_area": outer_scale_area,
        "inner_scale_w": inner_scale_w,
        "inner_scale_h": inner_scale_h,
        "inner_scale_area": inner_scale_area,
        "effective": effective,
        "scale_disagreement": abs(
            outer_scale_area - inner_scale_area
        ) / max(mean_area_scale, 1e-9),
        "width_ratio_error": abs(
            (inner_width / max(outer_width, 1e-9))
            / EXPECTED_SHORT_RATIO
            - 1.0
        ),
        "height_ratio_error": abs(
            (inner_height / max(outer_height, 1e-9))
            / EXPECTED_LONG_RATIO
            - 1.0
        ),
        "outer_anisotropy": abs(
            outer_scale_w - outer_scale_h
        ) / max(mean_outer_scale, 1e-9),
        "inner_anisotropy": abs(
            inner_scale_w - inner_scale_h
        ) / max(mean_inner_scale, 1e-9),
    }


def add_geometry_to_row(row, high_result):
    outer_float = high_result.get(
        "outer_corners_float", high_result["outer_corners"]
    )
    inner_float = high_result.get(
        "inner_corners_float", high_result["inner_corners"]
    )
    outer = quad_geometry(outer_float)
    inner = quad_geometry(inner_float)
    scales = effective_scale_from_geometry(outer, inner)

    row["outer_top_px"] = outer["top"]
    row["outer_right_px"] = outer["right"]
    row["outer_bottom_px"] = outer["bottom"]
    row["outer_left_px"] = outer["left"]
    row["outer_width_px"] = scales["outer_width"]
    row["outer_height_px"] = scales["outer_height"]
    row["outer_area_px2"] = outer["area"]

    row["inner_top_px"] = inner["top"]
    row["inner_right_px"] = inner["right"]
    row["inner_bottom_px"] = inner["bottom"]
    row["inner_left_px"] = inner["left"]
    row["inner_width_px"] = scales["inner_width"]
    row["inner_height_px"] = scales["inner_height"]
    row["inner_area_px2"] = inner["area"]

    row["outer_scale_w"] = scales["outer_scale_w"]
    row["outer_scale_h"] = scales["outer_scale_h"]
    row["outer_scale_area"] = scales["outer_scale_area"]
    row["inner_scale_w"] = scales["inner_scale_w"]
    row["inner_scale_h"] = scales["inner_scale_h"]
    row["inner_scale_area"] = scales["inner_scale_area"]
    row["effective_scale_geom4"] = scales["effective"]
    row["inner_outer_scale_disagreement"] = scales["scale_disagreement"]
    row["inner_outer_width_ratio_error"] = scales["width_ratio_error"]
    row["inner_outer_height_ratio_error"] = scales["height_ratio_error"]
    row["outer_scale_anisotropy"] = scales["outer_anisotropy"]
    row["inner_scale_anisotropy"] = scales["inner_anisotropy"]

    outer_int = quad_geometry(high_result["outer_corners"])
    inner_int = quad_geometry(high_result["inner_corners"])
    int_scales = effective_scale_from_geometry(outer_int, inner_int)
    row["effective_scale_int_geom4"] = int_scales["effective"]
    row["float_int_scale_delta_pct"] = (
        abs(int_scales["effective"] - scales["effective"])
        / max(scales["effective"], 1e-9)
        * 100.0
    )


def new_row(sample_index):
    return {
        "sample": sample_index,
        "known_distance_cm": KNOWN_DISTANCE_CM,
        "valid": 0,
        "pair": 0,
        "localization_valid": 0,
        "mode": "NOT_RUN",
        "error": "",
        "measurement_reject_reason": "NOT_RUN",
        "measurement_confidence": "REJECT",
        "measurement_confidence_rank": 0,
        "coarse_attempts": COARSE_MAX_ATTEMPTS,
        "coarse_candidates": 0,
        "coarse_ms": 0,
        "high_capture_ms": 0,
        "convert_ms": 0,
        "refine_ms": 0,
        "total_ms": 0,
    }


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return "%.6f" % value
    text = str(value)
    if "," in text or '"' in text or "\n" in text:
        text = '"' + text.replace('"', '""') + '"'
    return text


def create_csv(path):
    with open(path, "w") as file:
        file.write(",".join(CSV_FIELDS))
        file.write("\n")


def append_csv_row(path, row):
    values = []
    for field in CSV_FIELDS:
        values.append(csv_value(row.get(field)))
    with open(path, "a") as file:
        file.write(",".join(values))
        file.write("\n")


def measure_sample(sample_index, first_preview, detector, refiner):
    row = new_row(sample_index)
    total_start = time.ticks_ms()
    preview = first_preview
    preview_result = None
    high_capture = None
    high_image = None

    try:
        best_coarse = None
        coarse_results = []
        for attempt in range(COARSE_MAX_ATTEMPTS):
            os.exitpoint()
            if attempt > 0:
                preview = sensor.snapshot(chn=PREVIEW_CH)
            coarse_start = time.ticks_ms()
            attempt_results = detector.detect_hypotheses(
                preview,
                roi=None,
                max_results=COARSE_HYPOTHESES_PER_ATTEMPT,
            )
            row["coarse_ms"] += time.ticks_diff(
                time.ticks_ms(), coarse_start
            )
            for candidate in attempt_results:
                coarse_results.append(candidate)

        coarse_results = detector.select_distinct_hypotheses(
            coarse_results,
            max_results=MAX_HIGH_RES_HYPOTHESES,
        )
        for candidate in coarse_results:
            if (
                best_coarse is None
                or candidate["score"] > best_coarse["score"]
            ):
                best_coarse = candidate

        row["coarse_candidates"] = len(coarse_results)
        if best_coarse is None:
            row["mode"] = "COARSE_NONE"
            row["measurement_reject_reason"] = "COARSE_NONE"
            row["error"] = "coarse frame not found"
            row["total_ms"] = time.ticks_diff(time.ticks_ms(), total_start)
            return row, None, preview

        row["coarse_score"] = best_coarse["score"]
        # A coarse seed is only a search hypothesis.  Never draw it as a
        # locked result before the 1080p identity gates accept it.
        preview_result = None

        gc.collect()
        capture_start = time.ticks_ms()
        for _ in range(HIGH_RES_FLUSH_FRAMES):
            high_capture = sensor.snapshot(chn=CAPTURE_CH)
        row["high_capture_ms"] = time.ticks_diff(
            time.ticks_ms(), capture_start
        )

        convert_start = time.ticks_ms()
        high_image = high_capture.to_rgb565()
        high_capture = None
        row["convert_ms"] = time.ticks_diff(
            time.ticks_ms(), convert_start
        )
        gc.collect()
        if high_image is None:
            row["mode"] = "CONVERT_NONE"
            row["measurement_reject_reason"] = "CONVERT_NONE"
            row["error"] = "RGB888 to RGB565 conversion returned None"
            row["total_ms"] = time.ticks_diff(time.ticks_ms(), total_start)
            return row, preview_result, preview

        high_result = None
        selected_coarse = best_coarse
        for candidate in coarse_results:
            refine_start = time.ticks_ms()
            candidate_high = refiner.refine(
                high_image,
                candidate,
                PREVIEW_WIDTH,
                PREVIEW_HEIGHT,
            )
            row["refine_ms"] += time.ticks_diff(
                time.ticks_ms(), refine_start
            )
            if candidate_high is None:
                continue
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
                selected_coarse = candidate

        if high_result is None:
            row["mode"] = "REFINE_NONE"
            row["measurement_reject_reason"] = "REFINE_NONE"
            row["error"] = "high-resolution refinement returned None"
            row["total_ms"] = time.ticks_diff(time.ticks_ms(), total_start)
            return row, preview_result, preview

        row["coarse_score"] = selected_coarse["score"]
        row["mode"] = high_result["mode"]
        row["pair"] = 1 if high_result["mode"] == "HIGH_RES_PAIR" else 0
        row["localization_valid"] = (
            1 if high_result.get("localization_valid", False) else 0
        )
        row["measurement_reject_reason"] = high_result.get(
            "measurement_reject_reason", "UNKNOWN"
        )
        row["measurement_confidence"] = high_result.get(
            "measurement_confidence", "UNKNOWN"
        )
        row["measurement_confidence_rank"] = high_result.get(
            "measurement_confidence_rank", 0
        )
        row["quality_score"] = high_result["quality_score"]
        row["inner_valid_sides"] = high_result["inner_valid_sides"]
        row["outer_valid_sides"] = high_result["outer_valid_sides"]
        row["inner_edge_response"] = high_result["inner_edge_response"]
        row["outer_edge_response"] = high_result["outer_edge_response"]
        row["inner_edge_rms"] = high_result.get("inner_edge_rms", -1.0)
        row["outer_edge_rms"] = high_result.get("outer_edge_rms", -1.0)
        row["inner_point_count"] = high_result["inner_point_count"]
        row["outer_point_count"] = high_result["outer_point_count"]
        row["inner_search_radius"] = high_result["inner_search_radius"]
        row["outer_search_radius"] = high_result["outer_search_radius"]
        row["frame_model_disagreement"] = high_result[
            "frame_model_disagreement"
        ]
        row["detected_frame_disagreement"] = high_result.get(
            "detected_frame_disagreement",
            high_result["frame_model_disagreement"],
        )
        row["inner_fill_ratio"] = high_result.get("inner_fill_ratio")
        row["outer_fill_ratio"] = high_result.get("outer_fill_ratio")
        row["inner_max_opposite_error"] = high_result.get(
            "inner_max_opposite_error"
        )
        row["outer_max_opposite_error"] = high_result.get(
            "outer_max_opposite_error"
        )
        row["ring_valid_samples"] = high_result.get("ring_valid_samples")
        row["ring_inside_contrast"] = high_result.get(
            "ring_inside_contrast"
        )
        row["ring_outside_contrast"] = high_result.get(
            "ring_outside_contrast"
        )
        row["ring_inside_pass_ratio"] = high_result.get(
            "ring_inside_pass_ratio"
        )
        row["ring_outside_pass_ratio"] = high_result.get(
            "ring_outside_pass_ratio"
        )
        row["ring_min_side_pass_ratio"] = high_result.get(
            "ring_min_side_pass_ratio"
        )
        row["ring_mean_thickness"] = high_result.get(
            "ring_mean_thickness"
        )

        add_geometry_to_row(row, high_result)
        row["measurement_scale_inner_geom"] = math.sqrt(
            row["inner_scale_w"] * row["inner_scale_h"]
        )
        row["valid"] = (
            1
            if high_result["measurement_valid"]
            and row.get("measurement_scale_inner_geom") is not None
            else 0
        )
        preview_result = (
            high_result_to_preview(high_result)
            if high_result["measurement_valid"]
            else None
        )
        row["total_ms"] = time.ticks_diff(time.ticks_ms(), total_start)
        return row, preview_result, preview

    finally:
        high_capture = None
        high_image = None
        gc.collect()


def calculate_stats(values):
    count = len(values)
    if count == 0:
        return None
    total = 0.0
    minimum = values[0]
    maximum = values[0]
    for value in values:
        total += value
        minimum = min(minimum, value)
        maximum = max(maximum, value)
    mean = total / count

    square_total = 0.0
    for value in values:
        delta = value - mean
        square_total += delta * delta
    standard_deviation = 0.0
    if count > 1:
        standard_deviation = math.sqrt(square_total / (count - 1))
    cv_pct = standard_deviation / max(abs(mean), 1e-9) * 100.0
    peak_to_peak_pct = (maximum - minimum) / max(abs(mean), 1e-9) * 100.0
    return {
        "count": count,
        "mean": mean,
        "std": standard_deviation,
        "cv_pct": cv_pct,
        "min": minimum,
        "max": maximum,
        "peak_to_peak_pct": peak_to_peak_pct,
    }


def values_from_rows(rows, field, nonnegative=False):
    values = []
    for row in rows:
        if not row.get("valid"):
            continue
        value = row.get(field)
        if value is None:
            continue
        if nonnegative and value < 0.0:
            continue
        values.append(float(value))
    return values


def write_stats_line(file, label, stats):
    if stats is None:
        file.write("%s: count=0\n" % label)
        return
    file.write(
        "%s: count=%d mean=%.6f std=%.6f CV=%.4f%% min=%.6f max=%.6f peak_to_peak=%.4f%%\n"
        % (
            label,
            stats["count"],
            stats["mean"],
            stats["std"],
            stats["cv_pct"],
            stats["min"],
            stats["max"],
            stats["peak_to_peak_pct"],
        )
    )


def write_summary(path, rows, csv_path, total_batch_ms):
    valid_count = 0
    pair_count = 0
    fallback_count = 0
    mode_counts = {}
    reject_counts = {}
    confidence_counts = {}
    for row in rows:
        valid_count += int(row.get("valid", 0))
        pair_count += int(row.get("pair", 0))
        mode = row.get("mode", "UNKNOWN")
        mode_counts[mode] = mode_counts.get(mode, 0) + 1
        reject_reason = row.get("measurement_reject_reason", "UNKNOWN")
        reject_counts[reject_reason] = reject_counts.get(reject_reason, 0) + 1
        confidence = row.get("measurement_confidence", "UNKNOWN")
        confidence_counts[confidence] = confidence_counts.get(confidence, 0) + 1
        if mode == "HIGH_RES_FALLBACK":
            fallback_count += 1

    measurement_scale_stats = calculate_stats(
        values_from_rows(rows, "measurement_scale_inner_geom")
    )
    grade = "FAIL"
    if (
        measurement_scale_stats is not None
        and valid_count == SAMPLE_COUNT
        and fallback_count == 0
        and measurement_scale_stats["cv_pct"] <= 0.5
        and measurement_scale_stats["peak_to_peak_pct"] <= 1.0
    ):
        grade = "PASS"
        if (
            measurement_scale_stats["cv_pct"] <= 0.3
            and measurement_scale_stats["peak_to_peak_pct"] <= 0.8
        ):
            grade = "EXCELLENT"

    with open(path, "w") as file:
        file.write("K230 A4 frame scale stability test\n")
        file.write("known_distance_cm=%.3f\n" % KNOWN_DISTANCE_CM)
        file.write("requested_samples=%d\n" % SAMPLE_COUNT)
        file.write("valid_samples=%d\n" % valid_count)
        file.write("high_res_pair_samples=%d\n" % pair_count)
        file.write("high_res_fallback_samples=%d\n" % fallback_count)
        file.write("batch_total_ms=%d\n" % total_batch_ms)
        file.write("csv=%s\n" % csv_path)
        file.write("mode_counts=")
        first = True
        for mode in sorted(mode_counts.keys()):
            if not first:
                file.write(",")
            file.write("%s:%d" % (mode, mode_counts[mode]))
            first = False
        file.write("\n\n")
        file.write("confidence_counts=")
        first = True
        for confidence in sorted(confidence_counts.keys()):
            if not first:
                file.write(",")
            file.write(
                "%s:%d" % (confidence, confidence_counts[confidence])
            )
            first = False
        file.write("\n\n")
        file.write("reject_counts=")
        first = True
        for reason in sorted(reject_counts.keys()):
            if not first:
                file.write(",")
            file.write("%s:%d" % (reason, reject_counts[reason]))
            first = False
        file.write("\n\n")

        file.write("Definitions:\n")
        file.write("  scale unit: pixels per centimetre\n")
        file.write("  CV = sample standard deviation / mean\n")
        file.write("  ratio_error and disagreement are fractional, not percent\n")
        file.write("  edge_rms unit: high-resolution pixels\n\n")

        write_stats_line(
            file,
            "measurement_scale_inner_geom",
            measurement_scale_stats,
        )
        write_stats_line(
            file,
            "effective_scale_geom4_diagnostic",
            calculate_stats(values_from_rows(rows, "effective_scale_geom4")),
        )
        write_stats_line(
            file,
            "effective_scale_int_geom4",
            calculate_stats(values_from_rows(rows, "effective_scale_int_geom4")),
        )
        write_stats_line(
            file,
            "outer_scale_area",
            calculate_stats(values_from_rows(rows, "outer_scale_area")),
        )
        write_stats_line(
            file,
            "inner_scale_area",
            calculate_stats(values_from_rows(rows, "inner_scale_area")),
        )
        write_stats_line(
            file,
            "outer_width_px",
            calculate_stats(values_from_rows(rows, "outer_width_px")),
        )
        write_stats_line(
            file,
            "outer_height_px",
            calculate_stats(values_from_rows(rows, "outer_height_px")),
        )
        write_stats_line(
            file,
            "inner_width_px",
            calculate_stats(values_from_rows(rows, "inner_width_px")),
        )
        write_stats_line(
            file,
            "inner_height_px",
            calculate_stats(values_from_rows(rows, "inner_height_px")),
        )
        write_stats_line(
            file,
            "inner_edge_rms",
            calculate_stats(values_from_rows(rows, "inner_edge_rms", True)),
        )
        write_stats_line(
            file,
            "ring_inside_contrast",
            calculate_stats(values_from_rows(rows, "ring_inside_contrast")),
        )
        write_stats_line(
            file,
            "ring_inside_pass_ratio",
            calculate_stats(values_from_rows(rows, "ring_inside_pass_ratio")),
        )
        write_stats_line(
            file,
            "ring_min_side_pass_ratio",
            calculate_stats(values_from_rows(rows, "ring_min_side_pass_ratio")),
        )
        write_stats_line(
            file,
            "outer_edge_rms",
            calculate_stats(values_from_rows(rows, "outer_edge_rms", True)),
        )
        write_stats_line(
            file,
            "inner_outer_scale_disagreement",
            calculate_stats(
                values_from_rows(rows, "inner_outer_scale_disagreement")
            ),
        )
        write_stats_line(
            file,
            "float_int_scale_delta_pct",
            calculate_stats(values_from_rows(rows, "float_int_scale_delta_pct")),
        )
        write_stats_line(
            file,
            "total_ms",
            calculate_stats(values_from_rows(rows, "total_ms")),
        )

        file.write("\n")
        if measurement_scale_stats is not None:
            distance_std_cm = (
                KNOWN_DISTANCE_CM
                * measurement_scale_stats["cv_pct"]
                / 100.0
            )
            file.write(
                "distance_equivalent_std_cm=%.4f\n" % distance_std_cm
            )
        else:
            file.write("distance_equivalent_std_cm=NONE\n")
        file.write(
            "criterion=all samples valid; inner quality gate passed; inner-width-height-geometric-scale CV <= 0.5%; peak-to-peak <= 1.0%\n"
        )
        file.write("stability_grade=%s\n" % grade)

    return (
        grade,
        measurement_scale_stats,
        valid_count,
        pair_count,
        fallback_count,
    )


def run_batch(detector, refiner):
    tag = distance_tag(KNOWN_DISTANCE_CM)
    batch_id = find_next_batch_id(tag)
    base_name = "stability_%s_%03d" % (tag, batch_id)
    csv_path = "%s/%s.csv" % (SAVE_DIR, base_name)
    summary_path = "%s/%s_summary.txt" % (SAVE_DIR, base_name)
    create_csv(csv_path)

    rows = []
    last_preview_result = None
    last_valid = False
    batch_start = time.ticks_ms()

    print("")
    print(
        "STABILITY_START distance=%.1fcm samples=%d id=%03d"
        % (KNOWN_DISTANCE_CM, SAMPLE_COUNT, batch_id)
    )
    print("Keep both camera and target completely still.")

    for sample_index in range(1, SAMPLE_COUNT + 1):
        os.exitpoint()
        sample_start = time.ticks_ms()
        preview = sensor.snapshot(chn=PREVIEW_CH)
        try:
            row, preview_result, result_preview = measure_sample(
                sample_index,
                preview,
                detector,
                refiner,
            )
        except Exception as error:
            row = new_row(sample_index)
            row["mode"] = "EXCEPTION"
            row["error"] = repr(error)
            row["measurement_reject_reason"] = "EXCEPTION"
            row["total_ms"] = time.ticks_diff(
                time.ticks_ms(), sample_start
            )
            preview_result = None
            result_preview = preview
            print("SAMPLE_ERROR %02d: %s" % (sample_index, repr(error)))
            gc.collect()

        rows.append(row)
        append_csv_row(csv_path, row)
        if preview_result is not None:
            last_preview_result = preview_result
        last_valid = bool(row.get("valid"))

        draw_progress_overlay(
            result_preview,
            preview_result,
            sample_index,
            last_valid,
            True,
        )
        Display.show_image(result_preview, x=PREVIEW_X, y=PREVIEW_Y)

        scale = row.get("measurement_scale_inner_geom")
        scale_text = "NONE" if scale is None else "%.6f" % scale
        print(
            "sample=%02d/%02d valid=%d confidence=%s mode=%s reject=%s pair=%d coarse=%d inner_geom_scale=%s rms=%.3f/%.3f ring=%.2f/%.2f time=%dms"
            % (
                sample_index,
                SAMPLE_COUNT,
                row.get("valid", 0),
                row.get("measurement_confidence", "UNKNOWN"),
                row.get("mode", "UNKNOWN"),
                row.get("measurement_reject_reason", "UNKNOWN"),
                row.get("pair", 0),
                row.get("coarse_candidates", 0),
                scale_text,
                row.get("inner_edge_rms", -1.0),
                row.get("outer_edge_rms", -1.0),
                row.get("ring_inside_pass_ratio", 0.0),
                row.get("ring_min_side_pass_ratio", 0.0),
                row.get("total_ms", 0),
            )
        )
        gc.collect()

    total_batch_ms = time.ticks_diff(time.ticks_ms(), batch_start)
    (
        grade,
        measurement_scale_stats,
        valid_count,
        pair_count,
        fallback_count,
    ) = write_summary(summary_path, rows, csv_path, total_batch_ms)

    print("")
    print(
        "STABILITY_DONE grade=%s valid=%d/%d pair=%d fallback=%d total=%dms"
        % (
            grade,
            valid_count,
            SAMPLE_COUNT,
            pair_count,
            fallback_count,
            total_batch_ms,
        )
    )
    if measurement_scale_stats is not None:
        print(
            "inner_geom_scale mean=%.6f std=%.6f CV=%.4f%% peak_to_peak=%.4f%% distance_equivalent_std=%.4fcm"
            % (
                measurement_scale_stats["mean"],
                measurement_scale_stats["std"],
                measurement_scale_stats["cv_pct"],
                measurement_scale_stats["peak_to_peak_pct"],
                KNOWN_DISTANCE_CM
                * measurement_scale_stats["cv_pct"]
                / 100.0,
            )
        )
    print("CSV:", csv_path)
    print("Summary:", summary_path)
    print("Press KEY to run another independent batch.")
    return last_preview_result, grade != "FAIL"


try:
    ensure_dir(SAVE_DIR)
    os.exitpoint(os.EXITPOINT_ENABLE)

    key = YbKey()
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
    print("Frame-scale stability test ready.")
    print("Known distance: %.1f cm" % KNOWN_DISTANCE_CM)
    print("Samples per key press:", SAMPLE_COUNT)
    print(
        "Preview/coarse: %dx%d RGB565, refinement: %dx%d RGB888 -> RGB565"
        % (PREVIEW_WIDTH, PREVIEW_HEIGHT, CAPTURE_WIDTH, CAPTURE_HEIGHT)
    )
    print("No JPEG files will be saved.")
    print("Press onboard KEY once to begin the batch.")

    locked_preview_result = None
    locked_valid = False
    locked_sample_count = 0
    frame_count = 0
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()
        preview = sensor.snapshot(chn=PREVIEW_CH)

        if key.is_pressed():
            time.sleep_ms(30)
            if key.is_pressed():
                # Wait for release before measuring so finger pressure and
                # table vibration are not included in sample 1.
                while key.is_pressed():
                    os.exitpoint()
                    time.sleep_ms(20)
                time.sleep_ms(150)
                try:
                    locked_preview_result, locked_valid = run_batch(
                        detector, refiner
                    )
                    locked_sample_count = SAMPLE_COUNT
                except Exception as error:
                    print("BATCH_ERROR:", repr(error))
                    locked_preview_result = None
                    locked_valid = False
                    locked_sample_count = SAMPLE_COUNT
                    gc.collect()
                preview = sensor.snapshot(chn=PREVIEW_CH)

        draw_progress_overlay(
            preview,
            locked_preview_result,
            locked_sample_count,
            locked_valid,
            False,
        )
        Display.show_image(preview, x=PREVIEW_X, y=PREVIEW_Y)

        frame_count += 1
        if frame_count % 120 == 0:
            print(
                "preview fps=%.1f state=%s"
                % (
                    clock.fps(),
                    "BATCH_DONE" if locked_sample_count > 0 else "IDLE",
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
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("Camera stopped.")
