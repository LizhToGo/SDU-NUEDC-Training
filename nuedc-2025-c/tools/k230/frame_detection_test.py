"""Live nested-frame detection test for Yahboom K230 + GC2093.

Deploy this file and frame_detector.py to the same directory on the board,
then run this file from CanMV IDE.

Preview legend:
    red: outer frame
    green: inner frame boundary
    orange: inner-only fallback (not a complete outer/inner lock)
    top-left square: green=PAIR, cyan=MODEL_PAIR, orange=INNER_ONLY,
                     red=SEARCH/ERROR

Press the onboard KEY to save the annotated preview and a 1920x1080 JPG
under /data/captures.
"""

import gc
import os
import time

from media.sensor import *
from media.display import *
from ybUtils.YbKey import YbKey

from frame_detector import FrameDetector


# ------------------------------------------------------------------
# Camera and display configuration
# ------------------------------------------------------------------

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

# This only controls the JPEG stream sent to CanMV IDE.  It does not change
# the image used by the detector, so a moderate value saves encode/USB time
# without reducing recognition accuracy.
IDE_PREVIEW_QUALITY = 55
JPG_QUALITY = 95
DEBUG_JPG_QUALITY = 90

# Lower this value if the far target produces no rectangle candidates.
# Raise it if find_rects returns too many noisy candidates.
RECT_THRESHOLD = 2200
# Tracking happens inside a much smaller, already constrained area.  Lowering
# only its threshold makes the lock far less likely to fall through to a
# second expensive full-frame search.
ROI_RECT_THRESHOLD_SCALE = 0.60

# Keep this True for the first tuning run.  A full lock is shown as PAIR;
# INNER_ONLY means only the reliable black/white inner boundary was found.
ALLOW_INNER_ONLY = True
SHOW_ALL_CANDIDATES = False
SHOW_CORNER_LABELS = False
SHOW_TRACKING_ROI = False

# find_rects is the expensive operation.  Once locked, run it in a compact
# tracking ROI and only on every second preview frame.
DETECT_EVERY_N_FRAMES = 2
GLOBAL_SEARCH_EVERY_N_DETECTIONS = 60
# Keep drawing the last valid frame while a few detections are missed.  The
# old value of 2 made a perfectly valid lock disappear after only two noisy
# find_rects calls.
MAX_TRACK_MISSES = 4
LOCK_SMOOTHING_ALPHA = 0.35
# Smoothing is useful for 1--3 pixel corner jitter, but it must not turn a
# real target jump into several slow intermediate boxes.  Snap immediately
# when the inner-frame centre moves by this fraction of its diagonal.
LOCK_SNAP_DISTANCE_RATIO = 0.22
LOCK_SNAP_MIN_PIXELS = 14
STATUS_PRINT_EVERY_N_FRAMES = 60


sensor = None
key = None
display_inited = False
photo_id = 0


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
        if number_text.isdigit():
            number = int(number_text)
            if number > max_id:
                max_id = number
    return max_id


def draw_quad(image, corners, color, thickness=1):
    if corners is None:
        return
    for i in range(4):
        p0 = corners[i]
        p1 = corners[(i + 1) % 4]
        image.draw_line(
            int(p0[0]),
            int(p0[1]),
            int(p1[0]),
            int(p1[1]),
            color=color,
            thickness=thickness,
        )


def draw_corner_labels(image, corners, prefix, color):
    if corners is None:
        return
    for i in range(4):
        x = int(corners[i][0])
        y = int(corners[i][1])
        image.draw_circle(x, y, 2, color=color, thickness=1)


def blend_corners(previous, current, alpha):
    """Low-pass filter four corners so the locked frame does not jitter."""
    if current is None:
        return previous
    if previous is None:
        return current

    blended = []
    keep = 1.0 - alpha
    for i in range(4):
        x = previous[i][0] * keep + current[i][0] * alpha
        y = previous[i][1] * keep + current[i][1] * alpha
        blended.append((int(round(x)), int(round(y))))
    return tuple(blended)


def corners_bbox(corners):
    if corners is None:
        return None
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    x0 = min(xs)
    y0 = min(ys)
    return (x0, y0, max(xs) - x0 + 1, max(ys) - y0 + 1)


def should_snap_lock(previous, current):
    """Return True when the new target position is a real displacement."""
    if previous is None or current is None:
        return True

    old_corners = previous["inner_corners"]
    new_corners = current["inner_corners"]
    old_bbox = corners_bbox(old_corners)
    new_bbox = corners_bbox(new_corners)

    old_cx = old_bbox[0] + old_bbox[2] * 0.5
    old_cy = old_bbox[1] + old_bbox[3] * 0.5
    new_cx = new_bbox[0] + new_bbox[2] * 0.5
    new_cy = new_bbox[1] + new_bbox[3] * 0.5
    dx = new_cx - old_cx
    dy = new_cy - old_cy
    distance_squared = dx * dx + dy * dy

    diagonal_squared = (
        old_bbox[2] * old_bbox[2] + old_bbox[3] * old_bbox[3]
    )
    ratio_limit_squared = (
        diagonal_squared
        * LOCK_SNAP_DISTANCE_RATIO
        * LOCK_SNAP_DISTANCE_RATIO
    )
    pixel_limit_squared = LOCK_SNAP_MIN_PIXELS * LOCK_SNAP_MIN_PIXELS
    limit_squared = max(pixel_limit_squared, ratio_limit_squared)
    return distance_squared >= limit_squared


def smooth_locked_result(previous, current):
    """Merge a new detection into the current temporal lock."""
    if previous is None or should_snap_lock(previous, current):
        return current

    filtered = current.copy()
    old_inner = previous["inner_corners"]
    new_inner = current["inner_corners"]
    filtered_inner = blend_corners(
        old_inner, new_inner, LOCK_SMOOTHING_ALPHA
    )
    filtered["inner_corners"] = filtered_inner
    filtered["inner_bbox"] = corners_bbox(filtered_inner)

    old_outer = previous["outer_corners"]
    new_outer = current["outer_corners"]
    if new_outer is not None:
        filtered_outer = blend_corners(
            old_outer, new_outer, LOCK_SMOOTHING_ALPHA
        )
    elif old_outer is not None:
        # The inner rectangle is still visible but one outer-edge refinement
        # failed.  Move the previous outer corners by the corresponding inner
        # corner displacement instead of dropping the full lock immediately.
        carried = []
        for i in range(4):
            dx = filtered_inner[i][0] - old_inner[i][0]
            dy = filtered_inner[i][1] - old_inner[i][1]
            carried.append(
                (old_outer[i][0] + dx, old_outer[i][1] + dy)
            )
        filtered_outer = tuple(carried)
        filtered["mode"] = "MODEL_PAIR"
        filtered["outer_valid_sides"] = previous["outer_valid_sides"]
        filtered["outer_edge_response"] = previous[
            "outer_edge_response"
        ]
    else:
        filtered_outer = None

    filtered["outer_corners"] = filtered_outer
    filtered["outer_bbox"] = corners_bbox(filtered_outer)
    return filtered


def make_tracking_roi(result, image_width, image_height):
    corners = result["outer_corners"]
    if corners is None:
        corners = result["inner_corners"]

    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    x0 = min(xs)
    x1 = max(xs)
    y0 = min(ys)
    y1 = max(ys)
    width = x1 - x0 + 1
    height = y1 - y0 + 1

    # A slightly generous ROI lets find_rects keep seeing all four edges when
    # the camera or target moves between two expensive detection calls.
    margin_x = max(32, int(width * 0.28))
    margin_y = max(32, int(height * 0.28))
    x0 = max(0, int(x0) - margin_x)
    y0 = max(0, int(y0) - margin_y)
    x1 = min(image_width - 1, int(x1) + margin_x)
    y1 = min(image_height - 1, int(y1) + margin_y)

    # Align the complete ROI to 8-pixel boundaries.  This is friendlier to
    # the K230 image backend than only aligning its top-left coordinate.
    alignment_mask = 7
    x0 = int(x0) & ~alignment_mask
    y0 = int(y0) & ~alignment_mask
    x1_exclusive = min(
        image_width,
        (int(x1) + 1 + alignment_mask) & ~alignment_mask,
    )
    y1_exclusive = min(
        image_height,
        (int(y1) + 1 + alignment_mask) & ~alignment_mask,
    )
    roi_width = max(16, x1_exclusive - x0)
    roi_height = max(16, y1_exclusive - y0)
    if x0 + roi_width > image_width:
        roi_width = image_width - x0
    if y0 + roi_height > image_height:
        roi_height = image_height - y0
    return (x0, y0, roi_width, roi_height)


def draw_overlay(
    image,
    detector,
    result,
    detect_ms,
    fps,
    tracking_roi=None,
    error_text=None,
):
    if SHOW_ALL_CANDIDATES:
        for candidate in detector.last_candidates:
            draw_quad(image, candidate["corners"], (255, 190, 0), 1)

    if SHOW_TRACKING_ROI and tracking_roi is not None:
        image.draw_rectangle(
            tracking_roi,
            color=(0, 180, 255),
            thickness=1,
        )

    status_color = (255, 60, 60)

    if result is not None:
        if result["mode"] in ("PAIR", "MODEL_PAIR"):
            status_color = (0, 255, 80)
            draw_quad(image, result["outer_corners"], (255, 60, 60), 1)
            draw_quad(image, result["inner_corners"], (0, 255, 80), 1)
            if SHOW_CORNER_LABELS:
                draw_corner_labels(
                    image, result["outer_corners"], "O", (255, 60, 60)
                )
                draw_corner_labels(
                    image, result["inner_corners"], "I", (0, 255, 80)
                )
            if result["mode"] == "MODEL_PAIR":
                status_color = (0, 220, 255)
        else:
            status_color = (255, 170, 0)
            draw_quad(image, result["inner_corners"], status_color, 1)
            if SHOW_CORNER_LABELS:
                draw_corner_labels(
                    image, result["inner_corners"], "I", status_color
                )

    if error_text is not None:
        status_color = (255, 60, 60)

    # The minimal SD-card layout deliberately contains no FreeType font.
    # A small color indicator replaces all text drawing and avoids both the
    # FreeType dependency and the old draw_string deprecation flood.
    image.draw_rectangle(
        4, 4, 12, 12, color=status_color, thickness=-1
    )
    if detector.last_roi is not None:
        image.draw_rectangle(
            19, 4, 4, 12, color=(0, 180, 255), thickness=-1
        )


def save_capture(preview_image, result):
    global photo_id
    photo_id += 1

    print("")
    print("Saving capture:", photo_id)

    debug_path = "%s/frame_%03d.jpg" % (SAVE_DIR, photo_id)
    preview_image.save(debug_path, quality=DEBUG_JPG_QUALITY)
    print("Preview saved:", debug_path)

    full_image = None
    for _ in range(3):
        full_image = sensor.snapshot(chn=CAPTURE_CH)

    print(
        "Captured:",
        full_image.width(),
        "x",
        full_image.height(),
        "format:",
        full_image.format(),
    )

    gc.collect()
    image_565 = full_image.to_rgb565()
    jpg_path = "%s/gc2093_%03d.jpg" % (SAVE_DIR, photo_id)
    image_565.save(jpg_path, quality=JPG_QUALITY)
    print("Full-resolution JPG saved:", jpg_path)

    if result is None:
        print("Detection at capture: NONE")
    else:
        print(
            "Detection at capture:",
            result["mode"],
            "score=%.3f" % result["score"],
            "contrast=%.1f" % result["contrast"],
        )

    image_565 = None
    full_image = None
    gc.collect()
    print("Press KEY to capture again.")


try:
    ensure_dir(SAVE_DIR)
    photo_id = find_last_photo_id()
    os.exitpoint(os.EXITPOINT_ENABLE)

    key = YbKey()
    detector = FrameDetector(
        rect_threshold=RECT_THRESHOLD,
        roi_threshold_scale=ROI_RECT_THRESHOLD_SCALE,
        allow_inner_only=ALLOW_INNER_ONLY,
    )

    sensor = Sensor(
        id=2,
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        fps=30,
    )
    sensor.reset()

    # CH0 remains full resolution for saved evidence and later refinement.
    sensor.set_framesize(
        width=CAPTURE_WIDTH,
        height=CAPTURE_HEIGHT,
        chn=CAPTURE_CH,
    )
    sensor.set_pixformat(Sensor.RGB888, chn=CAPTURE_CH)

    # CH1 is an algorithm-friendly RGB565 preview.  It is not directly bound
    # to a display layer because overlays must be visible in CanMV IDE.
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
    print("Nested-frame detector ready.")
    print("CanMV IDE/LCD preview is running.")
    print("RECT_THRESHOLD:", RECT_THRESHOLD)
    print(
        "ROI_RECT_THRESHOLD:",
        int(RECT_THRESHOLD * ROI_RECT_THRESHOLD_SCALE),
    )
    print("Save directory:", SAVE_DIR)
    print("Press onboard KEY to save preview + 1080p JPG.")

    clock = time.clock()
    frame_count = 0
    detection_count = 0
    track_misses = 0
    last_mode = ""
    result = None
    error_text = None
    detect_ms = 0
    tracking_roi = None
    last_detection_action = "SEARCH"

    # Lightweight timing accumulators.  Their averages are printed every
    # STATUS_PRINT_EVERY_N_FRAMES frames so the actual bottleneck can be
    # separated into camera, detector and IDE/LCD display time.
    profile_frames = 0
    profile_snapshot_ms = 0
    profile_detect_ms = 0
    profile_detect_runs = 0
    profile_roi_ms = 0
    profile_roi_runs = 0
    profile_roi_successes = 0
    profile_full_ms = 0
    profile_full_runs = 0
    profile_display_ms = 0
    profile_loop_ms = 0
    profile_full_fallbacks = 0
    profile_snaps = 0

    while True:
        os.exitpoint()
        clock.tick()

        loop_start_ms = time.ticks_ms()
        snapshot_start_ms = loop_start_ms
        preview = sensor.snapshot(chn=PREVIEW_CH)
        snapshot_ms = time.ticks_diff(
            time.ticks_ms(), snapshot_start_ms
        )
        error_text = None
        run_detection = (
            result is None
            or frame_count % DETECT_EVERY_N_FRAMES == 0
        )

        if run_detection:
            detection_count += 1
            use_global_search = (
                tracking_roi is None
                or detection_count % GLOBAL_SEARCH_EVERY_N_DETECTIONS == 0
            )
            active_roi = None if use_global_search else tracking_roi
            start_ms = time.ticks_ms()
            new_result = None
            used_full_fallback = False

            try:
                attempt_start_ms = time.ticks_ms()
                new_result = detector.detect(preview, roi=active_roi)
                attempt_ms = time.ticks_diff(
                    time.ticks_ms(), attempt_start_ms
                )
                if active_roi is None:
                    profile_full_ms += attempt_ms
                    profile_full_runs += 1
                else:
                    profile_roi_ms += attempt_ms
                    profile_roi_runs += 1
                    if new_result is not None:
                        profile_roi_successes += 1

                if new_result is None and active_roi is not None:
                    # A target can jump completely outside the tracking ROI.
                    # Re-run a full-frame search now instead of holding the old
                    # box until a later scheduled detection frame.
                    used_full_fallback = True
                    fallback_start_ms = time.ticks_ms()
                    new_result = detector.detect(preview, roi=None)
                    profile_full_ms += time.ticks_diff(
                        time.ticks_ms(), fallback_start_ms
                    )
                    profile_full_runs += 1
            except Exception as error:
                error_text = repr(error)

            detect_ms = time.ticks_diff(time.ticks_ms(), start_ms)
            profile_detect_ms += detect_ms
            profile_detect_runs += 1
            if used_full_fallback:
                profile_full_fallbacks += 1

            if new_result is not None:
                had_lock = result is not None
                snap_update = (
                    had_lock and should_snap_lock(result, new_result)
                )
                result = smooth_locked_result(result, new_result)
                tracking_roi = make_tracking_roi(
                    result, PREVIEW_WIDTH, PREVIEW_HEIGHT
                )
                track_misses = 0
                if not had_lock:
                    last_detection_action = "ACQUIRE"
                elif snap_update:
                    profile_snaps += 1
                    last_detection_action = "SNAP"
                elif used_full_fallback:
                    last_detection_action = "REACQUIRE"
                else:
                    last_detection_action = "SMOOTH"
            else:
                track_misses += 1
                last_detection_action = "HOLD"
                if track_misses >= MAX_TRACK_MISSES:
                    result = None
                    tracking_roi = None
                    last_detection_action = "SEARCH"

        fps = clock.fps()
        draw_overlay(
            preview,
            detector,
            result,
            detect_ms,
            fps,
            tracking_roi,
            error_text,
        )
        display_start_ms = time.ticks_ms()
        Display.show_image(preview, x=PREVIEW_X, y=PREVIEW_Y)
        display_ms = time.ticks_diff(
            time.ticks_ms(), display_start_ms
        )

        profile_frames += 1
        profile_snapshot_ms += snapshot_ms
        profile_display_ms += display_ms
        profile_loop_ms += time.ticks_diff(
            time.ticks_ms(), loop_start_ms
        )

        mode = "ERROR" if error_text is not None else "SEARCH"
        if result is not None:
            mode = result["mode"]
        if mode != last_mode:
            print("Detector state:", mode)
            if error_text is not None:
                print("DETECT_ERROR:", error_text)
            last_mode = mode

        frame_count += 1
        if frame_count % STATUS_PRINT_EVERY_N_FRAMES == 0:
            scope = "FULL" if detector.last_roi is None else "ROI"
            if result is None:
                print(
                    "status=SEARCH scope=%s threshold=%d raw=%d candidates=%d errors=%d time=%dms fps=%.1f"
                    % (
                        scope,
                        detector.last_threshold,
                        detector.last_raw_rect_count,
                        len(detector.last_candidates),
                        detector.last_candidate_errors,
                        detect_ms,
                        fps,
                    )
                )
            else:
                print(
                    "status=%s scope=%s threshold=%d score=%.3f contrast=%.1f outer_sides=%d outer_edge=%.1f raw=%d candidates=%d errors=%d time=%dms fps=%.1f"
                    % (
                        result["mode"],
                        scope,
                        detector.last_threshold,
                        result["score"],
                        result["contrast"],
                        result["outer_valid_sides"],
                        result["outer_edge_response"],
                        detector.last_raw_rect_count,
                        len(detector.last_candidates),
                        detector.last_candidate_errors,
                        detect_ms,
                        fps,
                    )
                )

            average_detect_ms = 0
            if profile_detect_runs > 0:
                average_detect_ms = (
                    profile_detect_ms // profile_detect_runs
                )
            average_roi_ms = 0
            if profile_roi_runs > 0:
                average_roi_ms = profile_roi_ms // profile_roi_runs
            average_full_ms = 0
            if profile_full_runs > 0:
                average_full_ms = profile_full_ms // profile_full_runs
            print(
                "perf action=%s snapshot=%dms detect=%dms roi=%dms(%d/%d ok) full=%dms(%d runs) display=%dms loop=%dms fallbacks=%d snaps=%d"
                % (
                    last_detection_action,
                    profile_snapshot_ms // max(profile_frames, 1),
                    average_detect_ms,
                    average_roi_ms,
                    profile_roi_successes,
                    profile_roi_runs,
                    average_full_ms,
                    profile_full_runs,
                    profile_display_ms // max(profile_frames, 1),
                    profile_loop_ms // max(profile_frames, 1),
                    profile_full_fallbacks,
                    profile_snaps,
                )
            )
            profile_frames = 0
            profile_snapshot_ms = 0
            profile_detect_ms = 0
            profile_detect_runs = 0
            profile_roi_ms = 0
            profile_roi_runs = 0
            profile_roi_successes = 0
            profile_full_ms = 0
            profile_full_runs = 0
            profile_display_ms = 0
            profile_loop_ms = 0
            profile_full_fallbacks = 0
            profile_snaps = 0

        if key.is_pressed():
            time.sleep_ms(30)
            if key.is_pressed():
                try:
                    save_capture(preview, result)
                except Exception as error:
                    print("CAPTURE_ERROR:", repr(error))

                while key.is_pressed():
                    os.exitpoint()
                    time.sleep_ms(20)

        time.sleep_ms(1)


except KeyboardInterrupt:
    print("")
    print("Stopped by CanMV IDE.")


except Exception as error:
    print("")
    print("PROGRAM_ERROR:", repr(error))


finally:
    if isinstance(sensor, Sensor):
        try:
            sensor.stop()
        except Exception as error:
            print("Sensor stop error:", repr(error))

    if display_inited:
        try:
            Display.deinit()
        except Exception as error:
            print("Display deinit error:", repr(error))

    gc.collect()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    print("Camera stopped.")
