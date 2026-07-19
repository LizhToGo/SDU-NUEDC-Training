"""Button-triggered single-shot localization and distance measurement.

Idle state only previews the camera.  Each onboard-key press performs one
coarse 512x288 search, one local 1920x1080 edge refinement, saves annotated
images and metadata under /data/captures, prints the distance result, then
keeps that captured frame outline fixed on the live preview until the next
press.
"""

import gc
import os
import time

from media.sensor import *
from media.display import *
from ybUtils.YbKey import YbKey

from frame_detector import FrameDetector
from high_res_refiner import HighResRefiner
from distance_estimator import estimate_distance


SAVE_DIR = "/data/captures"

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
SAVED_JPG_QUALITY = 95
PREVIEW_JPG_QUALITY = 90

RECT_THRESHOLD = 1800
COARSE_MAX_ATTEMPTS = 3
HIGH_RES_FLUSH_FRAMES = 2

# Exact dimensions from the official target definition.
EXPECTED_SHORT_RATIO = 17.0 / 21.0
EXPECTED_LONG_RATIO = 25.7 / 29.7
EXPECTED_OUTER_ASPECT = 29.7 / 21.0
EXPECTED_INNER_ASPECT = 25.7 / 17.0


sensor = None
display_inited = False
key = None
measurement_id = 0


def ensure_dir(path):
    try:
        os.mkdir(path)
    except OSError:
        pass


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


def draw_measurement_overlay(image, result, status_color):
    if result is not None:
        draw_quad(image, result.get("outer_corners"), (255, 50, 50), 1)
        draw_quad(image, result.get("inner_corners"), (0, 255, 80), 1)
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


def coarse_result_to_preview(coarse_result):
    if coarse_result is None:
        return None
    return {
        "mode": "COARSE_ONLY",
        "outer_corners": coarse_result["outer_corners"],
        "inner_corners": coarse_result["inner_corners"],
    }


def format_corners(corners):
    if corners is None:
        return "NONE"
    parts = []
    for x, y in corners:
        parts.append("(%d,%d)" % (int(x), int(y)))
    return " ".join(parts)


def save_metadata(
    path,
    coarse_result,
    coarse_candidate_count,
    high_result,
    distance_result,
    coarse_ms,
    convert_ms,
    refine_ms,
    total_ms,
):
    with open(path, "w") as file:
        file.write("coarse_mode=%s\n" % coarse_result["mode"])
        file.write("coarse_score=%.6f\n" % coarse_result["score"])
        file.write("coarse_candidate_count=%d\n" % coarse_candidate_count)
        file.write("coarse_ms=%d\n" % coarse_ms)
        file.write("rgb888_to_rgb565_ms=%d\n" % convert_ms)
        file.write("refine_ms=%d\n" % refine_ms)
        file.write("total_ms=%d\n" % total_ms)
        file.write("high_mode=%s\n" % high_result["mode"])
        file.write(
            "measurement_valid=%s\n"
            % ("YES" if high_result["measurement_valid"] else "NO")
        )
        file.write("high_quality_score=%.3f\n" % high_result["quality_score"])
        file.write(
            "frame_model_disagreement=%.6f\n"
            % high_result["frame_model_disagreement"]
        )
        file.write(
            "inner_valid_sides=%d\n" % high_result["inner_valid_sides"]
        )
        file.write(
            "outer_valid_sides=%d\n" % high_result["outer_valid_sides"]
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
        if distance_result is None:
            file.write("distance_cm=NONE\n")
        else:
            file.write(
                "distance_cm=%.3f\n" % distance_result["distance_cm"]
            )
            file.write("distance_method=%s\n" % distance_result["method"])
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


def save_failed_preview(preview, current_id):
    path = "%s/measure_%03d_failed.jpg" % (SAVE_DIR, current_id)
    preview.save(path, quality=PREVIEW_JPG_QUALITY)
    print("Failed preview saved:", path)


def measure_once(first_preview, detector, refiner):
    global measurement_id
    measurement_id += 1
    current_id = measurement_id
    total_start = time.ticks_ms()

    print("")
    print("MEASURE_START id=%03d" % current_id)

    best_result = None
    coarse_results = []
    coarse_total_ms = 0
    preview = first_preview
    for attempt in range(COARSE_MAX_ATTEMPTS):
        if attempt > 0:
            preview = sensor.snapshot(chn=PREVIEW_CH)
        start = time.ticks_ms()
        candidate = detector.detect(preview, roi=None)
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        coarse_total_ms += elapsed

        if candidate is None:
            print(
                "coarse attempt=%d result=NONE raw=%d candidates=%d time=%dms"
                % (
                    attempt + 1,
                    detector.last_raw_rect_count,
                    len(detector.last_candidates),
                    elapsed,
                )
            )
        else:
            print(
                "coarse attempt=%d mode=%s score=%.3f time=%dms"
                % (
                    attempt + 1,
                    candidate["mode"],
                    candidate["score"],
                    elapsed,
                )
            )
            if best_result is None or candidate["score"] > best_result["score"]:
                best_result = candidate
            coarse_results.append(candidate)

    if best_result is None:
        print("MEASURE_FAILED: coarse frame not found")
        save_failed_preview(preview, current_id)
        return None, None, (255, 50, 50)

    gc.collect()
    high_capture = None
    for _ in range(HIGH_RES_FLUSH_FRAMES):
        high_capture = sensor.snapshot(chn=CAPTURE_CH)

    # On this Yahboom CanMV firmware RGB888 can be captured and converted, but
    # get_pixel() on that buffer returns None.  The refiner performs sparse
    # pixel reads, so convert once *before* refinement.  RGB565 still retains
    # much more spatial detail than the 512x288 coarse image and can be saved
    # directly as JPG afterwards.
    convert_start = time.ticks_ms()
    high_image = high_capture.to_rgb565()
    high_capture = None
    convert_ms = time.ticks_diff(time.ticks_ms(), convert_start)
    gc.collect()
    if high_image is None:
        print("MEASURE_FAILED: RGB888 to RGB565 conversion returned NONE")
        save_failed_preview(preview, current_id)
        return coarse_result_to_preview(best_result), None, (255, 170, 0)

    print(
        "high image=%dx%d format=%s convert=%dms"
        % (
            high_image.width(),
            high_image.height(),
            str(high_image.format()),
            convert_ms,
        )
    )

    # Coarse confidence mainly describes rectangle plausibility; it does not
    # guarantee that all four low-resolution corners are equally accurate.
    # Refine every successful coarse hypothesis against the same 1080p frame
    # and let high-resolution edge evidence select the final result.
    high_result = None
    selected_coarse_result = best_result
    refine_ms = 0
    for candidate_index in range(len(coarse_results)):
        candidate = coarse_results[candidate_index]
        refine_start = time.ticks_ms()
        candidate_high = refiner.refine(
            high_image,
            candidate,
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
        )
        candidate_refine_ms = time.ticks_diff(
            time.ticks_ms(), refine_start
        )
        refine_ms += candidate_refine_ms
        if candidate_high is None:
            print(
                "refine candidate=%d result=NONE time=%dms"
                % (candidate_index + 1, candidate_refine_ms)
            )
            continue

        print(
            "refine candidate=%d coarse=%.3f high=%s quality=%.1f sides=%d/%d radius=%d/%d model_error=%.3f time=%dms"
            % (
                candidate_index + 1,
                candidate["score"],
                candidate_high["mode"],
                candidate_high["quality_score"],
                candidate_high["inner_valid_sides"],
                candidate_high["outer_valid_sides"],
                candidate_high["inner_search_radius"],
                candidate_high["outer_search_radius"],
                candidate_high["frame_model_disagreement"],
                candidate_refine_ms,
            )
        )
        if (
            high_result is None
            or candidate_high["quality_score"]
            > high_result["quality_score"]
        ):
            high_result = candidate_high
            selected_coarse_result = candidate

    best_result = selected_coarse_result
    if high_result is None:
        print("MEASURE_FAILED: high-resolution refinement returned NONE")
        save_failed_preview(preview, current_id)
        high_image = None
        gc.collect()
        return coarse_result_to_preview(best_result), None, (255, 170, 0)

    distance_result = estimate_distance(high_result)
    preview_result = high_result_to_preview(high_result)

    high_path = "%s/measure_%03d_1080.jpg" % (SAVE_DIR, current_id)
    preview_path = "%s/measure_%03d_preview.jpg" % (SAVE_DIR, current_id)
    metadata_path = "%s/measure_%03d.txt" % (SAVE_DIR, current_id)

    # high_image is already RGB565 because the conversion must happen before
    # sparse get_pixel() refinement on this firmware.
    draw_measurement_overlay(high_image, high_result, (0, 220, 255))
    high_image.save(high_path, quality=SAVED_JPG_QUALITY)
    high_image = None
    gc.collect()

    draw_measurement_overlay(preview, preview_result, (0, 220, 255))
    preview.save(preview_path, quality=PREVIEW_JPG_QUALITY)

    total_ms = time.ticks_diff(time.ticks_ms(), total_start)
    save_metadata(
        metadata_path,
        best_result,
        len(coarse_results),
        high_result,
        distance_result,
        coarse_total_ms,
        convert_ms,
        refine_ms,
        total_ms,
    )

    print(
        "high mode=%s quality=%.1f inner_sides=%d outer_sides=%d inner_edge=%.1f outer_edge=%.1f radius=%d/%d model_error=%.3f convert=%dms refine_total=%dms"
        % (
            high_result["mode"],
            high_result["quality_score"],
            high_result["inner_valid_sides"],
            high_result["outer_valid_sides"],
            high_result["inner_edge_response"],
            high_result["outer_edge_response"],
            high_result["inner_search_radius"],
            high_result["outer_search_radius"],
            high_result["frame_model_disagreement"],
            convert_ms,
            refine_ms,
        )
    )
    if distance_result is None:
        print("distance=NONE")
        status_color = (255, 170, 0)
    else:
        print(
            "distance=%.1fcm method=%s source=%s frame_scale=%.4f anisotropy=%.3f disagreement=%.3f"
            % (
                distance_result["distance_cm"],
                distance_result["method"],
                distance_result["source"],
                distance_result["frame_scale"],
                distance_result["anisotropy"],
                distance_result["outer_inner_disagreement"],
            )
        )
        if distance_result["method"] == "CALIBRATION_TABLE":
            status_color = (0, 255, 80)
        else:
            status_color = (0, 220, 255)

    print("High-resolution image:", high_path)
    print("Preview image:", preview_path)
    print("Metadata:", metadata_path)
    print("MEASURE_DONE id=%03d total=%dms" % (current_id, total_ms))
    print("Press KEY for the next independent measurement.")
    return preview_result, distance_result, status_color


try:
    ensure_dir(SAVE_DIR)
    measurement_id = find_last_measurement_id()
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
    print("Single-shot measurement ready.")
    print(
        "Preview/coarse: %dx%d RGB565, high capture: %dx%d RGB888 -> RGB565 refinement"
        % (PREVIEW_WIDTH, PREVIEW_HEIGHT, CAPTURE_WIDTH, CAPTURE_HEIGHT)
    )
    print("Press onboard KEY once to measure, save and lock the overlay.")
    print("No localization runs while waiting for a key press.")

    locked_preview_result = None
    locked_distance_result = None
    locked_status_color = (80, 120, 255)
    frame_count = 0
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()
        preview = sensor.snapshot(chn=PREVIEW_CH)

        if key.is_pressed():
            time.sleep_ms(30)
            if key.is_pressed():
                locked_preview_result = None
                locked_distance_result = None
                locked_status_color = (255, 210, 0)
                try:
                    (
                        locked_preview_result,
                        locked_distance_result,
                        locked_status_color,
                    ) = measure_once(preview, detector, refiner)
                except Exception as error:
                    # A failed single measurement must not tear down the
                    # camera.  Keep preview alive so the next key press can
                    # retry and print the concrete exception for diagnosis.
                    print("MEASURE_ERROR:", repr(error))
                    locked_preview_result = None
                    locked_distance_result = None
                    locked_status_color = (255, 50, 50)
                    gc.collect()

                while key.is_pressed():
                    os.exitpoint()
                    time.sleep_ms(20)
                preview = sensor.snapshot(chn=PREVIEW_CH)

        draw_measurement_overlay(
            preview,
            locked_preview_result,
            locked_status_color,
        )
        Display.show_image(preview, x=PREVIEW_X, y=PREVIEW_Y)

        frame_count += 1
        if frame_count % 120 == 0:
            print(
                "preview fps=%.1f state=%s"
                % (
                    clock.fps(),
                    "LOCKED" if locked_preview_result is not None else "IDLE",
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
