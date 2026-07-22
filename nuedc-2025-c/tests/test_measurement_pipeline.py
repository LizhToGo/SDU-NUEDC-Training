import ast
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

import high_res_refiner as high_refiner
import frame_detector as frame_detector_module
from frame_detector import FrameDetector


class EmptyRectImage:
    def __init__(self):
        self.thresholds = []

    def find_rects(self, threshold, roi=None):
        self.thresholds.append((threshold, roi))
        return []


class SyntheticRect:
    def __init__(self, corners):
        self._corners = tuple(corners)

    def corners(self):
        return self._corners


class SyntheticFrameImage:
    """Small axis-aligned image model for coarse ring-gate tests."""

    def __init__(
        self,
        inner_corners,
        outer_corners=None,
        background=180,
        include_outer=True,
    ):
        self.inner_corners = tuple(inner_corners)
        self.outer_corners = (
            tuple(outer_corners) if outer_corners is not None else None
        )
        self.background = background
        self.rectangles = []
        if include_outer and self.outer_corners is not None:
            self.rectangles.append(SyntheticRect(self.outer_corners))
        self.rectangles.append(SyntheticRect(self.inner_corners))

    def width(self):
        return 160

    def height(self):
        return 120

    def find_rects(self, threshold, roi=None):
        return self.rectangles

    @staticmethod
    def _inside(corners, x, y):
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        return min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)

    def get_pixel(self, x, y):
        if self._inside(self.inner_corners, x, y):
            return 230
        if (
            self.outer_corners is not None
            and self._inside(self.outer_corners, x, y)
        ):
            return 20
        return self.background


class FixedSizeImage:
    def width(self):
        return 1920

    def height(self):
        return 1080


class SyntheticHighFrame(FixedSizeImage):
    """Ideal 1080p white-black-white frame used for recovery regression."""

    def get_pixel(self, x, y):
        if 993 <= x <= 1115 and 669 <= y <= 852:
            return 230
        if 978 <= x <= 1130 and 655 <= y <= 866:
            return 20
        return 180


def quad_result(
    corners,
    valid_sides,
    geometry_valid,
    edge_rms,
    mean_response,
    raw_valid_sides=None,
):
    if raw_valid_sides is None:
        raw_valid_sides = valid_sides
    return {
        "corners": tuple(corners),
        "valid_sides": valid_sides,
        "raw_valid_sides": raw_valid_sides,
        "geometry_valid": geometry_valid,
        "mean_response": mean_response,
        "raw_mean_response": mean_response,
        "edge_rms": edge_rms,
        "raw_edge_rms": edge_rms,
        "point_count": 96 if valid_sides else 0,
        "search_radius": 12,
    }


GOOD_RING = {
    "valid_samples": 32,
    "inside_contrast": 80.0,
    "outside_contrast": 30.0,
    "inside_pass_ratio": 1.0,
    "outside_pass_ratio": 1.0,
    "min_side_inside_pass_ratio": 1.0,
    "mean_thickness": 20.0,
}


class CoarseConfigurationTests(unittest.TestCase):
    @staticmethod
    def _target_detector():
        return FrameDetector(
            rect_threshold=1800,
            expected_short_ratio=17.0 / 21.0,
            expected_long_ratio=25.7 / 29.7,
            expected_outer_aspect=29.7 / 21.0,
            expected_inner_aspect=25.7 / 17.0,
        )

    def test_threshold_override_reaches_find_rects(self):
        detector = FrameDetector(rect_threshold=1800)
        image = EmptyRectImage()
        results = detector.detect_hypotheses(
            image,
            threshold_override=1200,
        )
        self.assertEqual(results, [])
        self.assertEqual(detector.last_threshold, 1200)
        self.assertEqual(image.thresholds, [(1200, None)])

    def test_center_roi_keeps_the_normal_threshold(self):
        detector = FrameDetector(rect_threshold=1800)
        image = EmptyRectImage()
        roi = (212, 0, 269, 360)

        results = detector.detect_hypotheses(
            image,
            roi=roi,
            scale_roi_threshold=False,
        )

        self.assertEqual(results, [])
        self.assertEqual(detector.last_threshold, 1800)
        self.assertEqual(image.thresholds, [(1800, roi)])

    def test_tracking_roi_can_still_scale_the_threshold(self):
        detector = FrameDetector(rect_threshold=1800, roi_threshold_scale=0.60)
        image = EmptyRectImage()
        roi = (100, 80, 120, 160)

        detector.detect_hypotheses(image, roi=roi)

        self.assertEqual(detector.last_threshold, 1080)
        self.assertEqual(image.thresholds, [(1080, roi)])

    def test_single_shot_uses_640x360_and_bounded_refinement(self):
        path = os.path.join(PROJECT_DIR, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()
        tree = ast.parse(source)
        constants = {}
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if isinstance(target, ast.Name) and isinstance(
                node.value, ast.Constant
            ):
                constants[target.id] = node.value.value
        self.assertEqual(constants["PREVIEW_WIDTH"], 640)
        self.assertEqual(constants["PREVIEW_HEIGHT"], 360)
        self.assertEqual(constants["FALLBACK_RECT_THRESHOLD"], 1200)
        self.assertEqual(constants["MAX_HIGH_RES_HYPOTHESES"], 3)
        self.assertEqual(constants["MAX_FRAME_SCALE_DISAGREEMENT"], 0.0050)
        self.assertEqual(constants["MAX_FRAME_CENTER_SHIFT_RATIO"], 0.10)
        self.assertEqual(constants["CENTER_SEARCH_CENTER_X_RATIO"], 0.54)
        self.assertEqual(constants["CENTER_SEARCH_WIDTH_RATIO"], 0.34)
        self.assertEqual(constants["SCREEN_POWER_UPDATE_INTERVAL_MS"], 200)
        self.assertEqual(
            constants["PIPELINE_VERSION"],
            "2026-07-22-screen-fastdigit-tilt-v8-tempcal-livepower",
        )
        self.assertNotIn("COARSE_MAX_ATTEMPTS", source)
        self.assertIn("refine early_stop", source)
        self.assertIn("periodic_screen_power_update()", source)

    def test_coarse_white_black_white_ring_is_accepted(self):
        inner = ((50, 30), (90, 30), (90, 90), (50, 90))
        outer = ((45, 25), (95, 25), (95, 95), (45, 95))
        image = SyntheticFrameImage(inner, outer, background=180)
        detector = self._target_detector()

        results = detector.detect_hypotheses(image, max_results=4)

        self.assertTrue(results)
        self.assertEqual(results[0]["mode"], "PAIR")
        self.assertGreaterEqual(
            results[0]["coarse_ring_outside_pass_ratio"], 0.90
        )

    def test_measured_good_quad_is_a_clean_seed(self):
        detector = self._target_detector()
        corners = ((330, 221), (372, 218), (371, 282), (330, 283))
        regularity = frame_detector_module._quad_regularity(corners)
        candidate = {
            "inner_reject_reason": None,
            "quad_regularity": regularity,
        }

        self.assertEqual(
            detector._seed_class_for_candidate(candidate),
            "CLEAN_SEED",
        )
        self.assertLess(regularity["max_opposite_error"], 0.06)
        self.assertLess(regularity["diagonal_midpoint_error"], 0.03)

    def test_measured_diagonal_quads_are_location_only_seeds(self):
        detector = self._target_detector()
        malformed = (
            ((328, 241), (370, 201), (370, 287), (328, 281)),
            ((327, 219), (370, 237), (370, 282), (329, 283)),
            ((325, 188), (373, 268), (374, 286), (332, 286)),
            ((371, 176), (371, 282), (330, 283), (330, 224)),
        )

        for corners in malformed:
            regularity = frame_detector_module._quad_regularity(corners)
            candidate = {
                "inner_reject_reason": None,
                "quad_regularity": regularity,
            }
            self.assertEqual(
                detector._seed_class_for_candidate(candidate),
                "ROI_SEED",
                msg=str(corners),
            )

    def test_coarse_dark_outside_rectangle_is_allowed_with_soft_diagnostic(self):
        inner = ((50, 30), (90, 30), (90, 90), (50, 90))
        outer = ((45, 25), (95, 25), (95, 95), (45, 95))
        image = SyntheticFrameImage(inner, outer, background=20)
        detector = self._target_detector()

        results = detector.detect_hypotheses(image, max_results=4)

        self.assertTrue(results)
        self.assertNotIn(
            "RING_OUTSIDE_PASS",
            detector.last_rejection_counts,
        )
        self.assertGreaterEqual(
            detector.last_soft_gate_counts.get("RING_OUTSIDE_PASS", 0),
            1,
        )

    def test_dark_background_does_not_require_location_seed_relaxation(self):
        inner = ((50, 30), (90, 30), (90, 90), (50, 90))
        outer = ((45, 25), (95, 25), (95, 95), (45, 95))
        image = SyntheticFrameImage(inner, outer, background=20)
        detector = self._target_detector()

        results = detector.detect_hypotheses(
            image,
            roi=(35, 0, 80, 120),
            max_results=4,
            scale_roi_threshold=False,
            allow_location_seed=True,
        )

        self.assertTrue(results)
        target_results = [
            result
            for result in results
            if result.get("inner_corners") == inner
        ]
        self.assertTrue(target_results)
        self.assertTrue(
            any(
                not result.get("location_seed_relaxed", False)
                for result in target_results
            )
        )
        self.assertGreaterEqual(
            detector.last_soft_gate_counts.get("RING_OUTSIDE_PASS", 0),
            1,
        )

    def test_one_inaccurate_inside_side_is_a_soft_gate(self):
        detector = self._target_detector()
        ring = {
            "valid_samples": 16,
            "inside_contrast": 30.0,
            "outside_contrast": 25.0,
            "inside_pass_ratio": 0.75,
            "outside_pass_ratio": 1.0,
            "min_side_inside_pass_ratio": 0.0,
        }
        candidate = {
            "contrast": 75.0,
            "predicted_outer_edge_margin": 20.0,
            "aspect": 25.7 / 17.0,
            "ring_metrics": ring,
        }

        hard_reason = detector._inner_candidate_reject_reason(candidate)
        soft_reasons = detector._inner_candidate_soft_reasons(candidate)

        self.assertIsNone(hard_reason)
        self.assertEqual(soft_reasons, ("RING_SIDE_PASS",))
        candidate["inner_reject_reason"] = hard_reason
        candidate["inner_soft_reasons"] = soft_reasons
        ranked = detector._rank_inner_only([candidate])
        self.assertEqual(len(ranked), 1)
        self.assertGreaterEqual(
            ranked[0]["score"], detector.min_soft_inner_score
        )

    def test_coarse_candidate_touching_image_edge_is_rejected(self):
        inner = ((60, 71), (90, 71), (90, 116), (60, 116))
        image = SyntheticFrameImage(
            inner,
            outer_corners=None,
            background=180,
            include_outer=False,
        )
        detector = self._target_detector()

        results = detector.detect_hypotheses(image, max_results=4)

        self.assertEqual(results, [])
        self.assertEqual(
            detector.last_rejection_counts.get("INNER_EDGE"),
            1,
        )

    def test_tracking_roi_is_expanded_and_clipped(self):
        path = os.path.join(PROJECT_DIR, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()
        tree = ast.parse(source)
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "tracking_roi_from_result"
        )
        module = ast.Module(body=[function], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "PREVIEW_WIDTH": 640,
            "PREVIEW_HEIGHT": 360,
            "TRACKING_ROI_MARGIN_SCALE": 1.0,
            "TRACKING_ROI_MIN_MARGIN": 24,
            "TRACKING_ROI_MIN_SIZE": 48,
        }
        exec(compile(module, path, "exec"), namespace)

        roi = namespace["tracking_roi_from_result"]({
            "outer_corners": ((2, 4), (42, 4), (42, 74), (2, 74)),
            "inner_corners": None,
        })

        self.assertEqual(roi[0], 0)
        self.assertEqual(roi[1], 0)
        self.assertGreater(roi[2], 80)
        self.assertGreater(roi[3], 140)

    def test_center_roi_coordinates_and_preview_guides(self):
        path = os.path.join(PROJECT_DIR, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()
        tree = ast.parse(source)
        functions = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name in ("center_search_roi", "draw_alignment_guides")
        ]
        module = ast.Module(body=functions, type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "CENTER_SEARCH_CENTER_X_RATIO": 0.54,
            "CENTER_SEARCH_WIDTH_RATIO": 0.34,
            "ALIGNMENT_CROSS_HALF_LENGTH": 10,
            "CENTER_ROI_GUIDE_COLOR": (0, 220, 255),
            "CENTER_CROSS_COLOR": (255, 255, 0),
        }
        exec(compile(module, path, "exec"), namespace)

        roi = namespace["center_search_roi"](640, 360)
        self.assertEqual(roi, (237, 0, 218, 360))

        class Preview:
            def __init__(self):
                self.lines = []

            def width(self):
                return 640

            def height(self):
                return 360

            def draw_line(self, *args, **kwargs):
                self.lines.append((args, kwargs))

        preview = Preview()
        namespace["draw_alignment_guides"](preview)
        self.assertEqual(len(preview.lines), 4)
        self.assertEqual(preview.lines[0][0], (237, 0, 237, 359))
        self.assertEqual(preview.lines[1][0], (454, 0, 454, 359))
        self.assertEqual(preview.lines[2][0], (336, 180, 356, 180))
        self.assertEqual(preview.lines[3][0], (346, 170, 346, 190))
        for _, kwargs in preview.lines:
            self.assertEqual(kwargs["thickness"], 1)

    def test_single_shot_uses_only_center_scans_and_aperture_recovery(self):
        path = os.path.join(PROJECT_DIR, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()

        self.assertIn('"CENTER"', source)
        self.assertIn('"CENTER_LOW"', source)
        self.assertIn('"CENTER_APERTURE"', source)
        self.assertIn('"HIGH_RES_CENTER_APERTURE"', source)
        self.assertNotIn('"FULL_FALLBACK"', source)
        self.assertIn("allow_location_seed=True", source)
        self.assertIn("threshold_override=FALLBACK_RECT_THRESHOLD", source)
        self.assertIn("run_aperture_scan", source)
        self.assertIn("CENTER_TARGET_NOT_FOUND", source)
        self.assertNotIn("last_success_preview_result", source)
        self.assertEqual(
            source.count('file.write("pipeline_version=%s\\n"'),
            2,
        )
        self.assertIn("READY SINGLE_SHOT_MEASUREMENT BAUD=%d VER=%s", source)
        self.assertGreaterEqual(source.count('"inner_refine_path=%s\\n"'), 2)
        self.assertGreaterEqual(source.count('"inner_edge_rms=%.6f\\n"'), 2)
        self.assertGreaterEqual(
            source.count('"inner_wide_search_reason=%s\\n"'),
            2,
        )
        self.assertLess(
            source.rfind("command = poll_uart_command()"),
            source.rfind("draw_alignment_guides(preview)"),
        )


class HighResolutionGateTests(unittest.TestCase):
    def test_roi_seed_is_regularized_near_the_physical_inner_frame(self):
        malformed_preview = (
            (371, 176),
            (371, 282),
            (330, 283),
            (330, 224),
        )
        malformed_high = tuple(
            (x * 3.0, y * 3.0) for x, y in malformed_preview
        )
        regularized, diagnostics = high_refiner._regularize_inner_seed(
            malformed_high,
            25.7 / 17.0,
        )

        self.assertIsNotNone(regularized)
        center = high_refiner._center(regularized)
        # Successful neighboring frames put the true center at about
        # (1053, 753). The malformed apex must no longer be used as a corner.
        self.assertLess(abs(center[0] - 1053.0), 5.0)
        self.assertLess(abs(center[1] - 753.0), 31.0)
        self.assertGreater(diagnostics["max_nearest_corner_shift"], 30.0)
        short_side, long_side = high_refiner._quad_dimensions(regularized)
        self.assertAlmostEqual(long_side / short_side, 25.7 / 17.0, places=5)

    def test_roi_seed_forces_bounded_wide_inner_relocalization(self):
        original_adaptive = high_refiner._adaptive_refine_quad
        original_border = high_refiner._border_ring_metrics
        calls = []

        def fake_adaptive(
            image,
            predicted,
            polarity,
            narrow_radius,
            wide_radius,
            sample_count,
            min_response,
            force_wide=False,
            max_acceptable_rms=None,
        ):
            calls.append((
                polarity,
                narrow_radius,
                wide_radius,
                force_wide,
                max_acceptable_rms,
            ))
            return quad_result(predicted, 4, True, 0.4, 100.0)

        try:
            high_refiner._adaptive_refine_quad = fake_adaptive
            high_refiner._border_ring_metrics = lambda *args: dict(GOOD_RING)
            result = high_refiner.HighResRefiner().refine(
                FixedSizeImage(),
                {
                    "inner_corners": (
                        (371, 176),
                        (371, 282),
                        (330, 283),
                        (330, 224),
                    ),
                    "outer_corners": None,
                    "seed_class": "ROI_SEED",
                },
                640,
                360,
            )
        finally:
            high_refiner._adaptive_refine_quad = original_adaptive
            high_refiner._border_ring_metrics = original_border

        self.assertTrue(calls[0][3])
        self.assertGreaterEqual(calls[0][2], 60)
        self.assertLessEqual(calls[0][2], 84)
        self.assertTrue(result["seed_regularized"])
        self.assertTrue(result["large_seed_shift"])
        self.assertEqual(result["coarse_seed_class"], "ROI_SEED")
        self.assertIsNotNone(result["relocalize_roi"])

    def test_modest_199cm_roi_seed_does_not_force_wide_search(self):
        original_adaptive = high_refiner._adaptive_refine_quad
        original_border = high_refiner._border_ring_metrics
        calls = []

        def fake_adaptive(
            image,
            predicted,
            polarity,
            narrow_radius,
            wide_radius,
            sample_count,
            min_response,
            force_wide=False,
            max_acceptable_rms=None,
        ):
            calls.append((polarity, force_wide, max_acceptable_rms))
            return quad_result(predicted, 4, True, 0.4, 100.0)

        try:
            high_refiner._adaptive_refine_quad = fake_adaptive
            high_refiner._border_ring_metrics = lambda *args: dict(GOOD_RING)
            result = high_refiner.HighResRefiner().refine(
                FixedSizeImage(),
                {
                    "inner_corners": (
                        (322, 209),
                        (360, 213),
                        (356, 260),
                        (326, 268),
                    ),
                    "outer_corners": None,
                    "seed_class": "ROI_SEED",
                },
                640,
                360,
            )
        finally:
            high_refiner._adaptive_refine_quad = original_adaptive
            high_refiner._border_ring_metrics = original_border

        self.assertFalse(calls[0][1])
        self.assertEqual(calls[0][2], 1.5)
        self.assertFalse(result["large_seed_shift"])
        self.assertLess(
            result["seed_regularization_max_shift"],
            result["large_seed_shift_threshold"],
        )

    def test_malformed_160cm_seeds_recover_on_high_resolution_ring(self):
        malformed = (
            ((328, 241), (370, 201), (370, 287), (328, 281)),
            ((327, 219), (370, 237), (370, 282), (329, 283)),
            ((371, 176), (371, 282), (330, 283), (330, 224)),
        )
        expected = ((992, 669), (1115, 668), (1116, 853), (993, 852))

        for corners in malformed:
            result = high_refiner.HighResRefiner().refine(
                SyntheticHighFrame(),
                {
                    "inner_corners": corners,
                    "outer_corners": None,
                    "seed_class": "ROI_SEED",
                },
                640,
                360,
            )
            self.assertTrue(result["measurement_valid"], msg=str(corners))
            self.assertEqual(result["measurement_reject_reason"], "OK")
            for actual, target in zip(result["inner_corners"], expected):
                self.assertLessEqual(abs(actual[0] - target[0]), 2)
                self.assertLessEqual(abs(actual[1] - target[1]), 2)

    def test_single_shot_bypasses_broken_preview_roi_with_track_seed(self):
        path = os.path.join(PROJECT_DIR, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()

        self.assertIn('"seed_class": "TRACK_SEED"', source)
        self.assertIn("coarse direct_track", source)
        self.assertIn("if tracking_seed is None:", source)
        self.assertIn(
            "current_tracking_hint = attempt.get(\"preview_result\")",
            source,
        )
        self.assertNotIn("last_success_preview_result", source)

    def test_invalid_narrow_geometry_triggers_wide_search(self):
        original_refine_quad = high_refiner._refine_quad
        calls = []

        def fake_refine_quad(
            image,
            predicted,
            polarity,
            search_radius,
            probe,
            sample_count,
            min_response,
        ):
            calls.append(search_radius)
            if len(calls) == 1:
                return quad_result(
                    predicted,
                    0,
                    False,
                    -1.0,
                    0.0,
                    raw_valid_sides=4,
                )
            return quad_result(predicted, 4, True, 0.4, 80.0)

        try:
            high_refiner._refine_quad = fake_refine_quad
            result = high_refiner._adaptive_refine_quad(
                None,
                ((0, 0), (10, 0), (10, 10), (0, 10)),
                1,
                10,
                30,
                24,
                12.0,
            )
        finally:
            high_refiner._refine_quad = original_refine_quad

        self.assertEqual(calls, [10, 30])
        self.assertEqual(result["valid_sides"], 4)
        self.assertTrue(result["wide_search_used"])

    def test_valid_narrow_geometry_does_not_run_wide_search(self):
        original_refine_quad = high_refiner._refine_quad
        calls = []

        def fake_refine_quad(
            image,
            predicted,
            polarity,
            search_radius,
            probe,
            sample_count,
            min_response,
        ):
            calls.append(search_radius)
            return quad_result(predicted, 4, True, 0.4, 80.0)

        try:
            high_refiner._refine_quad = fake_refine_quad
            result = high_refiner._adaptive_refine_quad(
                None,
                ((0, 0), (10, 0), (10, 10), (0, 10)),
                1,
                10,
                30,
                24,
                12.0,
            )
        finally:
            high_refiner._refine_quad = original_refine_quad

        self.assertEqual(calls, [10])
        self.assertFalse(result["wide_search_used"])

    def test_stronger_wide_response_cannot_override_clean_narrow_fit(self):
        original_refine_quad = high_refiner._refine_quad
        calls = []

        def fake_refine_quad(
            image,
            predicted,
            polarity,
            search_radius,
            probe,
            sample_count,
            min_response,
        ):
            calls.append(search_radius)
            if len(calls) == 1:
                return quad_result(predicted, 4, True, 0.45, 70.0)
            return quad_result(predicted, 4, True, 1.80, 130.0)

        try:
            high_refiner._refine_quad = fake_refine_quad
            result = high_refiner._adaptive_refine_quad(
                None,
                ((0, 0), (10, 0), (10, 10), (0, 10)),
                1,
                10,
                30,
                24,
                12.0,
                force_wide=True,
                max_acceptable_rms=1.5,
            )
        finally:
            high_refiner._refine_quad = original_refine_quad

        self.assertEqual(calls, [10, 30])
        self.assertEqual(result["refine_path"], "NARROW")
        self.assertAlmostEqual(result["edge_rms"], 0.45)
        self.assertAlmostEqual(result["wide_edge_rms"], 1.80)
        self.assertEqual(result["wide_search_reason"], "SEED_SHIFT")

    def test_bad_narrow_rms_triggers_and_selects_clean_wide_fit(self):
        original_refine_quad = high_refiner._refine_quad
        calls = []

        def fake_refine_quad(
            image,
            predicted,
            polarity,
            search_radius,
            probe,
            sample_count,
            min_response,
        ):
            calls.append(search_radius)
            if len(calls) == 1:
                return quad_result(predicted, 4, True, 2.10, 110.0)
            return quad_result(predicted, 4, True, 0.55, 75.0)

        try:
            high_refiner._refine_quad = fake_refine_quad
            result = high_refiner._adaptive_refine_quad(
                None,
                ((0, 0), (10, 0), (10, 10), (0, 10)),
                1,
                10,
                30,
                24,
                12.0,
                max_acceptable_rms=1.5,
            )
        finally:
            high_refiner._refine_quad = original_refine_quad

        self.assertEqual(calls, [10, 30])
        self.assertEqual(result["refine_path"], "WIDE")
        self.assertAlmostEqual(result["edge_rms"], 0.55)
        self.assertEqual(result["wide_search_reason"], "RMS")

    def _run_controlled_refinement(
        self,
        outer_factory,
        ring_metrics=None,
        localization_only=False,
    ):
        inner_corners = (
            (875.0, 410.0),
            (1045.0, 410.0),
            (1045.0, 667.0),
            (875.0, 667.0),
        )
        original_adaptive = high_refiner._adaptive_refine_quad
        original_border = high_refiner._border_ring_metrics
        original_refine_quad = high_refiner._refine_quad
        direct_refine_calls = []

        def fake_adaptive(
            image,
            predicted,
            polarity,
            narrow_radius,
            wide_radius,
            sample_count,
            min_response,
            force_wide=False,
            max_acceptable_rms=None,
        ):
            if polarity > 0:
                return quad_result(inner_corners, 4, True, 0.4, 100.0)
            return outer_factory(predicted)

        def unexpected_direct_refine(*args, **kwargs):
            direct_refine_calls.append(args[4])
            predicted = args[1]
            return quad_result(
                predicted,
                0,
                False,
                -1.0,
                0.0,
                raw_valid_sides=0,
            )

        try:
            high_refiner._adaptive_refine_quad = fake_adaptive
            high_refiner._border_ring_metrics = lambda *args: dict(
                GOOD_RING if ring_metrics is None else ring_metrics
            )
            high_refiner._refine_quad = unexpected_direct_refine
            result = high_refiner.HighResRefiner().refine(
                FixedSizeImage(),
                {
                    "inner_corners": inner_corners,
                    "outer_corners": None,
                },
                1920,
                1080,
                localization_only=localization_only,
            )
        finally:
            high_refiner._adaptive_refine_quad = original_adaptive
            high_refiner._border_ring_metrics = original_border
            high_refiner._refine_quad = original_refine_quad
        return result, direct_refine_calls

    def test_strong_inner_and_ring_can_pass_without_outer_lines(self):
        def invalid_outer(predicted):
            return quad_result(
                predicted,
                0,
                False,
                -1.0,
                0.0,
                raw_valid_sides=4,
            )

        result, direct_refine_calls = self._run_controlled_refinement(
            invalid_outer
        )
        self.assertEqual(result["mode"], "HIGH_RES_INNER")
        self.assertTrue(result["measurement_valid"])
        self.assertEqual(result["measurement_reject_reason"], "OK")
        self.assertTrue(result["inner_only_accepted"])
        self.assertFalse(result["outer_independently_valid"])
        self.assertFalse(result["outer_conflict_demoted"])
        self.assertEqual(direct_refine_calls, [])

    def test_tilt_localization_skips_outer_edge_refinement(self):
        def forbidden_outer(predicted):
            self.fail("localization-only mode must not refine outer edges")

        result, direct_refine_calls = self._run_controlled_refinement(
            forbidden_outer,
            localization_only=True,
        )
        self.assertEqual(result["mode"], "HIGH_RES_INNER")
        self.assertTrue(result["localization_valid"])
        self.assertTrue(result["localization_only"])
        self.assertFalse(result["measurement_valid"])
        self.assertEqual(
            result["measurement_reject_reason"],
            "TILT_LOCALIZATION_ONLY",
        )
        self.assertEqual(result["outer_valid_sides"], 0)
        self.assertEqual(direct_refine_calls, [])

    def test_conflicting_outer_with_complete_ring_is_demoted(self):
        def conflicting_outer(predicted):
            center_x = 960.0
            center_y = 538.5
            half_width = 145.0
            half_height = 205.0
            corners = (
                (center_x - half_width, center_y - half_height),
                (center_x + half_width, center_y - half_height),
                (center_x + half_width, center_y + half_height),
                (center_x - half_width, center_y + half_height),
            )
            return quad_result(corners, 4, True, 0.5, 70.0)

        result, direct_refine_calls = self._run_controlled_refinement(
            conflicting_outer
        )
        self.assertEqual(result["mode"], "HIGH_RES_INNER")
        self.assertTrue(result["measurement_valid"])
        self.assertEqual(result["measurement_reject_reason"], "OK")
        self.assertTrue(result["inner_only_accepted"])
        self.assertTrue(result["outer_independently_valid"])
        self.assertTrue(result["outer_conflict_demoted"])
        self.assertGreater(result["detected_frame_disagreement"], 0.04)
        self.assertEqual(result["frame_model_disagreement"], 0.0)
        self.assertEqual(direct_refine_calls, [])

    def test_conflicting_outer_with_dark_scene_background_is_demoted(self):
        def conflicting_outer(predicted):
            center_x = 960.0
            center_y = 538.5
            half_width = 145.0
            half_height = 205.0
            corners = (
                (center_x - half_width, center_y - half_height),
                (center_x + half_width, center_y - half_height),
                (center_x + half_width, center_y + half_height),
                (center_x - half_width, center_y + half_height),
            )
            return quad_result(corners, 4, True, 0.5, 70.0)

        incomplete_ring = dict(GOOD_RING)
        incomplete_ring["outside_pass_ratio"] = 0.25
        result, direct_refine_calls = self._run_controlled_refinement(
            conflicting_outer,
            ring_metrics=incomplete_ring,
        )
        self.assertEqual(result["mode"], "HIGH_RES_INNER")
        self.assertTrue(result["measurement_valid"])
        self.assertEqual(result["measurement_reject_reason"], "OK")
        self.assertTrue(result["inner_only_accepted"])
        self.assertTrue(result["outer_conflict_demoted"])
        self.assertTrue(result["outer_independently_valid"])
        self.assertEqual(direct_refine_calls, [])

    def test_conflicting_outer_with_weak_inner_ring_is_not_demoted(self):
        def conflicting_outer(predicted):
            center_x = 960.0
            center_y = 538.5
            corners = (
                (center_x - 145.0, center_y - 205.0),
                (center_x + 145.0, center_y - 205.0),
                (center_x + 145.0, center_y + 205.0),
                (center_x - 145.0, center_y + 205.0),
            )
            return quad_result(corners, 4, True, 0.5, 70.0)

        weak_ring = dict(GOOD_RING)
        weak_ring["inside_pass_ratio"] = 0.50
        weak_ring["min_side_inside_pass_ratio"] = 0.25
        weak_ring["outside_pass_ratio"] = 0.0
        result, _ = self._run_controlled_refinement(
            conflicting_outer,
            ring_metrics=weak_ring,
        )

        self.assertEqual(result["mode"], "HIGH_RES_PAIR")
        self.assertFalse(result["measurement_valid"])
        self.assertEqual(result["measurement_reject_reason"], "RING_COVERAGE")
        self.assertFalse(result["outer_conflict_demoted"])


if __name__ == "__main__":
    unittest.main()
