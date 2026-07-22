import math
import os
import sys
import unittest
import ast


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


from advanced_target_detector import (  # noqa: E402
    AdvancedTargetDetector,
    MODE_AUTO_MIN,
    MODE_DIGIT_SELECT,
    MODE_TILT_MIN,
    advanced_results_consistent,
)
from composite_square_detector import (  # noqa: E402
    CompositeSquareDetector,
    _single_bit_index,
)
from digit_template_recognizer import _DIGIT_PATTERNS  # noqa: E402
from plane_binary_mask import PlaneBinaryMask  # noqa: E402
from power_monitor import (  # noqa: E402
    PowerMonitor,
    parse_current_power_sample,
    parse_power_sample,
)
from high_res_refiner import HighResRefiner  # noqa: E402
from plane_mapper import PlaneMapper  # noqa: E402


PIXELS_PER_CM = 8
PLANE_WIDTH_CM = 17.0
PLANE_HEIGHT_CM = 25.7


def make_target(square_specs):
    """Create a rectified target from (cx, cy, side, angle, digit)."""
    width = int(PLANE_WIDTH_CM * PIXELS_PER_CM)
    height = int(PLANE_HEIGHT_CM * PIXELS_PER_CM)
    rows = [bytearray(width) for _ in range(height)]
    for y in range(height):
        plane_y = (y + 0.5) / PIXELS_PER_CM
        for x in range(width):
            plane_x = (x + 0.5) / PIXELS_PER_CM
            for center_x, center_y, side, angle, _ in square_specs:
                cosine = math.cos(angle)
                sine = math.sin(angle)
                delta_x = plane_x - center_x
                delta_y = plane_y - center_y
                local_x = cosine * delta_x + sine * delta_y
                local_y = -sine * delta_x + cosine * delta_y
                if abs(local_x) <= side * 0.5 and abs(local_y) <= side * 0.5:
                    rows[y][x] = 1
                    break

    # Digits are cut out only after all black square unions are drawn.
    for center_x, center_y, side, angle, digit in square_specs:
        if digit is None:
            continue
        pattern = _DIGIT_PATTERNS[digit][0]
        pattern_width = len(pattern[0])
        pattern_height = len(pattern)
        digit_width = side * (0.22 if digit == 1 else 0.40)
        digit_height = side * 0.58
        cosine = math.cos(angle)
        sine = math.sin(angle)
        for y in range(height):
            plane_y = (y + 0.5) / PIXELS_PER_CM
            for x in range(width):
                plane_x = (x + 0.5) / PIXELS_PER_CM
                delta_x = plane_x - center_x
                delta_y = plane_y - center_y
                local_x = cosine * delta_x + sine * delta_y
                local_y = -sine * delta_x + cosine * delta_y
                normal_x = local_x / digit_width + 0.5
                normal_y = local_y / digit_height + 0.5
                if normal_x < 0.0 or normal_y < 0.0:
                    continue
                if normal_x >= 1.0 or normal_y >= 1.0:
                    continue
                column = min(pattern_width - 1, int(normal_x * pattern_width))
                row = min(pattern_height - 1, int(normal_y * pattern_height))
                if pattern[row][column] == "1":
                    rows[y][x] = 0
    return PlaneBinaryMask.from_binary_grid(rows, PIXELS_PER_CM)


def sorted_sides(result):
    return sorted(square["side_cm"] for square in result["squares"])


class CompositeSquareTests(unittest.TestCase):
    def setUp(self):
        self.detector = CompositeSquareDetector(max_candidates=24)

    def assertSidesAlmostEqual(self, measured, expected, tolerance=0.38):
        self.assertEqual(len(measured), len(expected))
        for actual, target in zip(measured, expected):
            self.assertLessEqual(abs(actual - target), tolerance)

    def test_one_hot_index_is_canmv_micropython_compatible(self):
        for index in range(8):
            self.assertEqual(_single_bit_index(1 << index), index)
        source_path = os.path.join(ROOT, "composite_square_detector.py")
        with open(source_path, "r", encoding="utf-8") as source_file:
            self.assertNotIn(".bit_length(", source_file.read())

    def test_axis_aligned_overlap_recovers_hidden_small_square(self):
        mask = make_target((
            (5.3, 5.0, 8.0, 0.0, None),
            (9.1, 7.5, 6.0, 0.0, None),
            (12.6, 10.5, 7.0, 0.0, None),
            (6.4, 19.2, 9.0, 0.0, None),
        ))
        result = self.detector.detect(mask)
        self.assertTrue(result["valid"], result)
        self.assertSidesAlmostEqual(sorted_sides(result), [6.0, 7.0, 8.0, 9.0])
        self.assertGreater(result["coverage_recall"], 0.90)

    def test_single_large_square_reaches_13cm(self):
        mask = make_target(((8.5, 12.8, 13.0, 0.0, None),))
        result = self.detector.detect(mask)
        self.assertTrue(result["valid"], result)
        self.assertSidesAlmostEqual(sorted_sides(result), [13.0], tolerance=0.35)

    def test_rotated_overlap_recovers_multiple_orientation_families(self):
        mask = make_target((
            (5.3, 5.0, 8.0, 0.0, None),
            (9.1, 7.5, 6.0, 0.0, None),
            (12.6, 10.5, 7.0, 0.0, None),
            (8.6, 18.0, 10.0, math.radians(31.0), None),
        ))
        result = self.detector.detect(mask)
        self.assertTrue(result["valid"], result)
        self.assertSidesAlmostEqual(sorted_sides(result), [6.0, 7.0, 8.0, 10.0])
        self.assertGreaterEqual(result["orientation_count"], 2)

    def test_projective_resampling_recovers_tilted_plane_scale(self):
        source = make_target((
            (5.3, 5.0, 8.0, 0.0, None),
            (9.1, 7.5, 6.0, 0.0, None),
            (12.6, 10.5, 7.0, 0.0, None),
            (6.4, 19.2, 9.0, 0.0, None),
        ))
        mapper = PlaneMapper((
            (205.0, 55.0),
            (438.0, 128.0),
            (382.0, 432.0),
            (246.0, 388.0),
        ))

        class ProjectedImage:
            def width(self):
                return 640

            def height(self):
                return 480

            def get_pixel(self, x, y):
                try:
                    plane_x, plane_y = mapper.image_to_plane(x, y)
                except ValueError:
                    return (255, 255, 255)
                if source.is_black_plane(plane_x, plane_y):
                    return (0, 0, 0)
                return (255, 255, 255)

        recovered = PlaneBinaryMask.from_image(
            ProjectedImage(), mapper, pixels_per_cm=PIXELS_PER_CM
        )
        result = self.detector.detect(recovered)
        self.assertTrue(result["valid"], result)
        self.assertSidesAlmostEqual(
            sorted_sides(result), [6.0, 7.0, 8.0, 9.0], tolerance=0.45
        )


class AdvancedTaskTests(unittest.TestCase):
    def setUp(self):
        self.detector = AdvancedTargetDetector()

    def test_auto_min_and_tilt_modes_select_smallest_square(self):
        mask = make_target((
            (4.0, 5.0, 6.0, 0.0, None),
            (12.0, 6.0, 8.0, 0.0, None),
            (5.0, 18.0, 7.0, 0.0, None),
            (12.0, 18.0, 9.0, 0.0, None),
        ))
        for mode in (MODE_AUTO_MIN, MODE_TILT_MIN):
            result = self.detector.detect_mask(mask, mode=mode)
            self.assertTrue(result["shape_valid"], result)
            self.assertAlmostEqual(result["x_cm"], 6.0, delta=0.40)

    def test_tilt_mode_fast_path_keeps_in_plane_rotation(self):
        # The rectified plane can still contain a square rotated in its own
        # plane.  Its axis-aligned bbox is no longer square, so verify that
        # the connected-component path uses an oriented box instead of
        # falling back to the slower union enumerator.
        mask = make_target((
            (8.5, 12.8, 11.5, math.radians(45.0), None),
        ))
        result = self.detector.detect_mask(mask, mode=MODE_TILT_MIN)
        self.assertTrue(result["shape_valid"], result)
        self.assertEqual(result["selection_backend"], "CONNECTED_COMPONENTS")
        self.assertAlmostEqual(result["x_cm"], 11.5, delta=0.35)
        self.assertAlmostEqual(
            result["selected_square"]["component_angle_deg"],
            45.0,
            delta=1.0,
        )

    def test_tilt_mode_covers_rotation_range_after_rectification(self):
        # Projective sheet tilt is removed by PlaneMapper.  This loop checks
        # the remaining in-plane square orientation over the same angular
        # range so the final component box does not assume axis alignment.
        for angle_degrees in (30.0, 35.0, 45.0, 55.0, 60.0):
            mask = make_target(((
                8.5,
                12.8,
                8.0,
                math.radians(angle_degrees),
                None,
            ),))
            result = self.detector.detect_mask(mask, mode=MODE_TILT_MIN)
            self.assertTrue(result["shape_valid"], result)
            self.assertEqual(
                result["selection_backend"], "CONNECTED_COMPONENTS"
            )
            self.assertAlmostEqual(result["x_cm"], 8.0, delta=0.35)
            self.assertAlmostEqual(
                result["selected_square"]["component_angle_deg"],
                angle_degrees,
                delta=1.0,
            )

    def test_tilt_mode_survives_strong_projective_rectification(self):
        source = make_target(((
            8.5,
            12.8,
            9.0,
            math.radians(37.0),
            None,
        ),))
        mapper = PlaneMapper((
            (268.0, 72.0),
            (432.0, 158.0),
            (368.0, 438.0),
            (282.0, 390.0),
        ))

        class ProjectedImage:
            def width(self):
                return 640

            def height(self):
                return 480

            def get_pixel(self, x, y):
                try:
                    plane_x, plane_y = mapper.image_to_plane(x, y)
                except ValueError:
                    return (255, 255, 255)
                if source.is_black_plane(plane_x, plane_y):
                    return (0, 0, 0)
                return (255, 255, 255)

        recovered = PlaneBinaryMask.from_image(
            ProjectedImage(), mapper, pixels_per_cm=PIXELS_PER_CM
        )
        result = self.detector.detect_mask(recovered, mode=MODE_TILT_MIN)
        self.assertTrue(result["shape_valid"], result)
        self.assertEqual(result["selection_backend"], "CONNECTED_COMPONENTS")
        self.assertAlmostEqual(result["x_cm"], 9.0, delta=0.45)

    def test_numbered_target_selects_each_requested_digit(self):
        mask = make_target((
            (3.3, 4.5, 5.0, 0.0, 1),
            (12.5, 5.0, 6.0, 0.0, 2),
            (4.2, 19.0, 7.0, 0.0, 3),
            (12.4, 19.0, 8.0, 0.0, 4),
        ))
        expected = {1: 5.0, 2: 6.0, 3: 7.0, 4: 8.0}
        first_result = None
        for digit in range(1, 5):
            result = self.detector.detect_mask(
                mask,
                mode=MODE_DIGIT_SELECT,
                target_digit=digit,
            )
            self.assertTrue(result["shape_valid"], result)
            self.assertEqual(result["selected_digit"], digit)
            self.assertAlmostEqual(result["x_cm"], expected[digit], delta=0.50)
            if first_result is None:
                first_result = result

        # Numbered squares are separated, so the production path must not pay
        # for overlapping-union hypothesis enumeration.
        self.assertEqual(len(first_result["squares"]), 4)
        self.assertEqual(first_result["raw_candidate_count"], 4)
        self.assertEqual(first_result["selection_candidate_count"], 4)
        self.assertEqual(first_result["selection_subset_count"], 0)
        self.assertEqual(
            first_result["selection_backend"],
            "CONNECTED_COMPONENTS",
        )

    def test_requested_digit_can_use_high_confidence_soft_margin(self):
        mask = make_target((
            (3.3, 4.5, 5.0, 0.0, 1),
            (12.5, 5.0, 6.0, 0.0, 2),
            (4.2, 19.0, 7.0, 0.0, 3),
            (12.4, 19.0, 8.0, 0.0, 4),
        ))

        class SoftRecognizer:
            def recognize_all(self, unused_mask, squares):
                side_to_digit = {5: 1, 6: 2, 7: 3, 8: 4}
                for square in squares:
                    digit = side_to_digit[int(round(square["side_cm"]))]
                    result = {
                        "valid": digit != 3,
                        "digit": digit,
                        "confidence": 0.88 if digit == 3 else 0.95,
                        "margin": 0.003 if digit == 3 else 0.10,
                        "rotation_deg": 0,
                        "reject_reason": (
                            "AMBIGUOUS_DIGIT" if digit == 3 else "OK"
                        ),
                    }
                    square["digit_result"] = result
                    square["digit"] = digit
                    square["digit_confidence"] = result["confidence"]
                    square["digit_margin"] = result["margin"]
                    square["digit_rotation_deg"] = 0

        detector = AdvancedTargetDetector(digit_recognizer=SoftRecognizer())
        result = detector.detect_mask(
            mask,
            mode=MODE_DIGIT_SELECT,
            target_digit=3,
        )
        self.assertTrue(result["shape_valid"], result)
        self.assertEqual(result["digit_match_mode"], "SOFT_TARGET")
        self.assertEqual(result["selected_digit"], 3)

    def test_requested_digit_can_rescue_close_runner_up(self):
        mask = make_target((
            (8.5, 12.8, 6.5, 0.0, 3),
        ))

        class RunnerUpRecognizer:
            def recognize_all(self, unused_mask, squares):
                square = squares[0]
                square["digit_result"] = {
                    "valid": False,
                    "digit": 5,
                    "confidence": 0.875,
                    "margin": 0.008,
                    "second_digit": 3,
                    "second_score": 0.867,
                    "digit_scores": ((5, 0.875), (3, 0.867)),
                    "rotation_deg": 0,
                    "reject_reason": "AMBIGUOUS_DIGIT",
                }
                square["digit"] = 5
                square["digit_confidence"] = 0.875
                square["digit_margin"] = 0.008
                square["digit_rotation_deg"] = 0

        detector = AdvancedTargetDetector(
            digit_recognizer=RunnerUpRecognizer()
        )
        result = detector.detect_mask(
            mask,
            mode=MODE_DIGIT_SELECT,
            target_digit=3,
        )
        self.assertTrue(result["shape_valid"], result)
        self.assertEqual(result["digit_match_mode"], "SOFT_TARGET_RANKED")
        self.assertEqual(result["selected_digit"], 3)

    def test_numbered_target_supports_upside_down_camera_mount(self):
        specs = (
            (3.5, 3.5, 5.5, math.pi, 1),
            (13.5, 3.5, 5.5, math.pi, 2),
            (3.5, 22.0, 5.5, math.pi, 3),
            (13.5, 22.0, 5.5, math.pi, 4),
        )
        mask = make_target(specs)
        expected_centers = {
            digit: (center_x, center_y)
            for center_x, center_y, _, _, digit in specs
        }
        for digit in range(1, 5):
            result = self.detector.detect_mask(
                mask,
                mode=MODE_DIGIT_SELECT,
                target_digit=digit,
            )
            self.assertTrue(result["shape_valid"], result)
            self.assertEqual(result["selected_digit"], digit)
            self.assertEqual(
                result["selected_square"].get("digit_rotation_deg"),
                180,
            )
            expected_x, expected_y = expected_centers[digit]
            actual_x, actual_y = result["plane_center"]
            self.assertAlmostEqual(actual_x, expected_x, delta=0.40)
            self.assertAlmostEqual(actual_y, expected_y, delta=0.40)

    def test_two_frame_gate_checks_size_count_and_identity(self):
        first = {
            "shape_valid": True,
            "mode": MODE_DIGIT_SELECT,
            "selected_digit": 3,
            "x_cm": 7.02,
            "plane_center": (4.0, 18.0),
            "squares": ({}, {}, {}, {}),
        }
        second = dict(first)
        second["x_cm"] = 7.18
        second["plane_center"] = (4.2, 18.1)
        self.assertTrue(advanced_results_consistent(first, second)["valid"])
        second["selected_digit"] = 2
        rejected = advanced_results_consistent(first, second)
        self.assertFalse(rejected["valid"])
        self.assertEqual(rejected["reason"], "DIGIT_IDENTITY")


class PowerMonitorTests(unittest.TestCase):
    class FakeINA226:
        def __init__(self):
            self.reads = 0
            self.resets = 0

        def read_sample(self):
            self.reads += 1
            return {
                "current_a": 0.321,
                "power_w": 1.234,
                "pmax_w": 1.234,
                "overflow": False,
            }

        def reset_peak(self):
            self.resets += 1

    def test_uart_sample_parser_and_peak(self):
        self.assertEqual(parse_power_sample("W=1.25"), 1.25)
        self.assertEqual(parse_power_sample("power=2.5"), 2.5)
        self.assertIsNone(parse_power_sample("W=bad"))
        monitor = PowerMonitor(smoothing=1.0)
        monitor.update_watts(1.2)
        monitor.update_watts(1.8)
        monitor.update_watts(1.4)
        self.assertAlmostEqual(monitor.current_w, 1.4)
        self.assertAlmostEqual(monitor.maximum_w, 1.8)
        monitor.reset_maximum()
        self.assertAlmostEqual(monitor.maximum_w, 1.4)

    def test_current_and_power_sample_updates_screen_values(self):
        self.assertEqual(
            parse_current_power_sample("I=0.823,W=4.12"),
            (0.823, 4.12),
        )
        monitor = PowerMonitor(smoothing=1.0)
        monitor.update_sample(current_a=0.823, watts=4.12)
        status = monitor.status()
        self.assertAlmostEqual(status["current_a"], 0.823)
        self.assertAlmostEqual(status["current_w"], 4.12)
        self.assertEqual(status["maximum_w"], 4.12)

    def test_ina226_backend_is_throttled_and_updates_peak(self):
        backend = self.FakeINA226()
        monitor = PowerMonitor(
            smoothing=1.0,
            hardware_monitor=backend,
            hardware_poll_interval_ms=100,
        )
        self.assertTrue(monitor.poll_hardware(force=True, timestamp_ms=1000))
        self.assertEqual(backend.reads, 1)
        self.assertFalse(monitor.poll_hardware(timestamp_ms=1050))
        self.assertEqual(backend.reads, 1)
        self.assertTrue(monitor.poll_hardware(timestamp_ms=1100))
        self.assertEqual(backend.reads, 2)
        status = monitor.status()
        self.assertEqual(status["source"], "INA226")
        self.assertAlmostEqual(status["current_a"], 0.321)
        self.assertAlmostEqual(status["current_w"], 1.234)
        monitor.reset_maximum()
        self.assertEqual(backend.resets, 1)


class AdvancedIntegrationTests(unittest.TestCase):
    @staticmethod
    def single_shot_tree():
        path = os.path.join(ROOT, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            return ast.parse(file.read())

    def test_uart_parser_exposes_every_task_mode(self):
        tree = self.single_shot_tree()
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "normalize_uart_command"
        )
        namespace = {
            "parse_power_sample": parse_power_sample,
            "parse_current_power_sample": parse_current_power_sample,
        }
        ast.fix_missing_locations(function)
        exec(compile(ast.Module(body=[function], type_ignores=[]), "<uart>", "exec"), namespace)
        parse = namespace["normalize_uart_command"]
        self.assertEqual(parse("M"), "MEASURE")
        self.assertEqual(parse("A"), "ADVANCED_MIN")
        self.assertEqual(parse("N3"), "DIGIT:3")
        self.assertEqual(parse("T"), "TILT_MIN")
        self.assertEqual(parse("PW"), "POWER_STATUS")
        self.assertEqual(parse("R"), "POWER_RESET")
        self.assertEqual(parse("W=1.25"), "POWER_SAMPLE:1.250000")
        self.assertEqual(
            parse("I=0.823,W=4.12"),
            "POWER_SAMPLE_PAIR:0.823000:4.120000",
        )

    def test_measurement_entry_accepts_advanced_dispatch_arguments(self):
        tree = self.single_shot_tree()
        measure_once = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "measure_once"
        )
        argument_names = [item.arg for item in measure_once.args.args]
        self.assertIn("advanced_detector", argument_names)
        self.assertIn("task_mode", argument_names)
        self.assertIn("target_digit", argument_names)

        capture = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "capture_measurement_attempt"
        )
        refine_calls = [
            node
            for node in ast.walk(capture)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "refine"
        ]
        self.assertTrue(refine_calls)
        keyword_names = {
            keyword.arg
            for call in refine_calls
            for keyword in call.keywords
        }
        self.assertIn("preserve_projective_seed", keyword_names)
        self.assertIn("localization_only", keyword_names)
        self.assertIn("max_inner_scale_anisotropy", keyword_names)

    def test_high_refiner_keeps_parallel_mode_backward_compatible(self):
        argument_names = HighResRefiner.refine.__code__.co_varnames[
            : HighResRefiner.refine.__code__.co_argcount
        ]
        self.assertIn("preserve_projective_seed", argument_names)
        self.assertIn("localization_only", argument_names)
        self.assertIn("max_inner_scale_anisotropy", argument_names)


if __name__ == "__main__":
    unittest.main()
