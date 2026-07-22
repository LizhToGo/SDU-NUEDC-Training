import ast
import contextlib
import io
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)
TOOLS_DESKTOP_DIR = os.path.join(PROJECT_DIR, "tools", "desktop")
if TOOLS_DESKTOP_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DESKTOP_DIR)
CALIBRATION_CAPTURE_PATH = os.path.join(
    PROJECT_DIR,
    "tools",
    "k230",
    "distance_calibration_capture.py",
)

from analyze_distance_calibration import (
    DEFAULT_EXPECTED_POINTS,
    fit_reciprocal,
    interpolate_table,
    leave_one_distance_out_table_errors,
    model_errors,
    monotonic_violations,
    summarize,
)
from distance_estimator import (
    DISTANCE_CALIBRATION_POINTS,
    _distance_from_scale,
    _interpolate_table as production_interpolate_table,
)
from measurement_consistency import (
    relative_center_shift,
    relative_scale_difference,
)


class MeasurementConsistencyTests(unittest.TestCase):
    def test_observed_017_first_pair_passes_new_scale_gate(self):
        difference = relative_scale_difference(9.267919, 9.226336)
        self.assertGreater(difference, 0.0035)
        self.assertLess(difference, 0.0050)

    def test_center_gate_accepts_small_motion_and_rejects_position_change(self):
        attempt_1 = (
            (938, 690),
            (1096, 688),
            (1101, 924),
            (944, 929),
        )
        attempt_2 = (
            (937, 683),
            (1095, 678),
            (1101, 916),
            (944, 918),
        )
        attempt_4 = (
            (1069, 679),
            (1225, 676),
            (1232, 918),
            (1076, 911),
        )
        self.assertLess(relative_center_shift(attempt_1, attempt_2), 0.10)
        self.assertGreater(relative_center_shift(attempt_2, attempt_4), 0.50)


class CalibrationUartTriggerTests(unittest.TestCase):
    def _load_collect_consistent_sample(self, capture_attempt, attempts=5):
        path = CALIBRATION_CAPTURE_PATH
        with open(path, "r", encoding="utf-8") as file:
            tree = ast.parse(file.read())
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "collect_consistent_sample"
        )
        module = ast.Module(body=[function], type_ignores=[])
        ast.fix_missing_locations(module)

        class FakeTime:
            current = 0

            @classmethod
            def ticks_ms(cls):
                cls.current += 10
                return cls.current

            @staticmethod
            def ticks_diff(current, previous):
                return current - previous

        class FakeSensor:
            @staticmethod
            def snapshot(chn=None):
                return {"channel": chn}

        def fuse(results):
            scales = [result["frame_scale"] for result in results]
            mean = sum(scales) / len(scales)
            spread = (max(scales) - min(scales)) / mean * 100.0
            return {
                "frame_scale": mean,
                "distance_cm": 130.0,
                "frame_scale_spread_pct": spread,
            }

        namespace = {
            "MAX_MEASUREMENT_ATTEMPTS": attempts,
            "PREVIEW_CH": 1,
            "MAX_FRAME_SCALE_DISAGREEMENT": 0.005,
            "MAX_FRAME_CENTER_SHIFT_RATIO": 0.10,
            "time": FakeTime,
            "sensor": FakeSensor,
            "capture_strong_attempt": capture_attempt,
            "relative_scale_difference": (
                lambda left, right: abs(left - right)
                / ((left + right) * 0.5)
            ),
            "relative_center_shift": lambda left, right: 0.001,
            "high_result_to_preview": lambda result: {
                "strict_id": result["strict_id"],
                "inner_corners": result["inner_corners_float"],
            },
            "fuse_distance_results": fuse,
        }
        exec(compile(module, path, "exec"), namespace)
        return namespace["collect_consistent_sample"]

    def test_calibration_uses_uart1_protocol_without_key_delay(self):
        path = CALIBRATION_CAPTURE_PATH
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
        self.assertEqual(constants["UART_BAUDRATE"], 115200)
        self.assertEqual(constants["TRIGGER_SETTLE_MS"], 0)
        self.assertIn("from ybUtils.YbUart import YbUart", source)
        self.assertNotIn("YbKey", source)
        self.assertIn("ACK MEASURE", source)
        self.assertIn("RESULT OK", source)
        self.assertIn("drain_uart_input()", source)

    def test_calibration_uses_21_five_cm_points_and_one_command_batch(self):
        path = CALIBRATION_CAPTURE_PATH
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()
        tree = ast.parse(source)
        assignments = {}
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if not isinstance(target, ast.Name):
                continue
            try:
                assignments[target.id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                continue

        points = assignments["DISTANCE_POINTS_CM"]
        self.assertEqual(len(points), 21)
        self.assertEqual(points[0], 100.0)
        self.assertEqual(points[-1], 200.0)
        self.assertTrue(all(
            abs(current - previous - 5.0) < 1e-9
            for previous, current in zip(points, points[1:])
        ))
        self.assertEqual(assignments["SAMPLES_PER_DISTANCE"], 5)
        self.assertEqual(assignments["MAX_BATCH_SAMPLE_ATTEMPTS"], 15)
        self.assertIn("distance_calibration_5cm_v3.csv", source)
        self.assertIn("ACK MEASURE BATCH", source)
        self.assertIn("SAMPLE OK", source)
        self.assertIn("RESULT OK", source)

        collection_loops = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.While):
                continue
            if not any(
                isinstance(child, ast.Name)
                and child.id == "MAX_BATCH_SAMPLE_ATTEMPTS"
                for child in ast.walk(node.test)
            ):
                continue
            for child in ast.walk(node):
                if (
                    isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Name)
                    and child.func.id == "collect_consistent_sample"
                ):
                    collection_loops.append(node)
                    break
        self.assertEqual(len(collection_loops), 1)

    def test_calibration_reuses_only_strict_high_resolution_geometry(self):
        path = CALIBRATION_CAPTURE_PATH
        with open(path, "r", encoding="utf-8") as file:
            source = file.read()
        tree = ast.parse(source)
        functions = {
            node.name: node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
        }

        capture = functions["capture_strong_attempt"]
        capture_args = [argument.arg for argument in capture.args.args]
        self.assertIn("tracking_hint", capture_args)
        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "tracking_seed_from_result"
            for node in ast.walk(capture)
        ))

        collect = functions["collect_consistent_sample"]
        collect_args = [argument.arg for argument in collect.args.args]
        self.assertIn("tracking_hint", collect_args)
        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "high_result_to_preview"
            for node in ast.walk(collect)
        ))
        self.assertIn("TRACK_DIRECT", source)
        self.assertIn(
            "tracking hint dropped after rejected refinement",
            source,
        )

        batch_assignments = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not isinstance(node.value, ast.Call):
                continue
            if not isinstance(node.value.func, ast.Name):
                continue
            if node.value.func.id != "collect_consistent_sample":
                continue
            batch_assignments.append(node)
        self.assertEqual(len(batch_assignments), 1)
        target = batch_assignments[0].targets[0]
        self.assertIsInstance(target, ast.Tuple)
        self.assertEqual(
            [element.id for element in target.elts],
            ["record", "preview", "tracking_hint"],
        )

    def test_tracking_seed_contains_preview_geometry_only(self):
        path = CALIBRATION_CAPTURE_PATH
        with open(path, "r", encoding="utf-8") as file:
            tree = ast.parse(file.read())
        wanted = {
            "scale_corners",
            "high_result_to_preview",
            "tracking_seed_from_result",
        }
        body = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name in wanted
        ]
        module = ast.Module(body=body, type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {
            "PREVIEW_WIDTH": 640,
            "PREVIEW_HEIGHT": 360,
            "CAPTURE_WIDTH": 1920,
            "CAPTURE_HEIGHT": 1080,
            "EXPECTED_SHORT_RATIO": 17.0 / 21.0,
            "EXPECTED_LONG_RATIO": 25.7 / 29.7,
        }
        exec(compile(module, path, "exec"), namespace)
        high_result = {
            "mode": "HIGH_RES_INNER",
            "outer_corners": (
                (300, 150),
                (900, 150),
                (900, 750),
                (300, 750),
            ),
            "inner_corners": (
                (360, 210),
                (840, 210),
                (840, 690),
                (360, 690),
            ),
            "image": object(),
        }
        preview = namespace["high_result_to_preview"](high_result)
        seed = namespace["tracking_seed_from_result"](preview)
        self.assertEqual(seed["seed_class"], "TRACK_SEED")
        self.assertEqual(
            seed["inner_corners"],
            ((120, 70), (280, 70), (280, 230), (120, 230)),
        )
        self.assertNotIn("image", preview)

    def test_first_strong_frame_seeds_the_next_independent_attempt(self):
        received_hints = []

        def capture(
            preview,
            detector,
            refiner,
            attempt_index,
            tracking_hint=None,
        ):
            received_hints.append(tracking_hint)
            high_result = {
                "strict_id": attempt_index,
                "mode": "HIGH_RES_INNER",
                "inner_corners_float": (
                    (1.0, 1.0),
                    (2.0, 1.0),
                    (2.0, 2.0),
                    (1.0, 2.0),
                ),
            }
            return {
                "preview": preview,
                "coarse_ms": 1,
                "convert_ms": 2,
                "refine_ms": 3,
                "high_result": high_result,
                "distance_result": {
                    "frame_scale": 10.0 + attempt_index * 0.001,
                },
                "reject": "OK",
                "used_tracking": tracking_hint is not None,
            }

        collect = self._load_collect_consistent_sample(capture, attempts=3)
        with contextlib.redirect_stdout(io.StringIO()):
            record, _, tracking_hint = collect(None, None, None)
        self.assertIsNotNone(record)
        self.assertIsNone(received_hints[0])
        self.assertEqual(received_hints[1]["strict_id"], 1)
        self.assertEqual(tracking_hint["strict_id"], 2)

    def test_rejected_tracked_refinement_returns_to_cold_search(self):
        initial_hint = {"strict_id": 99, "inner_corners": ((0, 0),) * 4}
        received_hints = []

        def capture(
            preview,
            detector,
            refiner,
            attempt_index,
            tracking_hint=None,
        ):
            received_hints.append(tracking_hint)
            if attempt_index == 1:
                return {
                    "preview": preview,
                    "coarse_ms": 0,
                    "convert_ms": 1,
                    "refine_ms": 1,
                    "high_result": None,
                    "distance_result": None,
                    "reject": "INNER_RMS",
                    "used_tracking": True,
                }
            high_result = {
                "strict_id": attempt_index,
                "mode": "HIGH_RES_INNER",
                "inner_corners_float": (
                    (1.0, 1.0),
                    (2.0, 1.0),
                    (2.0, 2.0),
                    (1.0, 2.0),
                ),
            }
            return {
                "preview": preview,
                "coarse_ms": 1,
                "convert_ms": 2,
                "refine_ms": 3,
                "high_result": high_result,
                "distance_result": {"frame_scale": 10.0},
                "reject": "OK",
                "used_tracking": False,
            }

        collect = self._load_collect_consistent_sample(capture, attempts=2)
        with contextlib.redirect_stdout(io.StringIO()):
            record, _, tracking_hint = collect(
                None,
                None,
                None,
                tracking_hint=initial_hint,
            )
        self.assertIsNone(record)
        self.assertIs(received_hints[0], initial_hint)
        self.assertIsNone(received_hints[1])
        self.assertEqual(tracking_hint["strict_id"], 2)

    def test_uart_command_normalization(self):
        path = CALIBRATION_CAPTURE_PATH
        with open(path, "r", encoding="utf-8") as file:
            tree = ast.parse(file.read())
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "normalize_uart_command"
        )
        module = ast.Module(body=[function], type_ignores=[])
        ast.fix_missing_locations(module)
        namespace = {}
        exec(compile(module, path, "exec"), namespace)
        normalize = namespace["normalize_uart_command"]
        self.assertEqual(normalize("m"), "MEASURE")
        self.assertEqual(normalize("MEASURE"), "MEASURE")
        self.assertEqual(normalize("1"), "MEASURE")
        self.assertEqual(normalize("status"), "STATUS")
        self.assertEqual(normalize("p"), "PING")
        self.assertTrue(normalize("invalid").startswith("UNKNOWN:"))

    def test_single_shot_uses_uart1_protocol_without_onboard_key(self):
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
        self.assertEqual(constants["UART_BAUDRATE"], 115200)
        self.assertIn("from ybUtils.YbUart import YbUart", source)
        self.assertNotIn("YbKey", source)
        self.assertNotIn("key.is_pressed", source)
        self.assertIn("READY SINGLE_SHOT_MEASUREMENT", source)
        self.assertIn("ACK MEASURE", source)
        self.assertIn("RESULT OK", source)
        self.assertIn("RESULT RETRY", source)
        self.assertIn("drain_uart_input()", source)


class CalibrationAnalysisTests(unittest.TestCase):
    def test_analyzer_requires_the_new_21_point_grid_by_default(self):
        self.assertEqual(DEFAULT_EXPECTED_POINTS, 21)

    def test_reciprocal_fit_recovers_known_model(self):
        summaries = []
        for distance in (100.0, 120.0, 150.0, 180.0, 200.0):
            scale = 1160.0 / (distance + 2.0)
            summaries.append({
                "distance_cm": distance,
                "median_scale": scale,
            })
        coefficient, intercept = fit_reciprocal(
            summaries,
            with_intercept=True,
        )
        self.assertAlmostEqual(coefficient, 1160.0, places=6)
        self.assertAlmostEqual(intercept, -2.0, places=6)
        self.assertLess(max(abs(x) for x in model_errors(
            summaries,
            coefficient,
            intercept,
        )), 1e-9)

    def test_summary_uses_median_and_detects_monotonic_data(self):
        records = []
        for scale in (11.4, 11.5, 20.0):
            records.append({
                "distance_cm": 100.0,
                "frame_scale": scale,
                "scale_spread_pct": 0.2,
                "center_shift_ratio": 0.02,
                "total_ms": 1800,
            })
        for scale in (5.7, 5.8, 5.9):
            records.append({
                "distance_cm": 200.0,
                "frame_scale": scale,
                "scale_spread_pct": 0.2,
                "center_shift_ratio": 0.02,
                "total_ms": 1800,
            })
        summaries = summarize(records)
        self.assertEqual(summaries[0]["median_scale"], 11.5)
        self.assertEqual(summaries[1]["median_scale"], 5.8)
        self.assertEqual(monotonic_violations(summaries), [])

    def test_table_cross_validation_matches_production_interpolation(self):
        coefficient = 1160.0
        intercept_offset = 2.0
        summaries = []
        calibration_points = []
        for distance_cm in (100.0, 150.0, 200.0):
            scale = coefficient / (distance_cm + intercept_offset)
            summaries.append({
                "distance_cm": distance_cm,
                "median_scale": scale,
            })
            calibration_points.append((distance_cm, scale))

        test_scale = coefficient / (175.0 + intercept_offset)
        analyzed = interpolate_table(test_scale, summaries)
        produced = production_interpolate_table(
            test_scale,
            calibration_points,
        )
        self.assertAlmostEqual(analyzed, 175.0)
        self.assertAlmostEqual(produced, analyzed)
        for error in leave_one_distance_out_table_errors(summaries):
            self.assertAlmostEqual(error, 0.0, places=9)

    def test_final_calibration_table_is_installed_and_monotonic(self):
        expected = (
            (100.0, 10.783195),
            (105.0, 10.307374),
            (110.0, 9.871771),
            (115.0, 9.471493),
            (120.0, 9.102411),
            (125.0, 8.769537),
            (130.0, 8.460150),
            (135.0, 8.165320),
            (140.0, 7.890348),
            (145.0, 7.626735),
            (150.0, 7.380167),
            (155.0, 7.146226),
            (160.0, 6.926661),
            (165.0, 6.732378),
            (170.0, 6.548696),
            (175.0, 6.379375),
            (180.0, 6.218590),
            (185.0, 6.058420),
            (190.0, 5.906293),
            (195.0, 5.761619),
            (200.0, 5.623863),
        )
        self.assertEqual(DISTANCE_CALIBRATION_POINTS, expected)
        for previous, current in zip(
            DISTANCE_CALIBRATION_POINTS,
            DISTANCE_CALIBRATION_POINTS[1:],
        ):
            self.assertGreater(current[0], previous[0])
            self.assertLess(current[1], previous[1])

        distance_cm, method = _distance_from_scale(7.890348)
        self.assertAlmostEqual(distance_cm, 140.0)
        self.assertEqual(method, "CALIBRATION_TABLE")


if __name__ == "__main__":
    unittest.main()
