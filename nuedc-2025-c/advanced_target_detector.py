"""Task-level detector for every 2025 C-problem advanced target mode."""

import math
import time

from plane_binary_mask import PlaneBinaryMask
from composite_square_detector import CompositeSquareDetector
from digit_template_recognizer import DigitTemplateRecognizer
from numbered_square_detector import NumberedSquareDetector


MODE_AUTO_MIN = "AUTO_MIN"
MODE_DIGIT_SELECT = "DIGIT_SELECT"
MODE_TILT_MIN = "TILT_MIN"

# The physical numbered target currently contains only a few high-contrast
# glyphs, and the touch screen already tells us which one is requested.  At
# long range a resampled ``3`` can score a few hundredths below an unsupported
# ``5`` even though its own template score is strong.  Permit that narrowly
# target-aware case; geometry, two-frame identity and the requested digit all
# remain mandatory, so this is not a general classifier relaxation.
TARGET_AWARE_MIN_SCORE = 0.82
TARGET_AWARE_MAX_GAP = 0.06


def _ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.time() * 1000.0)


def _ticks_diff(current, previous):
    try:
        return time.ticks_diff(current, previous)
    except AttributeError:
        return current - previous


def _distance(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return math.sqrt(dx * dx + dy * dy)


def _copy_square_for_scale(square, scale_x, scale_y):
    copied = dict(square)
    image_corners = square.get("image_corners")
    if image_corners is not None:
        copied["image_corners"] = tuple(
            (point[0] * scale_x, point[1] * scale_y)
            for point in image_corners
        )
    # Plane coordinates, centimetre dimensions and digit results do not scale.
    return copied


def scale_advanced_result(result, scale_x, scale_y):
    if result is None:
        return None
    copied = dict(result)
    squares = []
    for square in result.get("squares", ()):
        squares.append(_copy_square_for_scale(square, scale_x, scale_y))
    copied["squares"] = tuple(squares)
    selected_index = result.get("selected_index")
    if selected_index is not None and selected_index < len(squares):
        copied["selected_square"] = squares[selected_index]
        copied["image_corners"] = squares[selected_index].get(
            "image_corners"
        )
    elif result.get("image_corners") is not None:
        copied["image_corners"] = tuple(
            (point[0] * scale_x, point[1] * scale_y)
            for point in result["image_corners"]
        )
    return copied


def draw_advanced_overlay(
    image,
    result,
    selected_color=(255, 40, 40),
    other_color=(0, 255, 80),
    thickness=2,
):
    if result is None:
        return
    squares = result.get("squares", ())
    selected_index = result.get("selected_index")
    for square_index in range(len(squares)):
        corners = squares[square_index].get("image_corners")
        if corners is None or len(corners) != 4:
            continue
        is_selected = square_index == selected_index
        color = selected_color if is_selected else other_color
        line_thickness = thickness + 1 if is_selected else thickness
        for index in range(4):
            first = corners[index]
            second = corners[(index + 1) % 4]
            image.draw_line(
                int(round(first[0])),
                int(round(first[1])),
                int(round(second[0])),
                int(round(second[1])),
                color=color,
                thickness=line_thickness,
            )
        try:
            center_x = sum(point[0] for point in corners) / 4.0
            center_y = sum(point[1] for point in corners) / 4.0
            image.draw_cross(
                int(round(center_x)),
                int(round(center_y)),
                color=color,
                size=5,
                thickness=1,
            )
        except Exception:
            pass


def advanced_results_consistent(
    first,
    second,
    maximum_side_difference_cm=0.30,
    maximum_center_shift_cm=0.75,
):
    result = {
        "valid": False,
        "reason": "MISSING_RESULT",
        "side_difference_cm": 999.0,
        "center_shift_cm": 999.0,
        "count_first": 0,
        "count_second": 0,
    }
    if first is None or second is None:
        return result
    if not first.get("shape_valid", False) or not second.get(
        "shape_valid", False
    ):
        result["reason"] = "INVALID_RESULT"
        return result
    first_count = len(first.get("squares", ()))
    second_count = len(second.get("squares", ()))
    result["count_first"] = first_count
    result["count_second"] = second_count
    if first_count != second_count:
        result["reason"] = "SQUARE_COUNT"
        return result
    if first.get("mode") != second.get("mode"):
        result["reason"] = "MODE"
        return result
    if first.get("mode") == MODE_DIGIT_SELECT:
        if first.get("selected_digit") != second.get("selected_digit"):
            result["reason"] = "DIGIT_IDENTITY"
            return result
    side_difference = abs(first.get("x_cm", 0.0) - second.get("x_cm", 0.0))
    result["side_difference_cm"] = side_difference
    first_center = first.get("plane_center")
    second_center = second.get("plane_center")
    center_shift = 999.0
    if first_center is not None and second_center is not None:
        center_shift = _distance(first_center, second_center)
    result["center_shift_cm"] = center_shift
    if side_difference > maximum_side_difference_cm:
        result["reason"] = "SIDE_DIFFERENCE"
        return result
    if center_shift > maximum_center_shift_cm:
        result["reason"] = "CENTER_SHIFT"
        return result
    result["valid"] = True
    result["reason"] = "OK"
    return result


class AdvancedTargetDetector:
    def __init__(
        self,
        pixels_per_cm=8.0,
        square_detector=None,
        digit_recognizer=None,
    ):
        self.pixels_per_cm = float(pixels_per_cm)
        self.square_detector = (
            CompositeSquareDetector()
            if square_detector is None
            else square_detector
        )
        self.digit_recognizer = (
            DigitTemplateRecognizer()
            if digit_recognizer is None
            else digit_recognizer
        )
        # Numbered targets are normally four separated black tiles.  The
        # connected-component path is both faster and less prone to losing the
        # smallest tile to the composite detector's bounded hypothesis budget.
        self.numbered_square_detector = NumberedSquareDetector()
        self.last_mask = None

    @staticmethod
    def _failure(mode, reason, elapsed_ms=0, target_digit=None):
        return {
            "shape_valid": False,
            "shape_type": "UNKNOWN",
            "x_cm": 0.0,
            "confidence": 0.0,
            "reject_reason": reason,
            "mode": mode,
            "target_digit": target_digit,
            "selected_digit": None,
            "selected_index": None,
            "selected_square": None,
            "squares": (),
            "image_corners": None,
            "plane_center": None,
            "fill_ratio": 0.0,
            "width_cm": 0.0,
            "height_cm": 0.0,
            "area_cm2": 0.0,
            "advanced_ms": elapsed_ms,
        }

    @staticmethod
    def _attach_square_diagnostics(result, square_result):
        """Expose bounded-search telemetry to UART and metadata callers."""
        result["square_detection"] = square_result
        for key in (
            "boundary_count",
            "raw_candidate_count",
            "raw_generated_count",
            "raw_complete_count",
            "raw_partial_count",
            "scored_candidate_count",
            "candidate_count",
            "orientation_count",
            "orientations_deg",
            "selection_candidate_count",
            "selection_subset_count",
            "selection_backend",
            "score_sample_count",
            "selection_stats",
            "timing_ms",
            "explanation_score",
            "coverage_recall",
            "coverage_precision",
        ):
            if key in square_result:
                result[key] = square_result[key]
        return result

    def detect(
        self,
        image,
        mapper,
        inner_corners=None,
        mode=MODE_AUTO_MIN,
        target_digit=None,
    ):
        start = _ticks_ms()
        try:
            mask = PlaneBinaryMask.from_image(
                image,
                mapper,
                pixels_per_cm=self.pixels_per_cm,
            )
        except Exception as error:
            return self._failure(
                mode,
                "PLANE_MASK_EXCEPTION:%s" % repr(error),
                _ticks_diff(_ticks_ms(), start),
                target_digit,
            )
        return self.detect_mask(
            mask,
            mapper=mapper,
            mode=mode,
            target_digit=target_digit,
            start_ms=start,
        )

    def detect_mask(
        self,
        mask,
        mapper=None,
        mode=MODE_AUTO_MIN,
        target_digit=None,
        start_ms=None,
    ):
        if start_ms is None:
            start_ms = _ticks_ms()
        self.last_mask = mask
        if mode not in (MODE_AUTO_MIN, MODE_DIGIT_SELECT, MODE_TILT_MIN):
            return self._failure(
                mode,
                "UNKNOWN_ADVANCED_MODE",
                _ticks_diff(_ticks_ms(), start_ms),
                target_digit,
            )
        fast_numbered = None
        if mode in (MODE_DIGIT_SELECT, MODE_TILT_MIN):
            fast_numbered = self.numbered_square_detector.detect(
                mask,
                mapper=mapper,
                minimum_candidates=(
                    4 if mode == MODE_DIGIT_SELECT else 1
                ),
                # Numbered sheets are normally rectified/front-facing and use
                # the cheap axis-aligned component path.  The tilted task may
                # contain an in-plane rotated square, so enable the bounded
                # oriented-box sweep only there.
                oriented=(mode == MODE_TILT_MIN),
            )
            if mode == MODE_DIGIT_SELECT:
                # Keep the normal numbered path at ~30 ms.  If its cheap
                # axis-aligned components look suspicious (or do not reach
                # four tiles), retry the same mask with the bounded oriented
                # sweep before paying for the general overlap detector.  A
                # rotated filled square has a markedly lower bbox fill ratio.
                suspicious_rotation = not fast_numbered.get(
                    "valid", False
                )
                if fast_numbered.get("valid", False):
                    for square in fast_numbered.get("squares", ()):
                        if square.get("fill_ratio", 1.0) < 0.72:
                            suspicious_rotation = True
                            break
                if suspicious_rotation:
                    oriented_numbered = self.numbered_square_detector.detect(
                        mask,
                        mapper=mapper,
                        minimum_candidates=4,
                        oriented=True,
                    )
                    if oriented_numbered.get("valid", False):
                        fast_numbered = oriented_numbered
        if fast_numbered is not None and fast_numbered.get("valid", False):
            square_result = fast_numbered
        else:
            square_result = self.square_detector.detect(mask, mapper=mapper)
        if not square_result.get("valid", False):
            failure = self._failure(
                mode,
                "SQUARE_%s" % square_result.get(
                    "reject_reason", "UNKNOWN"
                ),
                _ticks_diff(_ticks_ms(), start_ms),
                target_digit,
            )
            failure["squares"] = square_result.get("squares", ())
            return self._attach_square_diagnostics(failure, square_result)

        squares = list(square_result["squares"])
        selected = None
        digit_match_mode = None
        if mode == MODE_DIGIT_SELECT:
            if target_digit is None:
                return self._failure(
                    mode,
                    "TARGET_DIGIT_REQUIRED",
                    _ticks_diff(_ticks_ms(), start_ms),
                    target_digit,
                )
            target_digit = int(target_digit)
            self.digit_recognizer.recognize_all(mask, squares)
            matching, digit_match_mode = self._digit_matches(
                squares, target_digit
            )
            # A separated numbered sheet should be handled entirely by the
            # fast path.  If its target glyph is genuinely absent, retain the
            # old overlapping-union detector as a fallback for composite
            # layouts (or for a threshold that accidentally merged tiles).
            if not matching and fast_numbered is not None and fast_numbered.get(
                "valid", False
            ):
                fallback_result = self.square_detector.detect(
                    mask, mapper=mapper
                )
                if fallback_result.get("valid", False):
                    square_result = fallback_result
                    squares = list(square_result.get("squares", ()))
                    self.digit_recognizer.recognize_all(mask, squares)
                    matching, digit_match_mode = self._digit_matches(
                        squares, target_digit
                    )
            if not matching:
                failure = self._failure(
                    mode,
                    "TARGET_DIGIT_NOT_FOUND",
                    _ticks_diff(_ticks_ms(), start_ms),
                    target_digit,
                )
                failure["squares"] = tuple(squares)
                return self._attach_square_diagnostics(
                    failure, square_result
                )
            matching.sort(reverse=True, key=lambda item: item[0])
            selected_index = matching[0][1]
            selected = matching[0][2]
            selected["digit_match_mode"] = digit_match_mode
        else:
            selected_index = 0
            selected = squares[0]
            for index in range(1, len(squares)):
                if squares[index]["side_cm"] < selected["side_cm"]:
                    selected_index = index
                    selected = squares[index]

        digit_confidence = selected.get("digit_confidence", 1.0)
        confidence = (
            0.45 * selected.get("score", 0.0)
            + 0.45 * square_result.get("explanation_score", 0.0)
            + 0.10 * digit_confidence
        )
        side = selected["side_cm"]
        shape_type = "SQUARE_MIN"
        selected_digit = None
        if mode == MODE_DIGIT_SELECT:
            selected_digit = target_digit
            shape_type = "SQUARE_N%d" % target_digit
        elif mode == MODE_TILT_MIN:
            shape_type = "SQUARE_TILT_MIN"
        elapsed = _ticks_diff(_ticks_ms(), start_ms)
        result = {
            "shape_valid": True,
            "shape_type": shape_type,
            "x_cm": side,
            "confidence": confidence,
            "reject_reason": "OK",
            "mode": mode,
            "target_digit": target_digit,
            "selected_digit": selected_digit,
            "digit_match_mode": digit_match_mode,
            "selected_index": selected_index,
            "selected_square": selected,
            "squares": tuple(squares),
            "square_count": len(squares),
            "candidate_count": square_result.get("candidate_count", 0),
            "orientation_count": square_result.get(
                "orientation_count", 0
            ),
            "orientations_deg": square_result.get("orientations_deg", ()),
            "explanation_score": square_result.get(
                "explanation_score", 0.0
            ),
            "coverage_recall": square_result.get("coverage_recall", 0.0),
            "coverage_precision": square_result.get(
                "coverage_precision", 0.0
            ),
            "mask_threshold": mask.threshold,
            "mask_black_fraction": mask.black_fraction(),
            "image_corners": selected.get("image_corners"),
            "plane_center": selected.get("center"),
            "plane_corners": selected.get("plane_corners"),
            "fill_ratio": selected.get("fill_ratio", 0.0),
            "edge_support": selected.get("edge_support", 0.0),
            "visible_sides": selected.get("visible_sides", 0),
            "width_cm": side,
            "height_cm": side,
            "area_cm2": side * side,
            "advanced_ms": elapsed,
        }
        return self._attach_square_diagnostics(result, square_result)

    def _digit_matches(self, squares, target_digit):
        """Return target candidates with strict then target-aware soft gates.

        ``DigitTemplateRecognizer`` intentionally rejects low-margin glyphs
        when it is used as a general classifier.  In this task the touch
        screen already supplies the desired number, so a high-confidence
        top-ranked target with a small margin is useful information rather
        than an automatic failure.  A close, high-scoring runner-up is also
        accepted through the target-aware path; this covers the observed
        long-range 3/5 confusion without globally weakening classification.
        Both soft gates are scoped to the requested digit and remain
        protected by the existing two-frame identity gate.
        """
        strict = []
        soft = []
        target_aware = []
        for index in range(len(squares)):
            square = squares[index]
            digit_result = square.get("digit_result", {})
            confidence = digit_result.get("confidence", 0.0)
            margin = digit_result.get("margin", 0.0)
            rank = confidence + max(0.0, margin)
            if digit_result.get("digit") == target_digit:
                item = (rank, index, square)
                if digit_result.get("valid", False):
                    strict.append(item)
                elif (
                    confidence >= 0.84
                    and margin >= 0.0
                    and digit_result.get("reject_reason")
                    in ("AMBIGUOUS_DIGIT", "WEAK_DIGIT_SCORE")
                ):
                    soft.append(item)

            # Target-aware rescue: use the complete score vector when the
            # requested digit is the close runner-up.  This specifically
            # handles the real 3/5 ambiguity seen in low-resolution captures;
            # do not accept a distant or weak runner-up.
            target_score = None
            for scored_digit, scored_value in digit_result.get(
                "digit_scores", ()
            ):
                if int(scored_digit) == int(target_digit):
                    target_score = float(scored_value)
                    break
            if target_score is None and digit_result.get(
                "second_digit"
            ) == target_digit:
                target_score = float(digit_result.get("second_score", 0.0))
            if target_score is not None:
                gap = float(confidence) - target_score
                if (
                    target_score >= TARGET_AWARE_MIN_SCORE
                    and gap >= 0.0
                    and gap <= TARGET_AWARE_MAX_GAP
                ):
                    square["digit_target_score"] = target_score
                    square["digit_target_gap"] = gap
                    target_aware.append((target_score - gap, index, square))
        if strict:
            strict.sort(reverse=True, key=lambda item: item[0])
            return strict, "STRICT"
        if soft:
            soft.sort(reverse=True, key=lambda item: item[0])
            return soft, "SOFT_TARGET"
        if target_aware:
            target_aware.sort(reverse=True, key=lambda item: item[0])
            return target_aware, "SOFT_TARGET_RANKED"
        return [], None
