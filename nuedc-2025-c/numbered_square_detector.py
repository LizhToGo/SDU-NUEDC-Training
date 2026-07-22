"""Fast connected-component detector for the separated numbered squares.

The composite detector is intentionally general: it explains overlapping
square unions by enumerating line hypotheses.  That is the right tool for the
random/overlapping target, but it is unnecessary for the numbered target when
the printed squares are separated.  In that case every black square is an
independent component in the rectified A4 plane.  This module extracts those
components directly, estimates a bounded oriented bounding box (so an in-plane
30--60 degree square is not rejected by its axis-aligned bbox), and returns the
same candidate dictionary shape used by ``CompositeSquareDetector``.

Only the standard library and the compact :class:`PlaneBinaryMask` interface
are used, so the module runs on CanMV MicroPython without NumPy/OpenCV.
"""

import math
import time


MIN_NUMBERED_SIDE_CM = 4.2
MAX_NUMBERED_SIDE_CM = 13.8
MIN_COMPONENT_FILL = 0.42
MAX_COMPONENT_ASPECT = 1.28
ORIENTATION_COARSE_STEP_DEG = 5
ORIENTATION_FINE_STEP_DEG = 1


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


def _component_corners(x0, y0, side):
    return (
        (x0, y0),
        (x0 + side, y0),
        (x0 + side, y0 + side),
        (x0, y0 + side),
    )


class NumberedSquareDetector:
    """Extract separated black square components from a plane mask."""

    def __init__(
        self,
        min_side_cm=MIN_NUMBERED_SIDE_CM,
        max_side_cm=MAX_NUMBERED_SIDE_CM,
        min_fill=MIN_COMPONENT_FILL,
        max_aspect=MAX_COMPONENT_ASPECT,
    ):
        self.min_side_cm = float(min_side_cm)
        self.max_side_cm = float(max_side_cm)
        self.min_fill = float(min_fill)
        self.max_aspect = float(max_aspect)

    @staticmethod
    def _oriented_bounds(pixel_indices, width):
        """Find a small oriented bounding box in the rectified plane.

        A filled square has nearly isotropic second moments, so PCA cannot
        reliably recover its rotation.  A bounded projection sweep is both
        simpler and more stable on the tiny 8-pixel/cm mask: first scan 5
        degree bins, then refine the best bin at 1 degree.  The returned angle
        follows the transform used by ``DigitTemplateRecognizer``.
        """
        if not pixel_indices:
            return None

        def evaluate(angle_degrees):
            angle = math.radians(angle_degrees)
            cosine = math.cos(angle)
            sine = math.sin(angle)
            minimum_u = 1.0e30
            maximum_u = -1.0e30
            minimum_v = 1.0e30
            maximum_v = -1.0e30
            for index in pixel_indices:
                x = index % width + 0.5
                y = index // width + 0.5
                u = x * cosine + y * sine
                v = -x * sine + y * cosine
                minimum_u = min(minimum_u, u)
                maximum_u = max(maximum_u, u)
                minimum_v = min(minimum_v, v)
                maximum_v = max(maximum_v, v)
            width_px = max(1.0, maximum_u - minimum_u + 1.0)
            height_px = max(1.0, maximum_v - minimum_v + 1.0)
            area = width_px * height_px
            return {
                "angle_degrees": angle_degrees,
                "angle_rad": angle,
                "minimum_u": minimum_u,
                "maximum_u": maximum_u,
                "minimum_v": minimum_v,
                "maximum_v": maximum_v,
                "width_px": width_px,
                "height_px": height_px,
                "area": area,
            }

        best = None
        angle = 0
        while angle < 90:
            candidate = evaluate(angle)
            if best is None or candidate["area"] < best["area"]:
                best = candidate
            angle += ORIENTATION_COARSE_STEP_DEG

        start_angle = max(
            0,
            int(round(best["angle_degrees"]))
            - ORIENTATION_COARSE_STEP_DEG,
        )
        end_angle = min(
            89,
            int(round(best["angle_degrees"]))
            + ORIENTATION_COARSE_STEP_DEG,
        )
        angle = start_angle
        while angle <= end_angle:
            candidate = evaluate(angle)
            if candidate["area"] < best["area"]:
                best = candidate
            angle += ORIENTATION_FINE_STEP_DEG
        return best

    @classmethod
    def _component(cls, mask, start, visited, oriented=True):
        """Return one 8-connected component and its oriented bounds."""
        width = mask.width
        height = mask.height
        queue = [start]
        visited[start] = 1
        cursor = 0
        count = 0
        min_x = width
        min_y = height
        max_x = -1
        max_y = -1
        # Eight-connectivity tolerates a one-pixel threshold break at a corner.
        while cursor < len(queue):
            index = queue[cursor]
            cursor += 1
            x = index % width
            y = index // width
            count += 1
            if x < min_x:
                min_x = x
            if x > max_x:
                max_x = x
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= height:
                    continue
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx = x + dx
                    if nx < 0 or nx >= width:
                        continue
                    neighbour = ny * width + nx
                    if mask.black[neighbour] and not visited[neighbour]:
                        visited[neighbour] = 1
                        queue.append(neighbour)
        return {
            "count": count,
            "min_x": min_x,
            "min_y": min_y,
            "max_x": max_x,
            "max_y": max_y,
            "oriented": (
                cls._oriented_bounds(queue, width) if oriented else None
            ),
        }

    def _candidate_from_component(self, mask, component, oriented=True):
        count = component["count"]
        min_x = component["min_x"]
        min_y = component["min_y"]
        max_x = component["max_x"]
        max_y = component["max_y"]
        if max_x < min_x or max_y < min_y:
            return None
        width_px = max_x - min_x + 1
        height_px = max_y - min_y + 1
        axis_short_px = min(width_px, height_px)
        axis_long_px = max(width_px, height_px)
        if axis_short_px <= 0:
            return None
        oriented_bounds = component.get("oriented")
        if oriented_bounds is None:
            oriented_width = float(width_px)
            oriented_height = float(height_px)
        else:
            oriented_width = oriented_bounds["width_px"]
            oriented_height = oriented_bounds["height_px"]
        short_px = min(oriented_width, oriented_height)
        long_px = max(oriented_width, oriented_height)
        aspect = long_px / float(max(short_px, 1.0))
        if aspect > self.max_aspect:
            return None
        fill_ratio = count / float(oriented_width * oriented_height)
        if fill_ratio < self.min_fill:
            return None

        pixels_per_cm = max(float(mask.pixels_per_cm), 1.0e-6)
        side_cm = math.sqrt(oriented_width * oriented_height) / pixels_per_cm
        if (
            side_cm < self.min_side_cm
            or side_cm > self.max_side_cm
        ):
            return None

        # The recognizer samples a local (u,v) square and rotates it back to
        # plane coordinates.  Keep the oriented projection bounds in those
        # local coordinates so the same path handles 0--90 degree squares.
        if oriented_bounds is None:
            angle_rad = 0.0
            minimum_u = min_x / pixels_per_cm
            maximum_u = (max_x + 1.0) / pixels_per_cm
            minimum_v = min_y / pixels_per_cm
            maximum_v = (max_y + 1.0) / pixels_per_cm
            oriented_angle_degrees = 0.0
        else:
            angle_rad = oriented_bounds["angle_rad"]
            minimum_u = oriented_bounds["minimum_u"] / pixels_per_cm
            maximum_u = oriented_bounds["maximum_u"] / pixels_per_cm
            minimum_v = oriented_bounds["minimum_v"] / pixels_per_cm
            maximum_v = oriented_bounds["maximum_v"] / pixels_per_cm
            oriented_angle_degrees = oriented_bounds["angle_degrees"]
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)

        def plane_point(u, v):
            return (
                cosine * u - sine * v,
                sine * u + cosine * v,
            )

        corners = (
            plane_point(minimum_u, minimum_v),
            plane_point(maximum_u, minimum_v),
            plane_point(maximum_u, maximum_v),
            plane_point(minimum_u, maximum_v),
        )
        # Check the actual oriented square corners rather than its axis-
        # aligned component bbox.  A 45-degree square can legitimately span
        # almost the full mask width while all four physical corners remain
        # well inside the A4 aperture.
        corner_min_x = min(point[0] for point in corners) * pixels_per_cm
        corner_min_y = min(point[1] for point in corners) * pixels_per_cm
        corner_max_x = max(point[0] for point in corners) * pixels_per_cm
        corner_max_y = max(point[1] for point in corners) * pixels_per_cm
        corner_margin = 0.5 if abs(angle_rad) > 1.0e-6 else 1.0
        if (
            corner_min_x <= corner_margin
            or corner_min_y <= corner_margin
            or corner_max_x >= mask.width - corner_margin
            or corner_max_y >= mask.height - corner_margin
        ):
            return None
        center = plane_point(
            (minimum_u + maximum_u) * 0.5,
            (minimum_v + maximum_v) * 0.5,
        )
        x0 = minimum_u
        y0 = minimum_v
        # Score is only used for deterministic ordering; digit identity is
        # decided by the template recognizer after all components are found.
        aspect_score = max(0.0, 1.0 - (aspect - 1.0) / 0.28)
        score = 0.55 * min(1.0, fill_ratio / 0.82)
        score += 0.35 * aspect_score
        score += 0.10 * min(
            1.0,
            count / float(max(oriented_width * oriented_height, 1.0)),
        )
        return {
            "x0": x0,
            "y0": y0,
            "side_cm": side_cm,
            "angle_rad": angle_rad,
            "line_evidence": 1.0,
            "_proposal_quality": score,
            "_proposal_complete": True,
            "_proposal_center": center,
            "center": center,
            "plane_corners": corners,
            "fill_ratio": fill_ratio,
            "edge_support": aspect_score,
            "visible_edge_total": aspect_score * 4.0,
            "side_supports": (aspect_score,) * 4,
            "occluded_supports": (0.0, 0.0, 0.0, 0.0),
            "inside_ratios": (fill_ratio,) * 4,
            "visible_sides": 4,
            "strong_sides": 4 if aspect_score >= 0.65 else 3,
            "corner_support": aspect_score,
            "exposed_corners": 4,
            "strong_two_edge": True,
            "edge_continuation_ratio": 0.0,
            "score_samples": width_px * height_px,
            "score": score,
            "component_pixels": count,
            "component_bbox_px": (min_x, min_y, width_px, height_px),
            "component_obb_px": (
                minimum_u * pixels_per_cm,
                minimum_v * pixels_per_cm,
                oriented_width,
                oriented_height,
            ),
            "component_aspect": aspect,
            "component_angle_deg": oriented_angle_degrees,
        }

    def detect(
        self,
        mask,
        mapper=None,
        minimum_candidates=2,
        oriented=True,
    ):
        start = _ticks_ms()
        result = {
            "valid": False,
            "reject_reason": "UNKNOWN",
            "squares": (),
            "candidate_count": 0,
            "orientation_count": 1,
            "orientations_deg": (0.0,),
            "explanation_score": 0.0,
            "coverage_recall": 0.0,
            "coverage_precision": 0.0,
            "black_fraction": mask.black_fraction(),
            "threshold": mask.threshold,
            "boundary_count": mask.black_count,
            "raw_candidate_count": 0,
            "raw_generated_count": 0,
            "raw_complete_count": 0,
            "raw_partial_count": 0,
            "scored_candidate_count": 0,
            "selection_candidate_count": 0,
            "selection_subset_count": 0,
            "selection_backend": "CONNECTED_COMPONENTS",
            "score_sample_count": 0,
            "selection_stats": {},
            "timing_ms": {
                "boundary": 0,
                "orientation": 0,
                "candidate": 0,
                "selection": 0,
                "total": 0,
            },
            "fast_path": True,
        }
        if mask.black_count < 20:
            result["reject_reason"] = "NO_BLACK_TARGET"
            result["timing_ms"]["total"] = _ticks_diff(
                _ticks_ms(), start
            )
            return result

        visited = bytearray(mask.width * mask.height)
        components = []
        for index in range(mask.width * mask.height):
            if not mask.black[index] or visited[index]:
                continue
            components.append(
                self._component(
                    mask,
                    index,
                    visited,
                    oriented=oriented,
                )
            )
        result["raw_candidate_count"] = len(components)
        result["raw_generated_count"] = len(components)

        candidates = []
        for component in components:
            candidate = self._candidate_from_component(
                mask,
                component,
                oriented=oriented,
            )
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(
            key=lambda item: (
                item.get("side_cm", 0.0),
                item.get("center", (0.0, 0.0))[1],
                item.get("center", (0.0, 0.0))[0],
            )
        )
        if mapper is not None:
            for candidate in candidates:
                candidate["image_corners"] = mapper.plane_points_to_image(
                    candidate["plane_corners"]
                )

        result["squares"] = tuple(candidates)
        result["candidate_count"] = len(candidates)
        result["scored_candidate_count"] = len(candidates)
        result["selection_candidate_count"] = len(candidates)
        result["score_sample_count"] = sum(
            item.get("score_samples", 0) for item in candidates
        )
        # Two or more independent tiles are enough to identify the numbered
        # target.  A single component is deliberately left to the composite
        # fallback because it may be an overlapping union or a clipped frame.
        minimum_candidates = max(1, int(minimum_candidates))
        if len(candidates) < minimum_candidates:
            result["reject_reason"] = "TOO_FEW_SEPARATED_SQUARES"
        else:
            result["valid"] = True
            result["reject_reason"] = "OK"
            result["explanation_score"] = min(1.0, len(candidates) / 4.0)
            result["coverage_recall"] = result["explanation_score"]
            result["coverage_precision"] = 1.0
        result["timing_ms"]["candidate"] = _ticks_diff(
            _ticks_ms(), start
        )
        result["timing_ms"]["total"] = _ticks_diff(_ticks_ms(), start)
        return result


def detect_numbered_squares(mask, mapper=None):
    """Convenience wrapper used by desktop tests and the task detector."""
    return NumberedSquareDetector().detect(
        mask, mapper=mapper, minimum_candidates=2
    )
