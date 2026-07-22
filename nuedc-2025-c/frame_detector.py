"""Reference-frame detector for the 2025 C target.

This module is intentionally independent from Sensor, Display and filesystem
code.  It prefers a matched outer/inner rectangle pair and can fall back to
the high-contrast inner black/white boundary while the first prototype is
being tuned.

Corner order used everywhere in this module:
    top-left, top-right, bottom-right, bottom-left
"""

import math


def _clamp(value, low=0.0, high=1.0):
    if value < low:
        return low
    if value > high:
        return high
    return value


def _distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def _lerp(a, b, ratio):
    return (
        a[0] + (b[0] - a[0]) * ratio,
        a[1] + (b[1] - a[1]) * ratio,
    )


def _polygon_area(points):
    total = 0.0
    count = len(points)
    for i in range(count):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % count]
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def _polygon_center(points):
    sx = 0.0
    sy = 0.0
    for x, y in points:
        sx += x
        sy += y
    count = len(points)
    return (sx / count, sy / count)


def _order_corners(points):
    """Return four corners as TL, TR, BR, BL."""
    pts = []
    for point in points:
        pts.append((float(point[0]), float(point[1])))

    if len(pts) != 4:
        raise ValueError("a rectangle must contain four corners")

    center = _polygon_center(pts)
    pts.sort(key=lambda p: math.atan2(p[1] - center[1], p[0] - center[0]))

    # Rotate the cyclic list so the first point is the visual top-left.
    start = 0
    best_sum = pts[0][0] + pts[0][1]
    for i in range(1, 4):
        value = pts[i][0] + pts[i][1]
        if value < best_sum:
            start = i
            best_sum = value
    pts = pts[start:] + pts[:start]

    # With image Y pointing down, TL->TR->BR has a positive cross product.
    ax = pts[1][0] - pts[0][0]
    ay = pts[1][1] - pts[0][1]
    bx = pts[2][0] - pts[1][0]
    by = pts[2][1] - pts[1][1]
    if ax * by - ay * bx < 0:
        pts = [pts[0], pts[3], pts[2], pts[1]]

    result = []
    for x, y in pts:
        result.append((int(round(x)), int(round(y))))
    return tuple(result)


def _point_in_polygon(point, polygon):
    """Ray-casting test that also works for mildly distorted quads."""
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y))
        if intersects:
            denominator = yj - yi
            if denominator == 0:
                denominator = 1e-6
            edge_x = (xj - xi) * (y - yi) / denominator + xi
            if x < edge_x:
                inside = not inside
        j = i
    return inside


def _bbox(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0 = min(xs)
    y0 = min(ys)
    x1 = max(xs)
    y1 = max(ys)
    return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)


def _minimum_edge_margin(points, width, height):
    """Return the smallest signed distance from a quad to the image edge."""
    margin = None
    for x, y in points:
        point_margin = min(x, y, width - 1 - x, height - 1 - y)
        if margin is None or point_margin < margin:
            margin = point_margin
    return -1.0 if margin is None else margin


def _gray_from_pixel(pixel):
    if isinstance(pixel, (tuple, list)):
        if len(pixel) >= 3:
            return (
                0.299 * pixel[0]
                + 0.587 * pixel[1]
                + 0.114 * pixel[2]
            )
        if len(pixel) == 1:
            return float(pixel[0])

    value = int(pixel)
    if value <= 255:
        return float(value)

    # Defensive RGB565 decoding for firmwares returning a packed pixel.
    red = ((value >> 11) & 0x1F) * 255.0 / 31.0
    green = ((value >> 5) & 0x3F) * 255.0 / 63.0
    blue = (value & 0x1F) * 255.0 / 31.0
    return 0.299 * red + 0.587 * green + 0.114 * blue


def _rgb_from_pixel(pixel):
    if isinstance(pixel, (tuple, list)):
        if len(pixel) >= 3:
            return (float(pixel[0]), float(pixel[1]), float(pixel[2]))
        if len(pixel) == 1:
            value = float(pixel[0])
            return (value, value, value)

    value = int(pixel)
    if value <= 255:
        value = float(value)
        return (value, value, value)

    red = ((value >> 11) & 0x1F) * 255.0 / 31.0
    green = ((value >> 5) & 0x3F) * 255.0 / 63.0
    blue = (value & 0x1F) * 255.0 / 31.0
    return (red, green, blue)


def _sample_gray(image, point):
    x = int(round(point[0]))
    y = int(round(point[1]))
    if x < 0 or y < 0 or x >= image.width() or y >= image.height():
        return None
    return _gray_from_pixel(image.get_pixel(x, y))


def _sample_rgb(image, point):
    x = int(round(point[0]))
    y = int(round(point[1]))
    if x < 0 or y < 0 or x >= image.width() or y >= image.height():
        return None
    return _rgb_from_pixel(image.get_pixel(x, y))


def _edge_response(image, inside_point, outside_point):
    """Positive response for a dark frame with a brighter exterior."""
    inside = _sample_rgb(image, inside_point)
    outside = _sample_rgb(image, outside_point)
    if inside is None or outside is None:
        return None

    inside_y = 0.299 * inside[0] + 0.587 * inside[1] + 0.114 * inside[2]
    outside_y = (
        0.299 * outside[0] + 0.587 * outside[1] + 0.114 * outside[2]
    )
    color_distance = (
        abs(outside[0] - inside[0])
        + abs(outside[1] - inside[1])
        + abs(outside[2] - inside[2])
    )
    return outside_y - inside_y + 0.15 * color_distance


def _line_intersection(a0, a1, b0, b1):
    ax = a1[0] - a0[0]
    ay = a1[1] - a0[1]
    bx = b1[0] - b0[0]
    by = b1[1] - b0[1]
    denominator = ax * by - ay * bx
    if abs(denominator) < 1e-6:
        return None

    cx = b0[0] - a0[0]
    cy = b0[1] - a0[1]
    ratio = (cx * by - cy * bx) / denominator
    return (a0[0] + ratio * ax, a0[1] + ratio * ay)


def _mean(values):
    if not values:
        return 0.0
    total = 0.0
    for value in values:
        total += value
    return total / len(values)


def _closeness(value, expected, tolerance):
    if tolerance <= 0:
        return 0.0
    return _clamp(1.0 - abs(value - expected) / tolerance)


def _relative_difference(first, second):
    return abs(first - second) / max((first + second) * 0.5, 1e-6)


def _quad_regularity(corners):
    """Return rotation-invariant regularity diagnostics for a coarse quad.

    find_rects() occasionally closes three real target edges with one diagonal
    edge when the distant inner aperture is only a few dozen preview pixels
    wide. Its photometric score can still look plausible, so geometry must be
    classified independently before the four corners are trusted as a 1080p
    refinement seed.
    """
    sides = []
    angles = []
    for index in range(4):
        sides.append(_distance(
            corners[index], corners[(index + 1) % 4]
        ))

    dimension_a = (sides[0] + sides[2]) * 0.5
    dimension_b = (sides[1] + sides[3]) * 0.5
    short_side = max(min(dimension_a, dimension_b), 1e-6)
    fill_ratio = _polygon_area(corners) / max(
        dimension_a * dimension_b, 1e-6
    )
    opposite_error_a = _relative_difference(sides[0], sides[2])
    opposite_error_b = _relative_difference(sides[1], sides[3])

    first_midpoint = (
        (corners[0][0] + corners[2][0]) * 0.5,
        (corners[0][1] + corners[2][1]) * 0.5,
    )
    second_midpoint = (
        (corners[1][0] + corners[3][0]) * 0.5,
        (corners[1][1] + corners[3][1]) * 0.5,
    )
    diagonal_midpoint_error = _distance(
        first_midpoint, second_midpoint
    ) / short_side

    for index in range(4):
        point = corners[index]
        previous = corners[(index - 1) % 4]
        following = corners[(index + 1) % 4]
        first_x = previous[0] - point[0]
        first_y = previous[1] - point[1]
        second_x = following[0] - point[0]
        second_y = following[1] - point[1]
        denominator = max(
            math.sqrt(first_x * first_x + first_y * first_y)
            * math.sqrt(second_x * second_x + second_y * second_y),
            1e-6,
        )
        cosine = (first_x * second_x + first_y * second_y) / denominator
        cosine = max(-1.0, min(cosine, 1.0))
        angles.append(math.acos(cosine) * 180.0 / math.pi)

    return {
        "fill_ratio": fill_ratio,
        "opposite_error_a": opposite_error_a,
        "opposite_error_b": opposite_error_b,
        "max_opposite_error": max(
            opposite_error_a, opposite_error_b
        ),
        "diagonal_midpoint_error": diagonal_midpoint_error,
        "min_corner_angle": min(angles),
        "max_corner_angle": max(angles),
    }


class FrameDetector:
    """Detect the nested rectangular reference frame."""

    def __init__(
        self,
        rect_threshold=3000,
        roi_threshold_scale=0.60,
        expected_short_ratio=0.81,
        expected_long_ratio=0.86,
        expected_outer_aspect=1.414,
        expected_inner_aspect=1.50,
        allow_inner_only=True,
    ):
        self.rect_threshold = rect_threshold
        self.roi_threshold_scale = roi_threshold_scale
        self.expected_short_ratio = expected_short_ratio
        self.expected_long_ratio = expected_long_ratio
        self.expected_area_ratio = expected_short_ratio * expected_long_ratio
        self.expected_outer_aspect = expected_outer_aspect
        self.expected_inner_aspect = expected_inner_aspect
        self.allow_inner_only = allow_inner_only

        self.min_area_ratio = 0.0025
        self.max_area_ratio = 0.20
        self.min_short_side = 10.0
        self.max_candidates = 24
        self.min_inner_contrast = 15.0
        self.min_pair_score = 0.48
        # A coarse rectangle is only a seed for the strict 1080p refiner.  A
        # slightly lower clean-seed floor recovers small distant frames, while
        # candidates with an imperfect low-resolution ring must clear the
        # higher soft-seed floor below.
        self.min_inner_score = 0.45
        self.min_soft_inner_score = 0.55
        # A malformed find_rects() quad may still be a useful location-only
        # seed. It clears a lower score floor because it can never directly
        # become a measurement; the 1080p physical-model recovery and strict
        # final gate must validate it first.
        self.min_roi_seed_score = 0.30
        self.location_seed_relaxable_reasons = (
            "INNER_CONTRAST",
            "RING_SAMPLES",
        )
        self.min_outer_edge_response = 12.0
        self.min_outer_valid_sides = 3

        # CLEAN_SEED thresholds are deliberately far inside the measured gap:
        # good 160 cm quads had <=0.05 opposite error, <=0.02 diagonal-midpoint
        # error and >=0.96 fill, while the closest malformed quad measured
        # roughly 0.35, 0.14 and 0.83 respectively.
        self.max_clean_opposite_error = 0.15
        self.max_clean_diagonal_midpoint_error = 0.08
        self.min_clean_fill_ratio = 0.90
        self.min_clean_corner_angle = 70.0
        self.max_clean_corner_angle = 110.0
        self.min_roi_fill_ratio = 0.45
        self.max_roi_diagonal_midpoint_error = 0.75

        # Coarse candidates are intentionally checked more leniently than the
        # 1080p quality gate. Their job is only to provide a plausible seed.
        # Only the white-inside to black-border transition is invariant: the
        # material outside the printed target may be blue, black or white.
        self.coarse_ring_samples_per_side = 4
        self.coarse_ring_contrast = 8.0
        self.min_ring_valid_samples = 8
        self.min_ring_inside_pass_ratio = 0.45
        self.min_ring_side_pass_ratio = 0.25
        self.min_ring_outside_pass_ratio = 0.30
        self.min_ring_outside_contrast = 0.0
        self.min_edge_margin_px = 1.0

        self.last_candidates = []
        self.last_pair_count = 0
        self.last_raw_rect_count = 0
        self.last_candidate_errors = 0
        self.last_rejection_counts = {}
        self.last_soft_gate_counts = {}
        self.last_roi = None
        self.last_threshold = rect_threshold
        self._last_candidate_reject_reason = None
        self._last_pair_reject_reason = None

    def _count_rejection(self, reason):
        self.last_rejection_counts[reason] = (
            self.last_rejection_counts.get(reason, 0) + 1
        )

    def _count_soft_gate(self, reason):
        self.last_soft_gate_counts[reason] = (
            self.last_soft_gate_counts.get(reason, 0) + 1
        )

    def _reject_candidate(self, reason):
        self._last_candidate_reject_reason = reason
        return None

    def _reject_pair(self, reason):
        self._last_pair_reject_reason = reason
        return None

    def _candidate_from_rect(
        self,
        image,
        rect,
        allow_location_seed=False,
    ):
        self._last_candidate_reject_reason = None
        corners = _order_corners(rect.corners())
        area = _polygon_area(corners)
        image_area = image.width() * image.height()
        area_ratio = area / image_area

        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return self._reject_candidate("RAW_AREA")

        side_0 = _distance(corners[0], corners[1])
        side_1 = _distance(corners[1], corners[2])
        side_2 = _distance(corners[2], corners[3])
        side_3 = _distance(corners[3], corners[0])

        dimension_a = (side_0 + side_2) * 0.5
        dimension_b = (side_1 + side_3) * 0.5
        short_side = min(dimension_a, dimension_b)
        long_side = max(dimension_a, dimension_b)
        if short_side < self.min_short_side:
            return self._reject_candidate("RAW_SHORT_SIDE")

        aspect = long_side / short_side
        if aspect < 1.10 or aspect > 1.85:
            return self._reject_candidate("RAW_ASPECT")

        center = _polygon_center(corners)
        inner_mean, outer_mean, contrast = self._inner_boundary_contrast(
            image, corners
        )

        base_score = max(
            _closeness(aspect, self.expected_outer_aspect, 0.42),
            _closeness(aspect, self.expected_inner_aspect, 0.42),
        )

        predicted_outer = self._predict_outer_corners(corners)
        ring_metrics = self._quick_ring_metrics(
            image,
            corners,
            predicted_outer,
        )

        candidate = {
            "corners": corners,
            "bbox": _bbox(corners),
            "area": area,
            "area_ratio": area_ratio,
            "short_side": short_side,
            "long_side": long_side,
            "aspect": aspect,
            "center": center,
            "inner_mean": inner_mean,
            "outer_mean": outer_mean,
            "contrast": contrast,
            "base_score": base_score,
            "quad_edge_margin": _minimum_edge_margin(
                corners,
                image.width(),
                image.height(),
            ),
            "predicted_outer_corners": predicted_outer,
            "predicted_outer_edge_margin": _minimum_edge_margin(
                predicted_outer,
                image.width(),
                image.height(),
            ),
            "ring_metrics": ring_metrics,
            "quad_regularity": _quad_regularity(corners),
        }
        candidate["inner_reject_reason"] = (
            self._inner_candidate_reject_reason(candidate)
        )
        candidate["inner_soft_reasons"] = (
            self._inner_candidate_soft_reasons(candidate)
            if candidate["inner_reject_reason"] is None
            else ()
        )
        candidate["seed_class"] = self._seed_class_for_candidate(
            candidate,
            allow_location_seed=allow_location_seed,
        )
        candidate["location_seed_relaxed"] = (
            allow_location_seed
            and candidate["inner_reject_reason"] is not None
            and candidate["seed_class"] == "ROI_SEED"
        )
        candidate["coarse_inner_score"] = -1.0
        return candidate

    def _quick_ring_metrics(self, image, inner_corners, outer_corners):
        """Cheap low-resolution white-black-white ring verification."""
        inside_total = 0.0
        outside_total = 0.0
        inside_passes = 0
        outside_passes = 0
        valid_samples = 0
        side_inside_ratios = []
        positions = (0.20, 0.40, 0.60, 0.80)

        for side in range(4):
            side_valid = 0
            side_inside_passes = 0
            inner_start = inner_corners[side]
            inner_end = inner_corners[(side + 1) % 4]
            outer_start = outer_corners[side]
            outer_end = outer_corners[(side + 1) % 4]
            for position in positions:
                inner_point = _lerp(inner_start, inner_end, position)
                outer_point = _lerp(outer_start, outer_end, position)
                vector_x = outer_point[0] - inner_point[0]
                vector_y = outer_point[1] - inner_point[1]
                thickness = math.sqrt(
                    vector_x * vector_x + vector_y * vector_y
                )
                if thickness < 1.5:
                    continue

                unit_x = vector_x / thickness
                unit_y = vector_y / thickness
                probe = max(1.0, min(2.0, thickness * 0.30))
                inside_gray = _sample_gray(
                    image,
                    (
                        inner_point[0] - unit_x * probe,
                        inner_point[1] - unit_y * probe,
                    ),
                )
                border_values = []
                for fraction in (0.35, 0.55, 0.75):
                    border_gray = _sample_gray(
                        image,
                        (
                            inner_point[0] + vector_x * fraction,
                            inner_point[1] + vector_y * fraction,
                        ),
                    )
                    if border_gray is not None:
                        border_values.append(border_gray)
                outside_gray = _sample_gray(
                    image,
                    (
                        outer_point[0] + unit_x * probe,
                        outer_point[1] + unit_y * probe,
                    ),
                )
                if (
                    inside_gray is None
                    or outside_gray is None
                    or len(border_values) < 2
                ):
                    continue

                border_gray = _mean(border_values)
                inside_contrast = inside_gray - border_gray
                outside_contrast = outside_gray - border_gray
                inside_total += inside_contrast
                outside_total += outside_contrast
                valid_samples += 1
                side_valid += 1
                if inside_contrast >= self.coarse_ring_contrast:
                    inside_passes += 1
                    side_inside_passes += 1
                if outside_contrast >= self.coarse_ring_contrast * 0.5:
                    outside_passes += 1

            side_inside_ratios.append(
                side_inside_passes / max(side_valid, 1)
            )

        if valid_samples <= 0:
            return {
                "valid_samples": 0,
                "inside_contrast": -999.0,
                "outside_contrast": -999.0,
                "inside_pass_ratio": 0.0,
                "outside_pass_ratio": 0.0,
                "min_side_inside_pass_ratio": 0.0,
            }
        return {
            "valid_samples": valid_samples,
            "inside_contrast": inside_total / valid_samples,
            "outside_contrast": outside_total / valid_samples,
            "inside_pass_ratio": inside_passes / valid_samples,
            "outside_pass_ratio": outside_passes / valid_samples,
            "min_side_inside_pass_ratio": min(side_inside_ratios),
        }

    def _inner_candidate_reject_reason(self, candidate):
        """Return only gates that make a coarse seed unusable.

        The 640x360 find_rects() quad is allowed to be a little inaccurate.
        In particular, one poorly placed side must not prevent the 1080p
        refiner from seeing the target.  The inside-ring checks therefore live
        in _inner_candidate_soft_reasons(); missing samples and image-edge
        seeds remain hard failures. The material outside the
        printed black border is not controlled by the target specification,
        so outside contrast is diagnostic rather than a hard requirement.
        """
        if candidate["contrast"] < self.min_inner_contrast:
            return "INNER_CONTRAST"
        if (
            candidate["predicted_outer_edge_margin"]
            < self.min_edge_margin_px
        ):
            return "INNER_EDGE"

        ring = candidate["ring_metrics"]
        if ring["valid_samples"] < self.min_ring_valid_samples:
            return "RING_SAMPLES"
        return None

    def _inner_candidate_soft_reasons(self, candidate):
        """Describe recoverable low-resolution ring imperfections."""
        ring = candidate["ring_metrics"]
        reasons = []
        if ring["inside_contrast"] < self.coarse_ring_contrast:
            reasons.append("RING_INSIDE_CONTRAST")
        if (
            ring["inside_pass_ratio"]
            < self.min_ring_inside_pass_ratio
        ):
            reasons.append("RING_INSIDE_PASS")
        if (
            ring["min_side_inside_pass_ratio"]
            < self.min_ring_side_pass_ratio
        ):
            reasons.append("RING_SIDE_PASS")
        if ring["outside_contrast"] < self.min_ring_outside_contrast:
            reasons.append("RING_OUTSIDE_CONTRAST")
        if (
            ring["outside_pass_ratio"]
            < self.min_ring_outside_pass_ratio
        ):
            reasons.append("RING_OUTSIDE_PASS")
        return tuple(reasons)

    def _location_seed_geometry_valid(self, candidate):
        """Return whether a central-ROI quad is safe as a location hint.

        This intentionally ignores low-resolution photometry.  The result is
        never a measurement: it only bounds the 1080p relocalization region,
        whose strict ring and geometry gates still decide acceptance.
        """
        regularity = candidate["quad_regularity"]
        return (
            regularity["fill_ratio"] >= self.min_roi_fill_ratio
            and regularity["diagonal_midpoint_error"]
            <= self.max_roi_diagonal_midpoint_error
            and regularity["min_corner_angle"] >= 20.0
            and regularity["max_corner_angle"] <= 160.0
        )

    def _seed_class_for_candidate(
        self,
        candidate,
        allow_location_seed=False,
    ):
        """Classify whether the coarse corners are trusted or location-only."""
        hard_reason = candidate.get("inner_reject_reason")
        if hard_reason is not None:
            if (
                allow_location_seed
                and hard_reason in self.location_seed_relaxable_reasons
                and self._location_seed_geometry_valid(candidate)
            ):
                # Reuse ROI_SEED so the existing 1080p physical-model
                # regularization path is guaranteed to run.
                return "ROI_SEED"
            return "REJECT"
        regularity = candidate["quad_regularity"]
        clean = (
            regularity["max_opposite_error"]
            <= self.max_clean_opposite_error
            and regularity["diagonal_midpoint_error"]
            <= self.max_clean_diagonal_midpoint_error
            and regularity["fill_ratio"] >= self.min_clean_fill_ratio
            and regularity["min_corner_angle"]
            >= self.min_clean_corner_angle
            and regularity["max_corner_angle"]
            <= self.max_clean_corner_angle
        )
        if clean:
            return "CLEAN_SEED"

        # This broader geometric envelope keeps a severely self-inconsistent
        # quadrilateral from creating an enormous recovery region. Candidates
        # in this class provide only center/area/orientation hints.
        if self._location_seed_geometry_valid(candidate):
            return "ROI_SEED"
        return "REJECT"

    def _ring_score(self, candidate):
        ring = candidate["ring_metrics"]
        return _clamp(
            0.50 * ring["inside_pass_ratio"]
            + 0.25 * ring["outside_pass_ratio"]
            + 0.25 * ring["min_side_inside_pass_ratio"]
        )

    def _inner_boundary_contrast(self, image, corners):
        """Sample white just inside and black just outside a candidate edge."""
        center = _polygon_center(corners)
        inside_values = []
        outside_values = []
        positions = (0.20, 0.35, 0.50, 0.65, 0.80)

        for side in range(4):
            p0 = corners[side]
            p1 = corners[(side + 1) % 4]
            for position in positions:
                edge_point = _lerp(p0, p1, position)
                inside_point = _lerp(edge_point, center, 0.12)
                outside_point = _lerp(edge_point, center, -0.12)

                inside_gray = _sample_gray(image, inside_point)
                outside_gray = _sample_gray(image, outside_point)
                if inside_gray is not None:
                    inside_values.append(inside_gray)
                if outside_gray is not None:
                    outside_values.append(outside_gray)

        inside_mean = _mean(inside_values)
        outside_mean = _mean(outside_values)
        return inside_mean, outside_mean, inside_mean - outside_mean

    def _make_candidates(
        self,
        image,
        roi=None,
        threshold_override=None,
        scale_roi_threshold=True,
        allow_location_seed=False,
    ):
        self.last_roi = roi
        self.last_rejection_counts = {}
        self.last_soft_gate_counts = {}
        threshold = (
            self.rect_threshold
            if threshold_override is None
            else int(threshold_override)
        )
        if roi is not None and scale_roi_threshold:
            # find_rects produced almost no candidates in a small tracking ROI
            # with the full-frame threshold.  A lower local threshold is safe
            # because the ROI already restricts the search around a known lock.
            threshold = max(
                500,
                int(threshold * self.roi_threshold_scale),
            )
        self.last_threshold = threshold

        if roi is None:
            raw_rects = image.find_rects(threshold=threshold)
        else:
            raw_rects = image.find_rects(
                threshold=threshold,
                roi=roi,
            )
        self.last_raw_rect_count = len(raw_rects)
        self.last_candidate_errors = 0

        candidates = []
        for rect in raw_rects:
            try:
                candidate = self._candidate_from_rect(
                    image,
                    rect,
                    allow_location_seed=allow_location_seed,
                )
                if candidate is not None:
                    candidates.append(candidate)
                    reject_reason = candidate.get("inner_reject_reason")
                    if reject_reason is not None:
                        self._count_rejection(reject_reason)
                    elif candidate.get("seed_class") == "REJECT":
                        self._count_rejection("COARSE_GEOMETRY_REJECT")
                    else:
                        for reason in candidate.get(
                            "inner_soft_reasons", ()
                        ):
                            self._count_soft_gate(reason)
                elif self._last_candidate_reject_reason is not None:
                    self._count_rejection(
                        self._last_candidate_reject_reason
                    )
            except Exception:
                # A malformed rectangle must not stop the live preview.
                self.last_candidate_errors += 1
                self._count_rejection("RAW_EXCEPTION")

        candidates.sort(key=lambda item: item["base_score"], reverse=True)
        if len(candidates) > self.max_candidates:
            candidates = candidates[:self.max_candidates]

        self.last_candidates = candidates
        return candidates

    def _score_pair(self, outer, inner):
        self._last_pair_reject_reason = None
        if inner.get("inner_reject_reason") is not None:
            return self._reject_pair("PAIR_INNER_GATE")
        if inner.get("seed_class") == "REJECT":
            return self._reject_pair("PAIR_INNER_GEOMETRY")
        if outer.get("quad_edge_margin", -1.0) < self.min_edge_margin_px:
            return self._reject_pair("PAIR_OUTER_EDGE")
        if outer["area"] <= inner["area"] * 1.05:
            return self._reject_pair("PAIR_AREA_ORDER")

        if not _point_in_polygon(inner["center"], outer["corners"]):
            return self._reject_pair("PAIR_CENTER_OUTSIDE")

        inside_count = 0
        for point in inner["corners"]:
            if _point_in_polygon(point, outer["corners"]):
                inside_count += 1
        if inside_count < 3:
            return self._reject_pair("PAIR_CORNERS_OUTSIDE")

        short_ratio = inner["short_side"] / outer["short_side"]
        long_ratio = inner["long_side"] / outer["long_side"]
        area_ratio = inner["area"] / outer["area"]

        if short_ratio < 0.52 or short_ratio > 0.97:
            return self._reject_pair("PAIR_SHORT_RATIO")
        if long_ratio < 0.58 or long_ratio > 0.98:
            return self._reject_pair("PAIR_LONG_RATIO")
        if inner["contrast"] < self.min_inner_contrast:
            return self._reject_pair("PAIR_INNER_CONTRAST")

        outer_diagonal = math.sqrt(
            outer["short_side"] * outer["short_side"]
            + outer["long_side"] * outer["long_side"]
        )
        center_error = _distance(outer["center"], inner["center"])
        center_error /= max(outer_diagonal, 1.0)
        if center_error > 0.18:
            return self._reject_pair("PAIR_CENTER_ERROR")

        short_score = _closeness(
            short_ratio, self.expected_short_ratio, 0.18
        )
        long_score = _closeness(
            long_ratio, self.expected_long_ratio, 0.16
        )
        area_score = _closeness(
            area_ratio, self.expected_area_ratio, 0.26
        )
        center_score = _clamp(1.0 - center_error / 0.18)
        outer_aspect_score = _closeness(
            outer["aspect"], self.expected_outer_aspect, 0.40
        )
        inner_aspect_score = _closeness(
            inner["aspect"], self.expected_inner_aspect, 0.40
        )
        contrast_score = _clamp(
            (inner["contrast"] - self.min_inner_contrast) / 60.0
        )
        ring_score = self._ring_score(inner)

        score = (
            0.17 * short_score
            + 0.17 * long_score
            + 0.08 * area_score
            + 0.10 * center_score
            + 0.08 * outer_aspect_score
            + 0.08 * inner_aspect_score
            + 0.14 * contrast_score
            + 0.18 * ring_score
        )

        return {
            "outer": outer,
            "inner": inner,
            "score": score,
            "short_ratio": short_ratio,
            "long_ratio": long_ratio,
            "area_ratio": area_ratio,
            "center_error": center_error,
        }

    def _best_pair(self, candidates):
        ranked = self._rank_pairs(candidates)
        return ranked[0] if ranked else None

    def _rank_pairs(self, candidates):
        ranked = []
        outer_candidates = []
        inner_candidates = []
        for candidate in candidates:
            if candidate.get("inner_reject_reason") is None:
                inner_candidates.append(candidate)
            if (
                candidate.get("quad_edge_margin", -1.0)
                >= self.min_edge_margin_px
            ):
                outer_candidates.append(candidate)
            else:
                self._count_rejection("OUTER_EDGE")

        for outer in outer_candidates:
            for inner in inner_candidates:
                if outer is inner:
                    continue
                pair = self._score_pair(outer, inner)
                if pair is None:
                    if self._last_pair_reject_reason is not None:
                        self._count_rejection(
                            self._last_pair_reject_reason
                        )
                    continue
                ranked.append(pair)

        ranked.sort(key=lambda item: item["score"], reverse=True)
        self.last_pair_count = len(ranked)
        return ranked

    def _best_inner_only(self, candidates):
        ranked = self._rank_inner_only(candidates)
        return ranked[0] if ranked else None

    def _rank_inner_only(self, candidates):
        ranked = []
        for candidate in candidates:
            seed_class = candidate.get("seed_class")
            location_seed_relaxed = candidate.get(
                "location_seed_relaxed", False
            )
            if (
                candidate.get("inner_reject_reason") is not None
                and not location_seed_relaxed
            ):
                continue
            if seed_class == "REJECT":
                continue

            aspect_score = _closeness(
                candidate["aspect"], self.expected_inner_aspect, 0.42
            )
            contrast_score = _clamp(
                (candidate["contrast"] - self.min_inner_contrast) / 60.0
            )
            if location_seed_relaxed:
                # A central-ROI location seed is deliberately ranked mostly by
                # shape, not by the photometric gate that it was allowed to
                # miss.  It only chooses a bounded 1080p search region.
                regularity = candidate["quad_regularity"]
                fill_score = _clamp(
                    (regularity["fill_ratio"] - self.min_roi_fill_ratio)
                    / max(1.0 - self.min_roi_fill_ratio, 1e-6)
                )
                score = (
                    0.70 * aspect_score
                    + 0.20 * fill_score
                    + 0.10 * contrast_score
                )
            else:
                ring_score = self._ring_score(candidate)
                score = (
                    0.30 * aspect_score
                    + 0.35 * contrast_score
                    + 0.35 * ring_score
                )
            candidate["coarse_inner_score"] = score

            ranked.append({
                "inner": candidate,
                "score": score,
            })
        ranked.sort(key=lambda item: item["score"], reverse=True)
        return ranked

    def _pair_result(self, pair, candidate_count):
        outer = pair["outer"]
        inner = pair["inner"]
        ring = inner["ring_metrics"]
        return {
            "mode": "PAIR",
            "outer_corners": outer["corners"],
            "inner_corners": inner["corners"],
            "outer_bbox": outer["bbox"],
            "inner_bbox": inner["bbox"],
            "score": pair["score"],
            "contrast": inner["contrast"],
            "short_ratio": pair["short_ratio"],
            "long_ratio": pair["long_ratio"],
            "area_ratio": pair["area_ratio"],
            "outer_valid_sides": 4,
            "outer_edge_response": 0.0,
            "candidate_count": candidate_count,
            "coarse_ring_inside_pass_ratio": ring[
                "inside_pass_ratio"
            ],
            "coarse_ring_outside_pass_ratio": ring[
                "outside_pass_ratio"
            ],
            "coarse_ring_min_side_pass_ratio": ring[
                "min_side_inside_pass_ratio"
            ],
            "coarse_predicted_outer_margin": inner[
                "predicted_outer_edge_margin"
            ],
            "coarse_soft_reasons": inner.get("inner_soft_reasons", ()),
            "seed_class": inner.get("seed_class", "CLEAN_SEED"),
            "location_seed_relaxed": inner.get(
                "location_seed_relaxed", False
            ),
            "coarse_regularity": inner.get("quad_regularity", {}),
        }

    def _inner_result(self, fallback, candidate_count):
        inner = fallback["inner"]
        ring = inner["ring_metrics"]
        return {
            "mode": "INNER_ONLY",
            "outer_corners": None,
            "inner_corners": inner["corners"],
            "outer_bbox": None,
            "inner_bbox": inner["bbox"],
            "score": fallback["score"],
            "contrast": inner["contrast"],
            "short_ratio": 0.0,
            "long_ratio": 0.0,
            "area_ratio": 0.0,
            "outer_valid_sides": 0,
            "outer_edge_response": 0.0,
            "candidate_count": candidate_count,
            "coarse_ring_inside_pass_ratio": ring[
                "inside_pass_ratio"
            ],
            "coarse_ring_outside_pass_ratio": ring[
                "outside_pass_ratio"
            ],
            "coarse_ring_min_side_pass_ratio": ring[
                "min_side_inside_pass_ratio"
            ],
            "coarse_predicted_outer_margin": inner[
                "predicted_outer_edge_margin"
            ],
            "coarse_soft_reasons": inner.get("inner_soft_reasons", ()),
            "seed_class": inner.get("seed_class", "CLEAN_SEED"),
            "location_seed_relaxed": inner.get(
                "location_seed_relaxed", False
            ),
            "coarse_regularity": inner.get("quad_regularity", {}),
        }

    def _same_hypothesis(self, first, second):
        first_corners = first.get("inner_corners")
        second_corners = second.get("inner_corners")
        if first_corners is None or second_corners is None:
            return False

        first_center = _polygon_center(first_corners)
        second_center = _polygon_center(second_corners)
        first_sides = []
        second_sides = []
        for index in range(4):
            first_sides.append(_distance(
                first_corners[index], first_corners[(index + 1) % 4]
            ))
            second_sides.append(_distance(
                second_corners[index], second_corners[(index + 1) % 4]
            ))
        first_a = (first_sides[0] + first_sides[2]) * 0.5
        first_b = (first_sides[1] + first_sides[3]) * 0.5
        second_a = (second_sides[0] + second_sides[2]) * 0.5
        second_b = (second_sides[1] + second_sides[3]) * 0.5
        first_short = min(first_a, first_b)
        first_long = max(first_a, first_b)
        second_short = min(second_a, second_b)
        second_long = max(second_a, second_b)

        center_limit = max(5.0, min(first_short, second_short) * 0.15)
        if _distance(first_center, second_center) > center_limit:
            return False
        short_error = abs(first_short - second_short) / max(
            (first_short + second_short) * 0.5, 1.0
        )
        long_error = abs(first_long - second_long) / max(
            (first_long + second_long) * 0.5, 1.0
        )
        return short_error <= 0.18 and long_error <= 0.18

    def select_distinct_hypotheses(self, hypotheses, max_results=6):
        """Keep spatially distinct seeds from several preview frames."""
        ordered = list(hypotheses)
        ordered.sort(
            key=lambda item: item.get("score", 0.0)
            + (0.05 if item.get("mode") == "MODEL_PAIR" else 0.0)
            + (
                2.0
                if item.get("seed_class") == "TRACK_SEED"
                else 1.5
                if item.get("location_seed_relaxed", False)
                else 1.0
                if item.get("seed_class") == "CLEAN_SEED"
                else 0.5
                if item.get("seed_class") == "ROI_SEED"
                else 0.0
            ),
            reverse=True,
        )
        selected = []
        for hypothesis in ordered:
            duplicate = False
            for existing in selected:
                if self._same_hypothesis(hypothesis, existing):
                    duplicate = True
                    break
            if duplicate:
                continue
            selected.append(hypothesis)
            if len(selected) >= max_results:
                break
        return selected

    def detect_hypotheses(
        self,
        image,
        roi=None,
        max_results=3,
        threshold_override=None,
        scale_roi_threshold=True,
        allow_location_seed=False,
    ):
        """Return several plausible seeds so 1080p validation can decide.

        The former single-best coarse result could be a large rectangular
        structure elsewhere in the scene.  Returning a small, deduplicated
        set lets the high-resolution ring/geometry gates select the real A4
        target without running a full-frame 1080p rectangle search.
        """
        candidates = self._make_candidates(
            image,
            roi=roi,
            threshold_override=threshold_override,
            scale_roi_threshold=scale_roi_threshold,
            allow_location_seed=allow_location_seed,
        )
        ranked_pairs = self._rank_pairs(candidates)
        ranked_inner = self._rank_inner_only(candidates)
        results = []

        # Reserve at least one slot for an inner-only hypothesis.  A false
        # nested pair must not hide a real frame whose outer edge was missed
        # by low-resolution find_rects().
        pair_limit = max_results
        if self.allow_inner_only and max_results > 1:
            pair_limit = max(1, max_results // 2)
        for pair in ranked_pairs:
            if pair["score"] < self.min_pair_score:
                self._count_rejection("PAIR_SCORE")
                break
            result = self._pair_result(pair, len(candidates))
            if any(self._same_hypothesis(result, old) for old in results):
                continue
            results.append(result)
            if len(results) >= pair_limit:
                break

        if self.allow_inner_only:
            for fallback in ranked_inner:
                soft_reasons = fallback["inner"].get(
                    "inner_soft_reasons", ()
                )
                seed_class = fallback["inner"].get(
                    "seed_class", "CLEAN_SEED"
                )
                if seed_class == "ROI_SEED":
                    minimum_score = self.min_roi_seed_score
                else:
                    minimum_score = (
                        self.min_soft_inner_score
                        if soft_reasons
                        else self.min_inner_score
                    )
                if fallback["score"] < minimum_score:
                    self._count_rejection(
                        "ROI_SEED_SCORE"
                        if seed_class == "ROI_SEED"
                        else (
                            "INNER_SOFT_SCORE"
                            if soft_reasons
                            else "INNER_SCORE"
                        )
                    )
                    break
                result = self._inner_result(fallback, len(candidates))
                if any(self._same_hypothesis(result, old) for old in results):
                    continue
                results.append(result)
                if len(results) >= max_results:
                    break

        return results

    def _predict_outer_corners(self, inner_corners):
        """Expand the detected inner quad using the measured frame model."""
        tl, tr, br, bl = inner_corners
        short_margin = (1.0 / self.expected_short_ratio - 1.0) * 0.5
        long_margin = (1.0 / self.expected_long_ratio - 1.0) * 0.5

        return (
            (
                tl[0] + (tl[0] - tr[0]) * short_margin
                + (tl[0] - bl[0]) * long_margin,
                tl[1] + (tl[1] - tr[1]) * short_margin
                + (tl[1] - bl[1]) * long_margin,
            ),
            (
                tr[0] + (tr[0] - tl[0]) * short_margin
                + (tr[0] - br[0]) * long_margin,
                tr[1] + (tr[1] - tl[1]) * short_margin
                + (tr[1] - br[1]) * long_margin,
            ),
            (
                br[0] + (br[0] - bl[0]) * short_margin
                + (br[0] - tr[0]) * long_margin,
                br[1] + (br[1] - bl[1]) * short_margin
                + (br[1] - tr[1]) * long_margin,
            ),
            (
                bl[0] + (bl[0] - br[0]) * short_margin
                + (bl[0] - tl[0]) * long_margin,
                bl[1] + (bl[1] - br[1]) * short_margin
                + (bl[1] - tl[1]) * long_margin,
            ),
        )

    def _refine_outer_from_inner(self, image, inner):
        predicted = self._predict_outer_corners(inner["corners"])
        center = inner["center"]

        short_border = inner["short_side"] * (
            1.0 / self.expected_short_ratio - 1.0
        ) * 0.5
        long_border = inner["long_side"] * (
            1.0 / self.expected_long_ratio - 1.0
        ) * 0.5
        border = min(short_border, long_border)
        search_radius = int(round(border * 0.38))
        search_radius = max(2, min(search_radius, 8))
        probe = max(1.0, min(border * 0.25, 2.0))
        positions = (0.15, 0.22, 0.29, 0.36, 0.43, 0.50,
                     0.57, 0.64, 0.71, 0.78, 0.85)

        shifted_edges = []
        edge_scores = []
        valid_sides = 0

        for side in range(4):
            p0 = predicted[side]
            p1 = predicted[(side + 1) % 4]
            midpoint = _lerp(p0, p1, 0.5)
            normal_x = midpoint[0] - center[0]
            normal_y = midpoint[1] - center[1]
            normal_length = math.sqrt(
                normal_x * normal_x + normal_y * normal_y
            )
            if normal_length < 1e-6:
                return None
            normal_x /= normal_length
            normal_y /= normal_length

            best_offset = 0
            best_score = -9999.0
            for offset in range(-search_radius, search_radius + 1):
                responses = []
                for position in positions:
                    point = _lerp(p0, p1, position)
                    point = (
                        point[0] + normal_x * offset,
                        point[1] + normal_y * offset,
                    )
                    inside_point = (
                        point[0] - normal_x * probe,
                        point[1] - normal_y * probe,
                    )
                    outside_point = (
                        point[0] + normal_x * probe,
                        point[1] + normal_y * probe,
                    )
                    response = _edge_response(
                        image, inside_point, outside_point
                    )
                    if response is not None:
                        responses.append(response)

                if not responses:
                    continue
                responses.sort()
                score = responses[len(responses) // 2]
                if score > best_score:
                    best_score = score
                    best_offset = offset

            if best_score >= self.min_outer_edge_response:
                valid_sides += 1
            else:
                # Keep the model edge instead of following a weak background.
                best_offset = 0

            edge_scores.append(best_score)
            shifted_edges.append((
                (
                    p0[0] + normal_x * best_offset,
                    p0[1] + normal_y * best_offset,
                ),
                (
                    p1[0] + normal_x * best_offset,
                    p1[1] + normal_y * best_offset,
                ),
            ))

        refined = []
        for corner_index in range(4):
            previous_edge = shifted_edges[(corner_index - 1) % 4]
            current_edge = shifted_edges[corner_index]
            point = _line_intersection(
                previous_edge[0],
                previous_edge[1],
                current_edge[0],
                current_edge[1],
            )
            if point is None:
                point = predicted[corner_index]
            x = min(max(point[0], 0), image.width() - 1)
            y = min(max(point[1], 0), image.height() - 1)
            refined.append((int(round(x)), int(round(y))))

        refined = tuple(refined)
        if _polygon_area(refined) <= inner["area"] * 1.05:
            return None

        positive_scores = []
        for score in edge_scores:
            positive_scores.append(max(score, 0.0))
        mean_response = _mean(positive_scores)
        confidence = _clamp(
            (mean_response - self.min_outer_edge_response) / 55.0
        )
        return {
            "corners": refined,
            "bbox": _bbox(refined),
            "valid_sides": valid_sides,
            "edge_scores": tuple(edge_scores),
            "mean_response": mean_response,
            "confidence": confidence,
            "search_radius": search_radius,
        }

    def detect(self, image, roi=None):
        """Return a detection dictionary or None."""
        candidates = self._make_candidates(image, roi=roi)
        pair = self._best_pair(candidates)

        if pair is not None and pair["score"] >= self.min_pair_score:
            outer = pair["outer"]
            inner = pair["inner"]
            return {
                "mode": "PAIR",
                "outer_corners": outer["corners"],
                "inner_corners": inner["corners"],
                "outer_bbox": outer["bbox"],
                "inner_bbox": inner["bbox"],
                "score": pair["score"],
                "contrast": inner["contrast"],
                "short_ratio": pair["short_ratio"],
                "long_ratio": pair["long_ratio"],
                "area_ratio": pair["area_ratio"],
                "outer_valid_sides": 4,
                "outer_edge_response": 0.0,
                "candidate_count": len(candidates),
            }

        if not self.allow_inner_only:
            return None

        fallback = self._best_inner_only(candidates)
        if fallback is None or fallback["score"] < self.min_inner_score:
            return None

        inner = fallback["inner"]
        outer = self._refine_outer_from_inner(image, inner)
        if (
            outer is not None
            and outer["valid_sides"] >= self.min_outer_valid_sides
        ):
            score = 0.70 * fallback["score"] + 0.30 * outer["confidence"]
            return {
                "mode": "MODEL_PAIR",
                "outer_corners": outer["corners"],
                "inner_corners": inner["corners"],
                "outer_bbox": outer["bbox"],
                "inner_bbox": inner["bbox"],
                "score": score,
                "contrast": inner["contrast"],
                "short_ratio": self.expected_short_ratio,
                "long_ratio": self.expected_long_ratio,
                "area_ratio": self.expected_area_ratio,
                "outer_valid_sides": outer["valid_sides"],
                "outer_edge_response": outer["mean_response"],
                "candidate_count": len(candidates),
            }

        return {
            "mode": "INNER_ONLY",
            "outer_corners": None,
            "inner_corners": inner["corners"],
            "outer_bbox": None,
            "inner_bbox": inner["bbox"],
            "score": fallback["score"],
            "contrast": inner["contrast"],
            "short_ratio": 0.0,
            "long_ratio": 0.0,
            "area_ratio": 0.0,
            "outer_valid_sides": 0,
            "outer_edge_response": 0.0,
            "candidate_count": len(candidates),
        }
