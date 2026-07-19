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
        self.min_inner_score = 0.50
        self.min_outer_edge_response = 12.0
        self.min_outer_valid_sides = 3

        self.last_candidates = []
        self.last_pair_count = 0
        self.last_raw_rect_count = 0
        self.last_candidate_errors = 0
        self.last_roi = None
        self.last_threshold = rect_threshold

    def _candidate_from_rect(self, image, rect):
        corners = _order_corners(rect.corners())
        area = _polygon_area(corners)
        image_area = image.width() * image.height()
        area_ratio = area / image_area

        if area_ratio < self.min_area_ratio or area_ratio > self.max_area_ratio:
            return None

        side_0 = _distance(corners[0], corners[1])
        side_1 = _distance(corners[1], corners[2])
        side_2 = _distance(corners[2], corners[3])
        side_3 = _distance(corners[3], corners[0])

        dimension_a = (side_0 + side_2) * 0.5
        dimension_b = (side_1 + side_3) * 0.5
        short_side = min(dimension_a, dimension_b)
        long_side = max(dimension_a, dimension_b)
        if short_side < self.min_short_side:
            return None

        aspect = long_side / short_side
        if aspect < 1.10 or aspect > 1.85:
            return None

        center = _polygon_center(corners)
        inner_mean, outer_mean, contrast = self._inner_boundary_contrast(
            image, corners
        )

        base_score = max(
            _closeness(aspect, self.expected_outer_aspect, 0.42),
            _closeness(aspect, self.expected_inner_aspect, 0.42),
        )

        return {
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
        }

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

    def _make_candidates(self, image, roi=None):
        self.last_roi = roi
        threshold = self.rect_threshold
        if roi is not None:
            # find_rects produced almost no candidates in a small tracking ROI
            # with the full-frame threshold.  A lower local threshold is safe
            # because the ROI already restricts the search around a known lock.
            threshold = max(
                500,
                int(self.rect_threshold * self.roi_threshold_scale),
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
                candidate = self._candidate_from_rect(image, rect)
                if candidate is not None:
                    candidates.append(candidate)
            except Exception:
                # A malformed rectangle must not stop the live preview.
                self.last_candidate_errors += 1

        candidates.sort(key=lambda item: item["base_score"], reverse=True)
        if len(candidates) > self.max_candidates:
            candidates = candidates[:self.max_candidates]

        self.last_candidates = candidates
        return candidates

    def _score_pair(self, outer, inner):
        if outer["area"] <= inner["area"] * 1.05:
            return None

        if not _point_in_polygon(inner["center"], outer["corners"]):
            return None

        inside_count = 0
        for point in inner["corners"]:
            if _point_in_polygon(point, outer["corners"]):
                inside_count += 1
        if inside_count < 3:
            return None

        short_ratio = inner["short_side"] / outer["short_side"]
        long_ratio = inner["long_side"] / outer["long_side"]
        area_ratio = inner["area"] / outer["area"]

        if short_ratio < 0.52 or short_ratio > 0.97:
            return None
        if long_ratio < 0.58 or long_ratio > 0.98:
            return None
        if inner["contrast"] < self.min_inner_contrast:
            return None

        outer_diagonal = math.sqrt(
            outer["short_side"] * outer["short_side"]
            + outer["long_side"] * outer["long_side"]
        )
        center_error = _distance(outer["center"], inner["center"])
        center_error /= max(outer_diagonal, 1.0)
        if center_error > 0.18:
            return None

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

        score = (
            0.20 * short_score
            + 0.20 * long_score
            + 0.10 * area_score
            + 0.12 * center_score
            + 0.10 * outer_aspect_score
            + 0.10 * inner_aspect_score
            + 0.18 * contrast_score
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
        best = None
        pair_count = 0
        for outer in candidates:
            for inner in candidates:
                if outer is inner:
                    continue
                pair = self._score_pair(outer, inner)
                if pair is None:
                    continue
                pair_count += 1
                if best is None or pair["score"] > best["score"]:
                    best = pair

        self.last_pair_count = pair_count
        return best

    def _best_inner_only(self, candidates):
        best = None
        for candidate in candidates:
            if candidate["contrast"] < self.min_inner_contrast:
                continue

            aspect_score = _closeness(
                candidate["aspect"], self.expected_inner_aspect, 0.42
            )
            contrast_score = _clamp(
                (candidate["contrast"] - self.min_inner_contrast) / 60.0
            )
            score = 0.45 * aspect_score + 0.55 * contrast_score

            if best is None or score > best["score"]:
                best = {
                    "inner": candidate,
                    "score": score,
                }
        return best

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
