"""Local 1080p edge refinement for the 2025 C A4 reference frame.

The low-resolution detector already supplies the topology and approximate
corners.  This module therefore never performs a full-frame rectangle search.
It samples only narrow normal-direction bands around the predicted inner and
outer edges, fits four robust lines, and intersects adjacent lines.

Corner order throughout the project is TL, TR, BR, BL.
"""

import math


def _distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def _polygon_area(points):
    total = 0.0
    for index in range(len(points)):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % len(points)]
        total += x0 * y1 - x1 * y0
    return abs(total) * 0.5


def _is_convex_quad(points):
    direction = 0
    for index in range(4):
        first = points[index]
        second = points[(index + 1) % 4]
        third = points[(index + 2) % 4]
        cross = (
            (second[0] - first[0]) * (third[1] - second[1])
            - (second[1] - first[1]) * (third[0] - second[0])
        )
        if abs(cross) < 1e-3:
            return False
        current = 1 if cross > 0.0 else -1
        if direction == 0:
            direction = current
        elif current != direction:
            return False
    return True


def _center(points):
    x = 0.0
    y = 0.0
    for point in points:
        x += point[0]
        y += point[1]
    count = len(points)
    return (x / count, y / count)


def _quad_dimensions(corners):
    sides = []
    for index in range(4):
        sides.append(_distance(corners[index], corners[(index + 1) % 4]))
    dimension_a = (sides[0] + sides[2]) * 0.5
    dimension_b = (sides[1] + sides[3]) * 0.5
    return min(dimension_a, dimension_b), max(dimension_a, dimension_b)


def _frame_disagreement(inner_corners, outer_corners, short_ratio, long_ratio):
    inner_short, inner_long = _quad_dimensions(inner_corners)
    outer_short, outer_long = _quad_dimensions(outer_corners)
    expected_inner_short = outer_short * short_ratio
    expected_inner_long = outer_long * long_ratio
    short_error = abs(inner_short - expected_inner_short) / max(
        expected_inner_short, 1e-6
    )
    long_error = abs(inner_long - expected_inner_long) / max(
        expected_inner_long, 1e-6
    )
    return (short_error + long_error) * 0.5


def _gray_from_pixel(pixel):
    # Some CanMV image formats return None from get_pixel().  In particular,
    # the GC2093 RGB888 capture buffer on the Yahboom firmware is suitable for
    # to_rgb565()/saving, but is not directly CPU-readable pixel by pixel.
    if pixel is None:
        return None

    if isinstance(pixel, (tuple, list)):
        if len(pixel) >= 3:
            if pixel[0] is None or pixel[1] is None or pixel[2] is None:
                return None
            return (
                0.299 * pixel[0]
                + 0.587 * pixel[1]
                + 0.114 * pixel[2]
            )
        if len(pixel) == 1:
            if pixel[0] is None:
                return None
            return float(pixel[0])

    value = int(pixel)
    if value <= 255:
        return float(value)

    red = ((value >> 11) & 0x1F) * 255.0 / 31.0
    green = ((value >> 5) & 0x3F) * 255.0 / 63.0
    blue = (value & 0x1F) * 255.0 / 31.0
    return 0.299 * red + 0.587 * green + 0.114 * blue


def _sample_gray(image, x, y):
    x = int(round(x))
    y = int(round(y))
    if x < 0 or y < 0 or x >= image.width() or y >= image.height():
        return None
    return _gray_from_pixel(image.get_pixel(x, y))


def _line_from_points(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-6:
        return None
    a = -dy / length
    b = dx / length
    c = -(a * p0[0] + b * p0[1])
    return (a, b, c)


def _fit_line(points):
    """Weighted total-least-squares line as normalized (a, b, c)."""
    if len(points) < 2:
        return None

    total_weight = 0.0
    center_x = 0.0
    center_y = 0.0
    for x, y, weight in points:
        weight = max(float(weight), 1.0)
        total_weight += weight
        center_x += x * weight
        center_y += y * weight
    if total_weight <= 0.0:
        return None
    center_x /= total_weight
    center_y /= total_weight

    xx = 0.0
    yy = 0.0
    xy = 0.0
    for x, y, weight in points:
        weight = max(float(weight), 1.0)
        dx = x - center_x
        dy = y - center_y
        xx += weight * dx * dx
        yy += weight * dy * dy
        xy += weight * dx * dy

    direction_angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
    direction_x = math.cos(direction_angle)
    direction_y = math.sin(direction_angle)
    a = -direction_y
    b = direction_x
    c = -(a * center_x + b * center_y)
    return (a, b, c)


def _line_intersection(first, second):
    a0, b0, c0 = first
    a1, b1, c1 = second
    determinant = a0 * b1 - a1 * b0
    if abs(determinant) < 1e-6:
        return None
    x = (b0 * c1 - b1 * c0) / determinant
    y = (c0 * a1 - c1 * a0) / determinant
    return (x, y)


def _scale_corners(corners, scale_x, scale_y):
    if corners is None:
        return None
    result = []
    for x, y in corners:
        result.append((float(x) * scale_x, float(y) * scale_y))
    return tuple(result)


def _predict_outer(inner_corners, short_ratio, long_ratio):
    tl, tr, br, bl = inner_corners
    short_margin = (1.0 / short_ratio - 1.0) * 0.5
    long_margin = (1.0 / long_ratio - 1.0) * 0.5
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


def _bilinear_point(corners, u, v):
    tl, tr, br, bl = corners
    top_x = tl[0] + (tr[0] - tl[0]) * u
    top_y = tl[1] + (tr[1] - tl[1]) * u
    bottom_x = bl[0] + (br[0] - bl[0]) * u
    bottom_y = bl[1] + (br[1] - bl[1]) * u
    return (
        top_x + (bottom_x - top_x) * v,
        top_y + (bottom_y - top_y) * v,
    )


def _predict_inner(outer_corners, short_ratio, long_ratio):
    short_margin = (1.0 - short_ratio) * 0.5
    long_margin = (1.0 - long_ratio) * 0.5
    return (
        _bilinear_point(outer_corners, short_margin, long_margin),
        _bilinear_point(outer_corners, 1.0 - short_margin, long_margin),
        _bilinear_point(
            outer_corners,
            1.0 - short_margin,
            1.0 - long_margin,
        ),
        _bilinear_point(outer_corners, short_margin, 1.0 - long_margin),
    )


def _quad_short_side(corners):
    sides = []
    for index in range(4):
        sides.append(_distance(corners[index], corners[(index + 1) % 4]))
    return min(
        (sides[0] + sides[2]) * 0.5,
        (sides[1] + sides[3]) * 0.5,
    )


def _refine_edge(
    image,
    p0,
    p1,
    quad_center,
    polarity,
    search_radius,
    probe,
    sample_count,
    min_response,
):
    """Find edge points around one predicted side and fit a robust line.

    polarity > 0: brighter toward quad centre (inner black-to-white edge).
    polarity < 0: darker toward quad centre (outer background-to-black edge).
    """
    midpoint_x = (p0[0] + p1[0]) * 0.5
    midpoint_y = (p0[1] + p1[1]) * 0.5
    normal_x = quad_center[0] - midpoint_x
    normal_y = quad_center[1] - midpoint_y
    normal_length = math.sqrt(normal_x * normal_x + normal_y * normal_y)
    if normal_length < 1e-6:
        return None
    normal_x /= normal_length
    normal_y /= normal_length

    points = []
    response_total = 0.0
    for sample_index in range(sample_count):
        position = 0.10 + 0.80 * (sample_index + 0.5) / sample_count
        base_x = p0[0] + (p1[0] - p0[0]) * position
        base_y = p0[1] + (p1[1] - p0[1]) * position

        best_offset = 0
        best_offset_sum = 0.0
        best_offset_count = 0
        best_response = -9999.0
        for offset in range(-search_radius, search_radius + 1):
            inside = _sample_gray(
                image,
                base_x + normal_x * (offset + probe),
                base_y + normal_y * (offset + probe),
            )
            outside = _sample_gray(
                image,
                base_x + normal_x * (offset - probe),
                base_y + normal_y * (offset - probe),
            )
            if inside is None or outside is None:
                continue
            response = inside - outside
            if polarity < 0:
                response = -response
            if response > best_response + 0.5:
                best_response = response
                best_offset = offset
                best_offset_sum = float(offset)
                best_offset_count = 1
            elif abs(response - best_response) <= 0.5:
                # A symmetric two-probe step response has a short flat top.
                # Averaging tied offsets removes the otherwise systematic
                # outward bias caused by always keeping the first maximum.
                best_offset_sum += offset
                best_offset_count += 1

        if best_offset_count > 1:
            best_offset = int(round(best_offset_sum / best_offset_count))

        if best_response >= min_response:
            points.append((
                base_x + normal_x * best_offset,
                base_y + normal_y * best_offset,
                best_response,
            ))
            response_total += best_response

    minimum_points = max(6, sample_count // 2)
    if len(points) < minimum_points:
        return None

    line = _fit_line(points)
    if line is None:
        return None

    # Reject isolated texture edges and refit.  Total least squares handles
    # vertical and horizontal sides without special cases.
    filtered = []
    residual_limit = max(2.5, search_radius * 0.12)
    for point in points:
        residual = abs(line[0] * point[0] + line[1] * point[1] + line[2])
        if residual <= residual_limit:
            filtered.append(point)
    if len(filtered) >= minimum_points:
        refined_line = _fit_line(filtered)
        if refined_line is not None:
            line = refined_line
            points = filtered
            response_total = 0.0
            for point in points:
                response_total += point[2]

    residual_square_total = 0.0
    for point in points:
        residual = line[0] * point[0] + line[1] * point[1] + line[2]
        residual_square_total += residual * residual
    edge_rms = math.sqrt(residual_square_total / max(len(points), 1))

    return {
        "line": line,
        "point_count": len(points),
        "mean_response": response_total / max(len(points), 1),
        "rms": edge_rms,
    }


def _refine_quad(
    image,
    predicted,
    polarity,
    search_radius,
    probe,
    sample_count,
    min_response,
):
    quad_center = _center(predicted)
    lines = []
    valid_sides = 0
    response_total = 0.0
    point_total = 0
    residual_square_total = 0.0

    for side in range(4):
        p0 = predicted[side]
        p1 = predicted[(side + 1) % 4]
        edge = _refine_edge(
            image,
            p0,
            p1,
            quad_center,
            polarity,
            search_radius,
            probe,
            sample_count,
            min_response,
        )
        if edge is None:
            lines.append(_line_from_points(p0, p1))
        else:
            lines.append(edge["line"])
            valid_sides += 1
            response_total += edge["mean_response"]
            point_total += edge["point_count"]
            residual_square_total += (
                edge["rms"] * edge["rms"] * edge["point_count"]
            )

    corners = []
    for index in range(4):
        point = _line_intersection(lines[(index - 1) % 4], lines[index])
        if point is None:
            point = predicted[index]
        corners.append(point)
    corners = tuple(corners)

    predicted_area = _polygon_area(predicted)
    refined_area = _polygon_area(corners)
    valid_geometry = (
        predicted_area > 1.0
        and refined_area >= predicted_area * 0.65
        and refined_area <= predicted_area * 1.45
        and _is_convex_quad(corners)
    )
    max_corner_shift = 0.0
    for index in range(4):
        corner_shift = _distance(corners[index], predicted[index])
        max_corner_shift = max(max_corner_shift, corner_shift)
        if corner_shift > search_radius * 2.2:
            valid_geometry = False
            break
    raw_valid_sides = valid_sides
    raw_mean_response = response_total / max(raw_valid_sides, 1)
    raw_edge_rms = -1.0
    if point_total > 0:
        raw_edge_rms = math.sqrt(residual_square_total / point_total)
    if not valid_geometry:
        corners = predicted
        valid_sides = 0
        point_total = 0

    return {
        "corners": corners,
        "lines": tuple(lines),
        "valid_sides": valid_sides,
        "raw_valid_sides": raw_valid_sides,
        "geometry_valid": valid_geometry,
        "point_count": point_total,
        "mean_response": raw_mean_response if valid_geometry else 0.0,
        "raw_mean_response": raw_mean_response,
        "edge_rms": raw_edge_rms if valid_geometry else -1.0,
        "raw_edge_rms": raw_edge_rms,
        "max_corner_shift": max_corner_shift,
        "search_radius": search_radius,
    }


def _quad_result_score(result):
    if result is None or not result["geometry_valid"]:
        return -1.0
    return (
        result["valid_sides"] * 100.0
        + min(result["mean_response"], 255.0)
        + min(result["point_count"], 96) * 0.05
    )


def _adaptive_refine_quad(
    image,
    predicted,
    polarity,
    narrow_radius,
    wide_radius,
    sample_count,
    min_response,
    force_wide=False,
):
    narrow_probe = max(2, min(4, narrow_radius // 5))
    best = _refine_quad(
        image,
        predicted,
        polarity,
        narrow_radius,
        narrow_probe,
        sample_count,
        min_response,
    )

    if force_wide or best["valid_sides"] < 3:
        wide_probe = max(2, min(5, wide_radius // 8))
        wide = _refine_quad(
            image,
            predicted,
            polarity,
            wide_radius,
            wide_probe,
            sample_count,
            min_response,
        )
        if _quad_result_score(wide) > _quad_result_score(best):
            best = wide
    return best


class HighResRefiner:
    """Refine a coarse A4 frame result on a same-aspect high-res image."""

    def __init__(
        self,
        expected_short_ratio=17.0 / 21.0,
        expected_long_ratio=25.7 / 29.7,
        sample_count=24,
        min_response=12.0,
    ):
        self.expected_short_ratio = expected_short_ratio
        self.expected_long_ratio = expected_long_ratio
        self.sample_count = sample_count
        self.min_response = min_response

    def refine(self, image, coarse_result, coarse_width, coarse_height):
        if coarse_result is None or coarse_result["inner_corners"] is None:
            return None

        scale_x = image.width() / coarse_width
        scale_y = image.height() / coarse_height
        predicted_inner = _scale_corners(
            coarse_result["inner_corners"], scale_x, scale_y
        )
        short_side = _quad_short_side(predicted_inner)
        narrow_radius = int(round(short_side * 0.075))
        narrow_radius = max(10, min(narrow_radius, 32))
        wide_radius = int(round(short_side * 0.22))
        wide_radius = max(narrow_radius + 14, min(wide_radius, 56))

        inner = _adaptive_refine_quad(
            image,
            predicted_inner,
            1,
            narrow_radius,
            wide_radius,
            self.sample_count,
            self.min_response,
        )
        inner_corners = inner["corners"]

        model_outer = _predict_outer(
            inner_corners,
            self.expected_short_ratio,
            self.expected_long_ratio,
        )
        outer_seeds = [model_outer]
        coarse_outer = coarse_result.get("outer_corners")
        if coarse_outer is not None:
            scaled_coarse_outer = _scale_corners(
                coarse_outer, scale_x, scale_y
            )
            seed_difference = 0.0
            for index in range(4):
                seed_difference += _distance(
                    model_outer[index], scaled_coarse_outer[index]
                )
            if seed_difference / 4.0 >= 2.0:
                outer_seeds.append(scaled_coarse_outer)

        outer = None
        outer_selection_score = -999999.0
        for predicted_outer in outer_seeds:
            candidate_outer = _adaptive_refine_quad(
                image,
                predicted_outer,
                -1,
                narrow_radius,
                wide_radius,
                self.sample_count,
                self.min_response,
                force_wide=inner["valid_sides"] < 3,
            )
            candidate_score = _quad_result_score(candidate_outer)
            if (
                inner["valid_sides"] >= 3
                and candidate_outer["valid_sides"] >= 3
            ):
                candidate_disagreement = _frame_disagreement(
                    inner_corners,
                    candidate_outer["corners"],
                    self.expected_short_ratio,
                    self.expected_long_ratio,
                )
                candidate_score -= min(candidate_disagreement, 1.0) * 300.0
            if outer is None or candidate_score > outer_selection_score:
                outer = candidate_outer
                outer_selection_score = candidate_score

        outer_corners = outer["corners"]
        frame_disagreement = _frame_disagreement(
            inner_corners,
            outer_corners,
            self.expected_short_ratio,
            self.expected_long_ratio,
        )

        mode = "HIGH_RES_FALLBACK"
        inner_good = inner["valid_sides"] >= 3
        outer_good = outer["valid_sides"] >= 3
        if inner_good and outer_good and frame_disagreement <= 0.08:
            mode = "HIGH_RES_PAIR"
        elif inner_good and outer_good:
            # The two independently detected quads disagree with the known
            # 17/21 and 25.7/29.7 frame ratios.  Keep the stronger quad and
            # reconstruct the other instead of reporting a self-inconsistent
            # pair.
            if _quad_result_score(inner) >= _quad_result_score(outer):
                mode = "HIGH_RES_INNER"
                outer_corners = _predict_outer(
                    inner_corners,
                    self.expected_short_ratio,
                    self.expected_long_ratio,
                )
                outer_good = False
            else:
                mode = "HIGH_RES_OUTER"
                inner_corners = _predict_inner(
                    outer_corners,
                    self.expected_short_ratio,
                    self.expected_long_ratio,
                )
                inner_good = False
            frame_disagreement = 0.0
        elif inner_good:
            mode = "HIGH_RES_INNER"
            outer_corners = _predict_outer(
                inner_corners,
                self.expected_short_ratio,
                self.expected_long_ratio,
            )
            outer_good = False
            frame_disagreement = 0.0
        elif outer_good:
            mode = "HIGH_RES_OUTER"
            inner_corners = _predict_inner(
                outer_corners,
                self.expected_short_ratio,
                self.expected_long_ratio,
            )
            inner_good = False
            frame_disagreement = 0.0

        inner_valid_sides = inner["valid_sides"] if inner_good else 0
        outer_valid_sides = outer["valid_sides"] if outer_good else 0
        measurement_valid = mode != "HIGH_RES_FALLBACK"
        mode_rank = {
            "HIGH_RES_PAIR": 300.0,
            "HIGH_RES_INNER": 200.0,
            "HIGH_RES_OUTER": 200.0,
            "HIGH_RES_FALLBACK": 0.0,
        }[mode]
        quality_score = max(
            0.0,
            mode_rank
            + (inner_valid_sides + outer_valid_sides) * 10.0
            + min(inner["mean_response"], 255.0) * 0.05
            + min(outer["mean_response"], 255.0) * 0.05
            - frame_disagreement * 100.0,
        )
        selected_radius = max(
            inner["search_radius"], outer["search_radius"]
        )

        return {
            "mode": mode,
            # Keep sub-pixel intersections for geometry/scale statistics.
            # The integer variants remain for drawing and compatibility with
            # the existing single-shot measurement program.
            "inner_corners_float": tuple(
                (float(x), float(y)) for x, y in inner_corners
            ),
            "outer_corners_float": tuple(
                (float(x), float(y)) for x, y in outer_corners
            ),
            "inner_corners": tuple(
                (int(round(x)), int(round(y))) for x, y in inner_corners
            ),
            "outer_corners": tuple(
                (int(round(x)), int(round(y)))
                for x, y in outer_corners
            ),
            "inner_valid_sides": inner_valid_sides,
            "outer_valid_sides": outer_valid_sides,
            "inner_edge_response": inner["mean_response"],
            "outer_edge_response": outer["mean_response"],
            "inner_edge_rms": (
                inner["edge_rms"] if inner_valid_sides > 0 else -1.0
            ),
            "outer_edge_rms": (
                outer["edge_rms"] if outer_valid_sides > 0 else -1.0
            ),
            "inner_point_count": inner["point_count"],
            "outer_point_count": outer["point_count"],
            "inner_geometry_valid": inner["geometry_valid"],
            "outer_geometry_valid": outer["geometry_valid"],
            "inner_raw_valid_sides": inner["raw_valid_sides"],
            "outer_raw_valid_sides": outer["raw_valid_sides"],
            "inner_search_radius": inner["search_radius"],
            "outer_search_radius": outer["search_radius"],
            "search_radius": selected_radius,
            "frame_model_disagreement": frame_disagreement,
            "quality_score": quality_score,
            "measurement_valid": measurement_valid,
            "scale_x": scale_x,
            "scale_y": scale_y,
        }
