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


def _relative_difference(first, second):
    return abs(first - second) / max((first + second) * 0.5, 1e-6)


def _quad_regularity(corners, expected_aspect):
    """Describe whether a quad is plausible for a nearly front-facing sheet.

    This is deliberately separate from the generic localization validity.  A
    strongly tilted target can still be localized for the perspective task,
    but it must not be used by the parallel-plane distance calibration path.
    """
    sides = []
    for index in range(4):
        sides.append(_distance(corners[index], corners[(index + 1) % 4]))
    dimension_a = (sides[0] + sides[2]) * 0.5
    dimension_b = (sides[1] + sides[3]) * 0.5
    short_side = min(dimension_a, dimension_b)
    long_side = max(dimension_a, dimension_b)
    fill_ratio = _polygon_area(corners) / max(
        dimension_a * dimension_b, 1e-6
    )
    opposite_error_a = _relative_difference(sides[0], sides[2])
    opposite_error_b = _relative_difference(sides[1], sides[3])

    # Normalize both physical axes to an arbitrary one-unit short side.  Their
    # mismatch is the scale anisotropy expected to stay small when the target
    # plane is approximately parallel to the image plane.
    short_scale = short_side
    long_scale = long_side / max(expected_aspect, 1e-6)
    scale_anisotropy = _relative_difference(short_scale, long_scale)
    return {
        "fill_ratio": fill_ratio,
        "opposite_error_a": opposite_error_a,
        "opposite_error_b": opposite_error_b,
        "max_opposite_error": max(opposite_error_a, opposite_error_b),
        "scale_anisotropy": scale_anisotropy,
    }


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


def _median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return float(ordered[middle])
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def _robust_fit_line(points, minimum_points, residual_cap):
    """Two-pass MAD/Huber refit without assuming horizontal/vertical edges."""
    working = points
    line = _fit_line(working)
    if line is None:
        return None, points

    for _ in range(2):
        residuals = []
        for point in working:
            residuals.append(abs(
                line[0] * point[0]
                + line[1] * point[1]
                + line[2]
            ))
        residual_median = _median(residuals)
        deviations = []
        for residual in residuals:
            deviations.append(abs(residual - residual_median))
        robust_sigma = max(0.25, _median(deviations) * 1.4826)
        residual_limit = max(
            0.75,
            residual_median + robust_sigma * 2.8,
        )
        residual_limit = min(residual_limit, residual_cap)

        filtered = []
        weighted = []
        huber_limit = robust_sigma * 1.5
        for index in range(len(working)):
            point = working[index]
            residual = residuals[index]
            if residual > residual_limit:
                continue
            filtered.append(point)
            huber_weight = 1.0
            if residual > huber_limit:
                huber_weight = huber_limit / max(residual, 1e-6)
            weighted.append((
                point[0],
                point[1],
                point[2] * huber_weight,
            ))

        if len(filtered) < minimum_points:
            break
        refined_line = _fit_line(weighted)
        if refined_line is None:
            break
        working = filtered
        line = refined_line

    return line, working


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


def _regularize_inner_seed(corners, expected_aspect):
    """Replace malformed corners with a center/area/orientation rectangle.

    A low-resolution ROI_SEED is evidence for *where* the frame is, not for
    the exact position of each corner.  The vertex mean and polygon area stay
    useful even when find_rects() substitutes one diagonal edge.  Folding all
    edge directions modulo 90 degrees also makes the orientation estimate
    robust to one outlier edge.  The returned physical-model rectangle is only
    an initialization for wide high-resolution edge fitting.
    """
    area = _polygon_area(corners)
    if area <= 1.0 or expected_aspect <= 1.0:
        return None, None

    center_x = 0.0
    center_y = 0.0
    orientation_x = 0.0
    orientation_y = 0.0
    for index in range(4):
        point = corners[index]
        following = corners[(index + 1) % 4]
        center_x += point[0]
        center_y += point[1]
        angle = math.atan2(
            following[1] - point[1], following[0] - point[0]
        )
        orientation_x += math.cos(4.0 * angle)
        orientation_y += math.sin(4.0 * angle)
    center_x *= 0.25
    center_y *= 0.25
    angle = 0.25 * math.atan2(orientation_y, orientation_x)

    short_side = math.sqrt(area / expected_aspect)
    long_side = short_side * expected_aspect
    unit_x = math.cos(angle)
    unit_y = math.sin(angle)
    vertical_x = -unit_y
    vertical_y = unit_x
    half_short = short_side * 0.5
    half_long = long_side * 0.5
    regularized = (
        (
            center_x - unit_x * half_short - vertical_x * half_long,
            center_y - unit_y * half_short - vertical_y * half_long,
        ),
        (
            center_x + unit_x * half_short - vertical_x * half_long,
            center_y + unit_y * half_short - vertical_y * half_long,
        ),
        (
            center_x + unit_x * half_short + vertical_x * half_long,
            center_y + unit_y * half_short + vertical_y * half_long,
        ),
        (
            center_x - unit_x * half_short + vertical_x * half_long,
            center_y - unit_y * half_short + vertical_y * half_long,
        ),
    )

    max_nearest_shift = 0.0
    for regularized_corner in regularized:
        nearest = None
        for original_corner in corners:
            shift = _distance(regularized_corner, original_corner)
            if nearest is None or shift < nearest:
                nearest = shift
        max_nearest_shift = max(max_nearest_shift, nearest or 0.0)

    return regularized, {
        "center": (center_x, center_y),
        "short_side": short_side,
        "long_side": long_side,
        "angle_degrees": angle * 180.0 / math.pi,
        "max_nearest_corner_shift": max_nearest_shift,
    }


def _expanded_quad_roi(corners, image_width, image_height, margin_scale=0.45):
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    short_side = _quad_short_side(corners)
    margin = max(12, int(round(short_side * margin_scale)))
    x0 = max(0, int(math.floor(min(xs))) - margin)
    y0 = max(0, int(math.floor(min(ys))) - margin)
    x1 = min(image_width - 1, int(math.ceil(max(xs))) + margin)
    y1 = min(image_height - 1, int(math.ceil(max(ys))) + margin)
    return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)


def _quad_in_bounds(corners, width, height, margin=2.0):
    maximum_x = width - 1.0 - margin
    maximum_y = height - 1.0 - margin
    for x, y in corners:
        if x < margin or y < margin or x > maximum_x or y > maximum_y:
            return False
    return True


def _side_point(corners, side, position):
    first = corners[side]
    second = corners[(side + 1) % 4]
    return (
        first[0] + (second[0] - first[0]) * position,
        first[1] + (second[1] - first[1]) * position,
    )


def _border_ring_metrics(
    image,
    inner_corners,
    expected_outer_corners,
    samples_per_side=8,
    contrast_threshold=15.0,
):
    """Verify the white-black-white A4 border signature around the inner quad."""
    inside_contrast_total = 0.0
    outside_contrast_total = 0.0
    thickness_total = 0.0
    inside_pass_count = 0
    outside_pass_count = 0
    valid_count = 0
    side_inside_ratios = []

    for side in range(4):
        side_valid = 0
        side_inside_pass = 0
        for sample_index in range(samples_per_side):
            position = (
                0.12
                + 0.76 * (sample_index + 0.5) / samples_per_side
            )
            inner_point = _side_point(inner_corners, side, position)
            outer_point = _side_point(
                expected_outer_corners, side, position
            )
            vector_x = outer_point[0] - inner_point[0]
            vector_y = outer_point[1] - inner_point[1]
            thickness = math.sqrt(
                vector_x * vector_x + vector_y * vector_y
            )
            if thickness < 3.0:
                continue
            unit_x = vector_x / thickness
            unit_y = vector_y / thickness
            probe = max(2.0, min(5.0, thickness * 0.20))

            inside_gray = _sample_gray(
                image,
                inner_point[0] - unit_x * probe,
                inner_point[1] - unit_y * probe,
            )
            border_total = 0.0
            border_count = 0
            for fraction in (0.30, 0.50, 0.70):
                border_gray = _sample_gray(
                    image,
                    inner_point[0] + vector_x * fraction,
                    inner_point[1] + vector_y * fraction,
                )
                if border_gray is not None:
                    border_total += border_gray
                    border_count += 1
            outside_gray = _sample_gray(
                image,
                outer_point[0] + unit_x * probe,
                outer_point[1] + unit_y * probe,
            )
            if (
                inside_gray is None
                or outside_gray is None
                or border_count < 2
            ):
                continue

            border_gray = border_total / border_count
            inside_contrast = inside_gray - border_gray
            outside_contrast = outside_gray - border_gray
            inside_contrast_total += inside_contrast
            outside_contrast_total += outside_contrast
            thickness_total += thickness
            valid_count += 1
            side_valid += 1
            if inside_contrast >= contrast_threshold:
                inside_pass_count += 1
                side_inside_pass += 1
            if outside_contrast >= contrast_threshold * 0.5:
                outside_pass_count += 1

        side_inside_ratios.append(
            side_inside_pass / max(side_valid, 1)
        )

    if valid_count <= 0:
        return {
            "valid_samples": 0,
            "inside_contrast": -999.0,
            "outside_contrast": -999.0,
            "inside_pass_ratio": 0.0,
            "outside_pass_ratio": 0.0,
            "min_side_inside_pass_ratio": 0.0,
            "mean_thickness": 0.0,
        }
    return {
        "valid_samples": valid_count,
        "inside_contrast": inside_contrast_total / valid_count,
        "outside_contrast": outside_contrast_total / valid_count,
        "inside_pass_ratio": inside_pass_count / valid_count,
        "outside_pass_ratio": outside_pass_count / valid_count,
        "min_side_inside_pass_ratio": min(side_inside_ratios),
        "mean_thickness": thickness_total / valid_count,
    }


def _subpixel_peak_offset(offsets, responses, best_index):
    """Center a flat gradient peak or interpolate a sharp one."""
    best_response = responses[best_index]
    plateau_drop = max(0.75, min(3.0, best_response * 0.025))
    plateau_threshold = best_response - plateau_drop
    left = best_index
    right = best_index
    while left > 0 and responses[left - 1] >= plateau_threshold:
        left -= 1
    while (
        right + 1 < len(responses)
        and responses[right + 1] >= plateau_threshold
    ):
        right += 1

    if right > left:
        # The symmetric two-probe response of a step edge has a flat top.
        # Keeping this midpoint as a float avoids the former half-pixel loss.
        return (offsets[left] + offsets[right]) * 0.5

    if best_index <= 0 or best_index + 1 >= len(responses):
        return float(offsets[best_index])
    previous = responses[best_index - 1]
    current = responses[best_index]
    following = responses[best_index + 1]
    denominator = previous - 2.0 * current + following
    if denominator >= -1e-6:
        return float(offsets[best_index])
    delta = 0.5 * (previous - following) / denominator
    delta = max(-0.75, min(delta, 0.75))
    return float(offsets[best_index]) + delta


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

        response_offsets = []
        response_values = []
        best_index = -1
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
            response_offsets.append(offset)
            response_values.append(response)
            if response > best_response:
                best_response = response
                best_index = len(response_values) - 1

        if best_index >= 0 and best_response >= min_response:
            best_offset = _subpixel_peak_offset(
                response_offsets,
                response_values,
                best_index,
            )
            points.append((
                base_x + normal_x * best_offset,
                base_y + normal_y * best_offset,
                best_response,
            ))
            response_total += best_response

    minimum_points = max(6, sample_count // 2)
    if len(points) < minimum_points:
        return None

    residual_cap = max(2.5, search_radius * 0.12)
    line, points = _robust_fit_line(points, minimum_points, residual_cap)
    if line is None:
        return None
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


def _quad_result_selection_key(result, max_acceptable_rms=None):
    """Rank two refinements without rewarding a noisy stronger edge.

    Geometry and four-side completeness remain mandatory.  Once those match,
    an in-gate line fit and then lower RMS take precedence over raw response.
    """
    if result is None:
        return (0, 0, 0, -999999.0, -999999.0, 0)
    rms = result.get("edge_rms", -1.0)
    rms_available = rms >= 0.0
    rms_acceptable = rms_available and (
        max_acceptable_rms is None or rms <= max_acceptable_rms
    )
    rms_score = -rms if rms_available else -999999.0
    return (
        1 if result.get("geometry_valid", False) else 0,
        result.get("valid_sides", 0),
        1 if rms_acceptable else 0,
        rms_score,
        result.get("mean_response", 0.0),
        result.get("point_count", 0),
    )


def _select_quad_refinement(narrow, wide, max_acceptable_rms=None):
    """Choose the geometrically complete result with the cleaner line fit."""
    if wide is None:
        return narrow, "NARROW"
    narrow_key = _quad_result_selection_key(narrow, max_acceptable_rms)
    wide_key = _quad_result_selection_key(wide, max_acceptable_rms)
    if wide_key > narrow_key:
        return wide, "WIDE"
    return narrow, "NARROW"


def _adaptive_refine_quad(
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
    narrow_probe = max(2, min(4, narrow_radius // 5))
    wide_search_used = False
    narrow = _refine_quad(
        image,
        predicted,
        polarity,
        narrow_radius,
        narrow_probe,
        sample_count,
        min_response,
    )

    wide_reason = "NOT_NEEDED"
    if force_wide:
        wide_reason = "SEED_SHIFT"
    elif not narrow["geometry_valid"]:
        wide_reason = "GEOMETRY"
    elif narrow["valid_sides"] < 4:
        wide_reason = "SIDES"
    elif narrow.get("edge_rms", -1.0) < 0.0:
        wide_reason = "RMS_MISSING"
    elif (
        max_acceptable_rms is not None
        and narrow["edge_rms"] > max_acceptable_rms
    ):
        wide_reason = "RMS"

    wide = None
    if wide_reason != "NOT_NEEDED":
        wide_search_used = True
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
    best, selected_path = _select_quad_refinement(
        narrow,
        wide,
        max_acceptable_rms,
    )
    # Record both branches even when the expensive wide result was discarded.
    best["wide_search_used"] = wide_search_used
    best["wide_search_reason"] = wide_reason
    best["refine_path"] = selected_path
    best["narrow_edge_rms"] = narrow.get("edge_rms", -1.0)
    best["narrow_valid_sides"] = narrow.get("valid_sides", 0)
    best["wide_edge_rms"] = (
        wide.get("edge_rms", -1.0) if wide is not None else -1.0
    )
    best["wide_valid_sides"] = (
        wide.get("valid_sides", 0) if wide is not None else 0
    )
    return best


class HighResRefiner:
    """Refine a coarse A4 frame result on a same-aspect high-res image."""

    def __init__(
        self,
        expected_short_ratio=17.0 / 21.0,
        expected_long_ratio=25.7 / 29.7,
        expected_inner_aspect=25.7 / 17.0,
        expected_outer_aspect=29.7 / 21.0,
        sample_count=24,
        min_response=12.0,
        max_inner_edge_rms=1.0,
        max_weak_inner_edge_rms=1.5,
        max_outer_edge_rms=5.0,
        min_inner_fill_ratio=0.97,
        max_inner_opposite_error=0.12,
        max_inner_scale_anisotropy=0.08,
        max_frame_disagreement=0.04,
        min_ring_inside_contrast=20.0,
        min_ring_inside_pass_ratio=0.75,
        min_ring_side_pass_ratio=0.50,
        min_conflict_ring_samples=24,
        min_conflict_ring_inside_pass_ratio=0.85,
        min_conflict_ring_side_pass_ratio=0.50,
        min_conflict_ring_outside_pass_ratio=0.50,
    ):
        self.expected_short_ratio = expected_short_ratio
        self.expected_long_ratio = expected_long_ratio
        self.expected_inner_aspect = expected_inner_aspect
        self.expected_outer_aspect = expected_outer_aspect
        self.sample_count = sample_count
        self.min_response = min_response
        self.max_inner_edge_rms = max_inner_edge_rms
        self.max_weak_inner_edge_rms = max_weak_inner_edge_rms
        self.max_outer_edge_rms = max_outer_edge_rms
        self.min_inner_fill_ratio = min_inner_fill_ratio
        self.max_inner_opposite_error = max_inner_opposite_error
        self.max_inner_scale_anisotropy = max_inner_scale_anisotropy
        self.max_frame_disagreement = max_frame_disagreement
        self.min_ring_inside_contrast = min_ring_inside_contrast
        self.min_ring_inside_pass_ratio = min_ring_inside_pass_ratio
        self.min_ring_side_pass_ratio = min_ring_side_pass_ratio
        self.min_conflict_ring_samples = min_conflict_ring_samples
        self.min_conflict_ring_inside_pass_ratio = (
            min_conflict_ring_inside_pass_ratio
        )
        self.min_conflict_ring_side_pass_ratio = (
            min_conflict_ring_side_pass_ratio
        )
        self.min_conflict_ring_outside_pass_ratio = (
            min_conflict_ring_outside_pass_ratio
        )

    def refine(
        self,
        image,
        coarse_result,
        coarse_width,
        coarse_height,
        preserve_projective_seed=False,
        localization_only=False,
        max_inner_scale_anisotropy=None,
    ):
        if coarse_result is None or coarse_result["inner_corners"] is None:
            return None

        scale_x = image.width() / coarse_width
        scale_y = image.height() / coarse_height
        original_predicted_inner = _scale_corners(
            coarse_result["inner_corners"], scale_x, scale_y
        )
        seed_class = coarse_result.get("seed_class", "CLEAN_SEED")
        predicted_inner = original_predicted_inner
        seed_regularized = False
        seed_regularization = None
        if seed_class == "ROI_SEED" and not preserve_projective_seed:
            regularized, seed_regularization = _regularize_inner_seed(
                original_predicted_inner,
                self.expected_inner_aspect,
            )
            if regularized is not None:
                predicted_inner = regularized
                seed_regularized = True

        short_side = _quad_short_side(predicted_inner)
        narrow_radius = int(round(short_side * 0.075))
        narrow_radius = max(10, min(narrow_radius, 32))
        wide_radius = int(round(short_side * 0.22))
        wide_radius = max(narrow_radius + 14, min(wide_radius, 56))
        inner_wide_radius = wide_radius
        if seed_regularized:
            # The 160 cm malformed seeds were displaced by up to roughly 50
            # high-resolution pixels after physical-model regularization. This
            # wider radius runs only on ROI_SEED and is still a sparse local
            # normal-band search, never a full 1920x1080 rectangle scan.
            inner_wide_radius = int(round(short_side * 0.50))
            inner_wide_radius = max(
                wide_radius,
                min(inner_wide_radius, 84),
            )

        seed_regularization_max_shift = (
            seed_regularization["max_nearest_corner_shift"]
            if seed_regularization is not None
            else 0.0
        )
        large_seed_shift_threshold = max(18.0, short_side * 0.15)
        large_seed_shift = (
            seed_regularized
            and seed_regularization_max_shift
            > large_seed_shift_threshold
        )
        if localization_only and seed_class == "ROI_SEED":
            # A projective aperture fallback supplies a reliable location but
            # only an axis-aligned bounding box.  The true tilted edges can be
            # displaced from that box by more than the normal narrow band, so
            # spend one bounded wide pass on the inner quad.  This is still a
            # sparse local search and is enabled only for TILT mode.
            inner_wide_radius = max(
                inner_wide_radius,
                min(int(round(short_side * 0.50)), 84),
            )
            large_seed_shift = True

        inner = _adaptive_refine_quad(
            image,
            predicted_inner,
            1,
            narrow_radius,
            inner_wide_radius,
            self.sample_count,
            self.min_response,
            force_wide=large_seed_shift,
            max_acceptable_rms=self.max_weak_inner_edge_rms,
        )
        inner_corners = inner["corners"]

        model_outer = _predict_outer(
            inner_corners,
            self.expected_short_ratio,
            self.expected_long_ratio,
        )
        if localization_only:
            # The 30--60 degree task needs the four inner-aperture edges and a
            # plane homography, but it explicitly does not need D.  Searching
            # a second set of outer edges is both expensive and physically
            # unnecessary under strong perspective.  Keep a complete
            # diagnostic dictionary backed by the official-dimension model so
            # all downstream drawing/metadata code remains unchanged.
            outer = {
                "corners": model_outer,
                "valid_sides": 0,
                "raw_valid_sides": 0,
                "edge_rms": -1.0,
                "raw_edge_rms": -1.0,
                "mean_response": 0.0,
                "point_count": 0,
                "geometry_valid": True,
                "search_radius": 0,
                "wide_search_used": False,
            }
            outer_corners = model_outer
            detected_outer_corners = model_outer
            inner_detected_good = inner["valid_sides"] >= 3
            outer_detected_good = False
            outer_independently_valid = False
            detected_outer_edge_rms = -1.0
            detected_frame_disagreement = 0.0
            outer_wide_search_used = False
        else:
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
                    max_acceptable_rms=self.max_outer_edge_rms,
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
                    candidate_score -= min(
                        candidate_disagreement, 1.0
                    ) * 300.0
                if outer is None or candidate_score > outer_selection_score:
                    outer = candidate_outer
                    outer_selection_score = candidate_score

            outer_corners = outer["corners"]
            detected_outer_corners = outer_corners
            inner_detected_good = inner["valid_sides"] >= 3
            outer_detected_good = outer["valid_sides"] >= 3
            outer_independently_valid = outer_detected_good
            detected_outer_edge_rms = (
                outer["edge_rms"] if outer_detected_good else -1.0
            )
            detected_frame_disagreement = _frame_disagreement(
                inner_corners,
                outer_corners,
                self.expected_short_ratio,
                self.expected_long_ratio,
            )
            outer_wide_search_used = outer.get("wide_search_used", False)

        # The model-predicted ring is derived only from the accurately refined
        # inner aperture and the official 17/21, 25.7/29.7 dimensions. At
        # 175--200 cm an unrelated outer gradient can still produce four neat
        # fitted lines. The paper exterior is arbitrary scene background, so
        # only the known white-aperture-to-black-band evidence may authorize
        # demoting a conflicting independent outer fit.
        ring_outer_corners = _predict_outer(
            inner_corners,
            self.expected_short_ratio,
            self.expected_long_ratio,
        )
        ring_metrics = _border_ring_metrics(
            image,
            inner_corners,
            ring_outer_corners,
        )
        outer_conflict_demoted = False
        conflict_inner_ring_valid = (
            ring_metrics["valid_samples"] >= self.min_conflict_ring_samples
            and ring_metrics["inside_contrast"]
            >= self.min_ring_inside_contrast
            and ring_metrics["inside_pass_ratio"]
            >= self.min_conflict_ring_inside_pass_ratio
            and ring_metrics["min_side_inside_pass_ratio"]
            >= self.min_conflict_ring_side_pass_ratio
        )
        if (
            inner["valid_sides"] == 4
            and outer_detected_good
            and detected_frame_disagreement > self.max_frame_disagreement
            and conflict_inner_ring_valid
        ):
            # Keep the detected fit in diagnostic fields, but do not let a
            # demonstrably drifted outer edge veto a strong inner aperture and
            # complete official-dimension ring.  Distance/plane scale continue
            # to use the inner aperture only.
            outer_conflict_demoted = True
            outer_detected_good = False
            outer_corners = ring_outer_corners

        mode = "HIGH_RES_FALLBACK"
        inner_good = inner_detected_good
        outer_good = outer_detected_good
        frame_disagreement = detected_frame_disagreement
        if inner_good and outer_good:
            mode = "HIGH_RES_PAIR"
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
        localization_valid = mode != "HIGH_RES_FALLBACK"
        anisotropy_limit = self.max_inner_scale_anisotropy
        if max_inner_scale_anisotropy is not None:
            # Advanced targets still use the same inner-aperture scale, but
            # their later plane/shape consistency gate is more informative
            # than the parallel-sheet anisotropy gate.  Keep the relaxation
            # explicit and local to this refinement call; the BASIC path
            # continues to use the calibrated 0.08 limit.
            anisotropy_limit = max(
                self.max_inner_scale_anisotropy,
                float(max_inner_scale_anisotropy),
            )
        inner_regularity = _quad_regularity(
            inner_corners, self.expected_inner_aspect
        )
        outer_regularity = _quad_regularity(
            outer_corners, self.expected_outer_aspect
        )
        inner_in_bounds = _quad_in_bounds(
            inner_corners, image.width(), image.height()
        )
        outer_in_bounds = _quad_in_bounds(
            outer_corners, image.width(), image.height()
        )

        measurement_reject_reason = "OK"
        measurement_confidence = "STRONG"
        measurement_confidence_rank = 2
        if localization_only:
            measurement_reject_reason = "TILT_LOCALIZATION_ONLY"
            measurement_confidence = "LOCALIZATION"
            measurement_confidence_rank = 1
        elif not localization_valid:
            measurement_reject_reason = "LOCALIZATION_INVALID"
        elif not inner_in_bounds:
            measurement_reject_reason = "INNER_BOUNDS"
        elif not outer_in_bounds:
            measurement_reject_reason = "OUTER_BOUNDS"
        elif inner_valid_sides < 4:
            measurement_reject_reason = "INNER_SIDES"
        elif inner["edge_rms"] < 0.0:
            measurement_reject_reason = "INNER_RMS_MISSING"
        elif inner["edge_rms"] > self.max_weak_inner_edge_rms:
            measurement_reject_reason = "INNER_RMS"
        elif inner_regularity["fill_ratio"] < self.min_inner_fill_ratio:
            measurement_reject_reason = "INNER_FILL"
        elif (
            inner_regularity["max_opposite_error"]
            > self.max_inner_opposite_error
        ):
            measurement_reject_reason = "INNER_OPPOSITE"
        elif (
            inner_regularity["scale_anisotropy"]
            > anisotropy_limit
        ):
            measurement_reject_reason = "INNER_ANISOTROPY"
        elif ring_metrics["valid_samples"] < 16:
            measurement_reject_reason = "RING_SAMPLES"
        elif (
            ring_metrics["inside_contrast"]
            < self.min_ring_inside_contrast
        ):
            measurement_reject_reason = "RING_CONTRAST"
        elif (
            ring_metrics["inside_pass_ratio"]
            < self.min_ring_inside_pass_ratio
        ):
            measurement_reject_reason = "RING_COVERAGE"
        elif (
            ring_metrics["min_side_inside_pass_ratio"]
            < self.min_ring_side_pass_ratio
        ):
            measurement_reject_reason = "RING_SIDE"
        elif outer_detected_good and outer["edge_rms"] < 0.0:
            measurement_reject_reason = "OUTER_RMS_MISSING"
        elif (
            outer_detected_good
            and outer["edge_rms"] > self.max_outer_edge_rms
        ):
            measurement_reject_reason = "OUTER_RMS"
        elif (
            outer_detected_good
            and detected_frame_disagreement
            > self.max_frame_disagreement
        ):
            measurement_reject_reason = "FRAME_DISAGREEMENT"
        elif not outer_detected_good and mode != "HIGH_RES_INNER":
            measurement_reject_reason = "OUTER_SIDES"
        elif inner["edge_rms"] > self.max_inner_edge_rms:
            measurement_reject_reason = "WEAK_INNER_RMS"
            measurement_confidence = "WEAK"
            measurement_confidence_rank = 1

        if measurement_reject_reason not in (
            "OK",
            "WEAK_INNER_RMS",
            "TILT_LOCALIZATION_ONLY",
        ):
            measurement_confidence = "REJECT"
            measurement_confidence_rank = 0
        measurement_valid = measurement_reject_reason == "OK"
        inner_only_accepted = measurement_valid and mode == "HIGH_RES_INNER"
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
            - frame_disagreement * 100.0
            - max(inner["edge_rms"], 0.0) * 2.0
            - max(outer["edge_rms"], 0.0) * 0.5
            - max(0.0, 1.0 - ring_metrics["inside_pass_ratio"]) * 10.0
        )
        selected_radius = max(
            inner["search_radius"], outer["search_radius"]
        )
        relocalize_roi = None
        if seed_regularized:
            relocalize_roi = _expanded_quad_roi(
                _predict_outer(
                    predicted_inner,
                    self.expected_short_ratio,
                    self.expected_long_ratio,
                ),
                image.width(),
                image.height(),
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
            "inner_raw_edge_rms": inner.get("raw_edge_rms", -1.0),
            "inner_refine_path": inner.get("refine_path", "NARROW"),
            "inner_wide_search_reason": inner.get(
                "wide_search_reason", "NOT_NEEDED"
            ),
            "inner_narrow_edge_rms": inner.get(
                "narrow_edge_rms", -1.0
            ),
            "inner_wide_edge_rms": inner.get("wide_edge_rms", -1.0),
            "inner_narrow_valid_sides": inner.get(
                "narrow_valid_sides", 0
            ),
            "inner_wide_valid_sides": inner.get("wide_valid_sides", 0),
            "outer_edge_rms": (
                outer["edge_rms"] if outer_valid_sides > 0 else -1.0
            ),
            "inner_point_count": inner["point_count"],
            "outer_point_count": outer["point_count"],
            "inner_geometry_valid": inner["geometry_valid"],
            "outer_geometry_valid": outer["geometry_valid"],
            "inner_raw_valid_sides": inner["raw_valid_sides"],
            "outer_raw_valid_sides": outer["raw_valid_sides"],
            "outer_independently_valid": outer_independently_valid,
            # Backward-compatible field name; it now truthfully means that the
            # adaptive outer refiner executed its wide pass, not that an extra
            # conflict-only recovery was selected.
            "outer_wide_recovery_used": outer_wide_search_used,
            "outer_wide_search_used": outer_wide_search_used,
            "inner_wide_search_used": inner.get(
                "wide_search_used", False
            ),
            "outer_conflict_demoted": outer_conflict_demoted,
            "outer_conflict_inner_ring_valid": conflict_inner_ring_valid,
            "detected_outer_edge_rms": detected_outer_edge_rms,
            "detected_outer_corners_float": tuple(
                (float(x), float(y)) for x, y in detected_outer_corners
            ),
            "detected_outer_corners": tuple(
                (int(round(x)), int(round(y)))
                for x, y in detected_outer_corners
            ),
            "model_outer_corners": tuple(
                (int(round(x)), int(round(y)))
                for x, y in ring_outer_corners
            ),
            "inner_only_accepted": inner_only_accepted,
            "inner_search_radius": inner["search_radius"],
            "outer_search_radius": outer["search_radius"],
            "search_radius": selected_radius,
            "coarse_seed_class": seed_class,
            "seed_regularized": seed_regularized,
            "seed_original_inner_corners": tuple(
                (int(round(x)), int(round(y)))
                for x, y in original_predicted_inner
            ),
            "seed_regularized_inner_corners": (
                tuple(
                    (int(round(x)), int(round(y)))
                    for x, y in predicted_inner
                )
                if seed_regularized
                else None
            ),
            "seed_regularization_angle_degrees": (
                seed_regularization["angle_degrees"]
                if seed_regularization is not None
                else 0.0
            ),
            "seed_regularization_max_shift": (
                seed_regularization_max_shift
            ),
            "large_seed_shift_threshold": large_seed_shift_threshold,
            "large_seed_shift": large_seed_shift,
            "relocalize_roi": relocalize_roi,
            "frame_model_disagreement": frame_disagreement,
            "detected_frame_disagreement": detected_frame_disagreement,
            "inner_fill_ratio": inner_regularity["fill_ratio"],
            "outer_fill_ratio": outer_regularity["fill_ratio"],
            "inner_max_opposite_error": inner_regularity[
                "max_opposite_error"
            ],
            "outer_max_opposite_error": outer_regularity[
                "max_opposite_error"
            ],
            "inner_scale_anisotropy": inner_regularity[
                "scale_anisotropy"
            ],
            "inner_scale_anisotropy_limit": anisotropy_limit,
            "outer_scale_anisotropy": outer_regularity[
                "scale_anisotropy"
            ],
            "inner_in_bounds": inner_in_bounds,
            "outer_in_bounds": outer_in_bounds,
            "ring_valid_samples": ring_metrics["valid_samples"],
            "ring_inside_contrast": ring_metrics["inside_contrast"],
            "ring_outside_contrast": ring_metrics["outside_contrast"],
            "ring_inside_pass_ratio": ring_metrics[
                "inside_pass_ratio"
            ],
            "ring_outside_pass_ratio": ring_metrics[
                "outside_pass_ratio"
            ],
            "ring_min_side_pass_ratio": ring_metrics[
                "min_side_inside_pass_ratio"
            ],
            "ring_mean_thickness": ring_metrics["mean_thickness"],
            "quality_score": quality_score,
            "localization_valid": localization_valid,
            "measurement_valid": measurement_valid,
            "measurement_confidence": measurement_confidence,
            "measurement_confidence_rank": measurement_confidence_rank,
            "measurement_reject_reason": measurement_reject_reason,
            "localization_only": bool(localization_only),
            "scale_x": scale_x,
            "scale_y": scale_y,
        }
