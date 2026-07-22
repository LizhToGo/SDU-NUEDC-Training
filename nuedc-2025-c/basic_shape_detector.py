"""Detect and measure the basic black target inside the refined A4 frame.

The detector deliberately does not run a neural network.  ``find_blobs`` is
used only to obtain a small set of black connected-component hypotheses.  A
candidate is then analysed in the 17.0 cm x 25.7 cm object-plane coordinate
system supplied by :mod:`plane_mapper`:

* white and black samples produce a per-shot adaptive grayscale threshold;
* radial rays trace the solid target boundary after perspective correction;
* rotation-invariant third/fourth radial harmonics distinguish an equilateral
  triangle and a square, while a nearly constant radius identifies a circle;
* polar area gives the circle diameter or polygon side length.

Only basic circle/triangle/square targets are handled here.  Combination and
digit targets will use separate modules later and do not complicate this
stable basic-target path.
"""

import math
import time


DEFAULT_BLACK_THRESHOLD = (0, 50, -128, 127, -128, 127)


def _clamp(value, low=0.0, high=1.0):
    if value < low:
        return low
    if value > high:
        return high
    return value


def _distance(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return math.sqrt(dx * dx + dy * dy)


def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return float(ordered[middle])
    return (ordered[middle - 1] + ordered[middle]) * 0.5


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


def _gray_from_pixel(pixel):
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


def _sample_plane_patch(image, mapper, u_cm, v_cm):
    x, y = mapper.plane_to_image(u_cm, v_cm)
    values = []
    for offset_y in (-1, 0, 1):
        for offset_x in (-1, 0, 1):
            value = _sample_gray(image, x + offset_x, y + offset_y)
            if value is not None:
                values.append(value)
    return _median(values)


def _blob_call(blob, name, fallback_index=None, default=None):
    try:
        member = getattr(blob, name)
        if callable(member):
            return member()
        return member
    except Exception:
        if fallback_index is not None:
            try:
                return blob[fallback_index]
            except Exception:
                pass
    return default


def _blob_rect(blob):
    result = _blob_call(blob, "rect", default=None)
    if result is None:
        try:
            result = blob[0:4]
        except Exception:
            return None
    if result is None or len(result) < 4:
        return None
    return (
        int(result[0]),
        int(result[1]),
        int(result[2]),
        int(result[3]),
    )


def _image_roi(image, corners):
    minimum_x = corners[0][0]
    maximum_x = corners[0][0]
    minimum_y = corners[0][1]
    maximum_y = corners[0][1]
    for x, y in corners[1:]:
        minimum_x = min(minimum_x, x)
        maximum_x = max(maximum_x, x)
        minimum_y = min(minimum_y, y)
        maximum_y = max(maximum_y, y)

    x = max(0, int(math.floor(minimum_x)) - 1)
    y = max(0, int(math.floor(minimum_y)) - 1)
    right = min(image.width() - 1, int(math.ceil(maximum_x)) + 1)
    bottom = min(image.height() - 1, int(math.ceil(maximum_y)) + 1)
    return (x, y, max(1, right - x + 1), max(1, bottom - y + 1))


def _ray_plane_limit(mapper, center, direction, margin_cm=0.04):
    u_cm, v_cm = center
    direction_u, direction_v = direction
    limits = []
    if direction_u > 1.0e-9:
        limits.append(
            (mapper.plane_width_cm - margin_cm - u_cm) / direction_u
        )
    elif direction_u < -1.0e-9:
        limits.append((margin_cm - u_cm) / direction_u)
    if direction_v > 1.0e-9:
        limits.append(
            (mapper.plane_height_cm - margin_cm - v_cm) / direction_v
        )
    elif direction_v < -1.0e-9:
        limits.append((margin_cm - v_cm) / direction_v)

    positive = []
    for value in limits:
        if value > 0.0:
            positive.append(value)
    if not positive:
        return 0.0
    return min(positive)


def _fill_missing_radii(radii):
    valid_count = 0
    for value in radii:
        if value is not None:
            valid_count += 1
    if valid_count < int(len(radii) * 0.85):
        return None, valid_count

    filled = list(radii)
    count = len(filled)
    for index in range(count):
        if filled[index] is not None:
            continue
        previous = None
        following = None
        for offset in range(1, count):
            value = filled[(index - offset) % count]
            if value is not None:
                previous = value
                break
        for offset in range(1, count):
            value = filled[(index + offset) % count]
            if value is not None:
                following = value
                break
        if previous is None or following is None:
            return None, valid_count
        filled[index] = (previous + following) * 0.5
    return tuple(filled), valid_count


def _radial_statistics(radii):
    count = len(radii)
    mean_radius = sum(radii) / count
    variance = 0.0
    for radius in radii:
        delta = radius - mean_radius
        variance += delta * delta
    radial_cv = math.sqrt(variance / count) / max(mean_radius, 1.0e-9)

    harmonics = {}
    phases = {}
    for order in (1, 3, 4):
        cosine = 0.0
        sine = 0.0
        for index in range(count):
            angle = 2.0 * math.pi * index / count
            cosine += radii[index] * math.cos(order * angle)
            sine += radii[index] * math.sin(order * angle)
        cosine /= count
        sine /= count
        harmonics[order] = (
            2.0 * math.sqrt(cosine * cosine + sine * sine)
            / max(mean_radius, 1.0e-9)
        )
        phases[order] = math.atan2(sine, cosine)

    polar_area = 0.5 * (2.0 * math.pi / count) * sum(
        radius * radius for radius in radii
    )
    return {
        "mean_radius": mean_radius,
        "radial_cv": radial_cv,
        "harmonic_1": harmonics[1],
        "harmonic_3": harmonics[3],
        "harmonic_4": harmonics[4],
        "phase_3": phases[3],
        "phase_4": phases[4],
        "area_cm2": polar_area,
    }


def _classification(radial_cv, harmonic_3, harmonic_4):
    # Ideal signatures sampled from the exact radial functions.  Magnitudes
    # are rotation invariant, which avoids assuming an upright printed shape.
    models = (
        ("CIRCLE", 0.000, 0.000, 0.000),
        ("SQUARE", 0.105, 0.000, 0.140),
        ("TRIANGLE", 0.214, 0.275, 0.000),
    )
    scored = []
    for name, model_cv, model_h3, model_h4 in models:
        error = (
            ((radial_cv - model_cv) / 0.105) ** 2
            + ((harmonic_3 - model_h3) / 0.180) ** 2
            + ((harmonic_4 - model_h4) / 0.120) ** 2
        )
        scored.append((error, name))
    scored.sort(key=lambda item: item[0])
    return scored[0][1], scored[0][0], scored[1][0]


def _extract_vertices(radii, center, mapper, vertex_count):
    count = len(radii)
    ranked = list(range(count))
    ranked.sort(key=lambda index: radii[index], reverse=True)
    selected = []
    minimum_separation = max(2, int(count / (vertex_count * 2.2)))
    for index in ranked:
        separated = True
        for existing in selected:
            difference = abs(index - existing)
            difference = min(difference, count - difference)
            if difference < minimum_separation:
                separated = False
                break
        if separated:
            selected.append(index)
            if len(selected) >= vertex_count:
                break
    if len(selected) != vertex_count:
        return None, None
    selected.sort()

    plane_vertices = []
    image_vertices = []
    for index in selected:
        angle = 2.0 * math.pi * index / count
        point = (
            center[0] + radii[index] * math.cos(angle),
            center[1] + radii[index] * math.sin(angle),
        )
        plane_vertices.append(point)
        image_point = mapper.plane_to_image(point[0], point[1])
        image_vertices.append((
            int(round(image_point[0])),
            int(round(image_point[1])),
        ))
    return tuple(plane_vertices), tuple(image_vertices)


def _fit_plane_line(points):
    """Fit ``a*u + b*v + c = 0`` by total least squares."""
    if points is None or len(points) < 4:
        return None
    center_u = sum(point[0] for point in points) / len(points)
    center_v = sum(point[1] for point in points) / len(points)
    covariance_uu = 0.0
    covariance_uv = 0.0
    covariance_vv = 0.0
    for u_cm, v_cm in points:
        delta_u = u_cm - center_u
        delta_v = v_cm - center_v
        covariance_uu += delta_u * delta_u
        covariance_uv += delta_u * delta_v
        covariance_vv += delta_v * delta_v

    direction_angle = 0.5 * math.atan2(
        2.0 * covariance_uv,
        covariance_uu - covariance_vv,
    )
    direction_u = math.cos(direction_angle)
    direction_v = math.sin(direction_angle)
    normal_u = -direction_v
    normal_v = direction_u
    offset = -(normal_u * center_u + normal_v * center_v)

    error_sum = 0.0
    for u_cm, v_cm in points:
        error = normal_u * u_cm + normal_v * v_cm + offset
        error_sum += error * error
    rms = math.sqrt(error_sum / len(points))
    return (normal_u, normal_v, offset, rms, len(points))


def _line_intersection(first, second):
    determinant = first[0] * second[1] - second[0] * first[1]
    if abs(determinant) < 1.0e-8:
        return None
    u_cm = (first[1] * second[2] - second[1] * first[2]) / determinant
    v_cm = (first[2] * second[0] - second[2] * first[0]) / determinant
    return (u_cm, v_cm)


def _polygon_area(points):
    area = 0.0
    for index in range(len(points)):
        first = points[index]
        second = points[(index + 1) % len(points)]
        area += first[0] * second[1] - first[1] * second[0]
    return abs(area) * 0.5


def _refine_regular_polygon(
    plane_outline,
    mapper,
    vertex_count,
    harmonic_phase,
):
    """Fit polygon sides to radial boundary arcs and intersect adjacent lines.

    The harmonic phase predicts the vertex directions.  Boundary samples
    between two consecutive vertex directions belong to one straight side;
    samples near a corner are excluded so a missed exact corner ray cannot
    chamfer the fitted polygon.
    """
    if len(plane_outline) < vertex_count * 6:
        return None
    full_turn = 2.0 * math.pi
    sector = full_turn / vertex_count
    base_angle = harmonic_phase / vertex_count
    while base_angle < 0.0:
        base_angle += sector
    while base_angle >= sector:
        base_angle -= sector

    groups = []
    for _ in range(vertex_count):
        groups.append([])
    exclusion = 0.14
    for index, point in enumerate(plane_outline):
        angle = full_turn * index / len(plane_outline)
        relative = angle - base_angle
        while relative < 0.0:
            relative += full_turn
        while relative >= full_turn:
            relative -= full_turn
        edge_index = int(relative / sector)
        if edge_index >= vertex_count:
            edge_index = vertex_count - 1
        position = (relative - edge_index * sector) / sector
        if position >= exclusion and position <= 1.0 - exclusion:
            groups[edge_index].append(point)

    lines = []
    for group in groups:
        line = _fit_plane_line(group)
        if line is None:
            return None
        lines.append(line)

    plane_corners = []
    for index in range(vertex_count):
        corner = _line_intersection(
            lines[(index - 1) % vertex_count],
            lines[index],
        )
        if corner is None:
            return None
        if not mapper.contains_plane_point(
            corner[0], corner[1], margin_cm=-0.6
        ):
            return None
        plane_corners.append(corner)

    side_lengths = []
    for index in range(vertex_count):
        side_lengths.append(_distance(
            plane_corners[index],
            plane_corners[(index + 1) % vertex_count],
        ))
    mean_side = sum(side_lengths) / vertex_count
    variance = 0.0
    for side in side_lengths:
        variance += (side - mean_side) * (side - mean_side)
    side_cv = math.sqrt(variance / vertex_count) / max(mean_side, 1.0e-9)

    image_corners = []
    for u_cm, v_cm in plane_corners:
        x, y = mapper.plane_to_image(u_cm, v_cm)
        image_corners.append((int(round(x)), int(round(y))))

    line_rms_values = tuple(line[3] for line in lines)
    return {
        "plane_corners": tuple(plane_corners),
        "image_corners": tuple(image_corners),
        "side_lengths_cm": tuple(side_lengths),
        "mean_side_cm": mean_side,
        "side_cv": side_cv,
        "line_rms_cm": line_rms_values,
        "max_line_rms_cm": max(line_rms_values),
        "polygon_area_cm2": _polygon_area(plane_corners),
        "edge_point_counts": tuple(line[4] for line in lines),
    }


def scale_shape_result(shape_result, scale_x, scale_y):
    """Return the image-coordinate subset required for preview drawing."""
    if shape_result is None:
        return None
    outline = []
    for x, y in shape_result.get("image_outline", ()):
        outline.append((
            int(round(x * scale_x)),
            int(round(y * scale_y)),
        ))
    center = shape_result.get("image_center")
    if center is not None:
        center = (
            int(round(center[0] * scale_x)),
            int(round(center[1] * scale_y)),
        )
    corners = shape_result.get("image_corners")
    scaled_corners = None
    if corners is not None:
        scaled_corners = []
        for x, y in corners:
            scaled_corners.append((
                int(round(x * scale_x)),
                int(round(y * scale_y)),
            ))
        scaled_corners = tuple(scaled_corners)
    return {
        "shape_valid": shape_result.get("shape_valid", False),
        "shape_type": shape_result.get("shape_type", "UNKNOWN"),
        "x_cm": shape_result.get("x_cm", 0.0),
        "confidence": shape_result.get("confidence", 0.0),
        "image_outline": tuple(outline),
        "image_center": center,
        "image_corners": scaled_corners,
    }


def draw_shape_overlay(image, shape_result, color=(255, 255, 0), thickness=2):
    """Draw only geometry; no FreeType/draw_string dependency is used."""
    if shape_result is None or not shape_result.get("shape_valid", False):
        return
    shape_type = shape_result.get("shape_type", "UNKNOWN")
    corners = shape_result.get("image_corners")
    expected_corner_count = 0
    if shape_type == "SQUARE":
        expected_corner_count = 4
    elif shape_type == "TRIANGLE":
        expected_corner_count = 3

    drew_polygon = False
    if (
        expected_corner_count > 0
        and corners is not None
        and len(corners) == expected_corner_count
    ):
        for index in range(expected_corner_count):
            first = corners[index]
            second = corners[(index + 1) % expected_corner_count]
            image.draw_line(
                int(first[0]),
                int(first[1]),
                int(second[0]),
                int(second[1]),
                color=color,
                thickness=thickness,
            )
            image.draw_circle(
                int(first[0]),
                int(first[1]),
                3,
                color=color,
                thickness=1,
            )
        drew_polygon = True

    outline = shape_result.get("image_outline", ())
    if not drew_polygon and len(outline) >= 3:
        # Drawing every second point is visually clean and halves display work.
        stride = 2 if len(outline) >= 48 else 1
        # CanMV MicroPython does not implement sequence slices with a step
        # (``outline[::2]``), so walk the original tuple by index instead.
        reduced_count = (len(outline) + stride - 1) // stride
        for index in range(reduced_count):
            first_index = index * stride
            if index + 1 >= reduced_count:
                second_index = 0
            else:
                second_index = (index + 1) * stride
            first = outline[first_index]
            second = outline[second_index]
            image.draw_line(
                int(first[0]),
                int(first[1]),
                int(second[0]),
                int(second[1]),
                color=color,
                thickness=thickness,
            )
    center = shape_result.get("image_center")
    if center is not None:
        image.draw_cross(
            int(center[0]),
            int(center[1]),
            color=color,
            size=5,
            thickness=1,
        )


class BasicShapeDetector:
    """Find one central solid circle, equilateral triangle, or square."""

    def __init__(
        self,
        black_threshold=DEFAULT_BLACK_THRESHOLD,
        angular_samples=72,
        min_contrast=25.0,
        minimum_x_cm=8.5,
        maximum_x_cm=17.5,
    ):
        self.black_threshold = black_threshold
        self.angular_samples = max(36, int(angular_samples))
        self.min_contrast = float(min_contrast)
        self.minimum_x_cm = float(minimum_x_cm)
        self.maximum_x_cm = float(maximum_x_cm)

    def _failure(self, reason, started_ms, blob_count=0, candidate_count=0):
        return {
            "shape_valid": False,
            "shape_type": "UNKNOWN",
            "x_cm": 0.0,
            "confidence": 0.0,
            "reject_reason": reason,
            "blob_count": blob_count,
            "candidate_count": candidate_count,
            "shape_ms": _ticks_diff(_ticks_ms(), started_ms),
        }

    def _find_blobs(self, image, roi):
        try:
            return image.find_blobs(
                [self.black_threshold],
                roi=roi,
                pixels_threshold=60,
                area_threshold=80,
                merge=False,
            )
        except TypeError:
            # Retain compatibility with older CanMV keyword sets.
            return image.find_blobs(
                [self.black_threshold],
                roi=roi,
                pixels_threshold=60,
                area_threshold=80,
            )

    def _candidate(self, blob, mapper):
        rect = _blob_rect(blob)
        if rect is None or rect[2] < 4 or rect[3] < 4:
            return None
        centre_x = _blob_call(blob, "cx", 5, rect[0] + rect[2] * 0.5)
        centre_y = _blob_call(blob, "cy", 6, rect[1] + rect[3] * 0.5)
        pixels = _blob_call(blob, "pixels", 4, rect[2] * rect[3])
        centre_x = float(centre_x)
        centre_y = float(centre_y)
        pixels = max(1, int(pixels))

        plane_center = mapper.image_to_plane(centre_x, centre_y)
        delta_u = abs(plane_center[0] - mapper.plane_width_cm * 0.5)
        delta_v = abs(plane_center[1] - mapper.plane_height_cm * 0.5)
        if delta_u > 4.5 or delta_v > 4.5:
            return None

        bounds = mapper.image_rect_plane_bounds(rect)
        width_cm = bounds[2] - bounds[0]
        height_cm = bounds[3] - bounds[1]
        smaller = min(width_cm, height_cm)
        larger = max(width_cm, height_cm)
        if smaller < 6.0 or larger > 20.0:
            return None

        area_cm2 = mapper.image_area_to_plane(
            pixels, centre_x, centre_y
        )
        if area_cm2 < 20.0 or area_cm2 > 330.0:
            return None

        centre_error = math.sqrt(
            (delta_u / 4.5) ** 2 + (delta_v / 4.5) ** 2
        )
        score = (
            2.0 * _clamp(1.0 - centre_error)
            + _clamp((smaller - 6.0) / 5.0)
            + _clamp(pixels / 1500.0)
        )
        return {
            "blob": blob,
            "rect": rect,
            "pixels": pixels,
            "image_center": (centre_x, centre_y),
            "plane_center": plane_center,
            "blob_width_cm": width_cm,
            "blob_height_cm": height_cm,
            "blob_area_cm2": area_cm2,
            "candidate_score": score,
        }

    def _adaptive_threshold(self, image, mapper, plane_center):
        black_values = []
        for offset_u, offset_v in (
            (0.0, 0.0),
            (-0.15, 0.0),
            (0.15, 0.0),
            (0.0, -0.15),
            (0.0, 0.15),
        ):
            value = _sample_plane_patch(
                image,
                mapper,
                plane_center[0] + offset_u,
                plane_center[1] + offset_v,
            )
            if value is not None:
                black_values.append(value)

        white_values = []
        for v_cm in (1.3, mapper.plane_height_cm - 1.3):
            for fraction in (0.25, 0.50, 0.75):
                value = _sample_plane_patch(
                    image,
                    mapper,
                    mapper.plane_width_cm * fraction,
                    v_cm,
                )
                if value is not None:
                    white_values.append(value)

        black_level = _median(black_values)
        white_level = _median(white_values)
        if black_level is None or white_level is None:
            return None
        contrast = white_level - black_level
        return {
            "black_level": black_level,
            "white_level": white_level,
            "contrast": contrast,
            "threshold": black_level + contrast * 0.48,
        }

    def _trace_radii(self, image, mapper, plane_center, threshold):
        pixels_per_cm = mapper.local_pixels_per_cm(
            plane_center[0], plane_center[1]
        )
        radial_step = 0.12
        if pixels_per_cm > 0.0:
            radial_step = _clamp(0.78 / pixels_per_cm, 0.075, 0.14)

        radii = []
        for index in range(self.angular_samples):
            angle = 2.0 * math.pi * index / self.angular_samples
            direction = (math.cos(angle), math.sin(angle))
            limit = min(
                12.5,
                _ray_plane_limit(mapper, plane_center, direction),
            )
            if limit <= radial_step * 3.0:
                radii.append(None)
                continue

            centre_image = mapper.plane_to_image(
                plane_center[0], plane_center[1]
            )
            centre_gray = _sample_gray(
                image, centre_image[0], centre_image[1]
            )
            if centre_gray is None or centre_gray > threshold:
                radii.append(None)
                continue

            last_dark_radius = 0.0
            last_dark_gray = centre_gray
            first_white_radius = None
            first_white_gray = None
            white_run = 0
            boundary = None
            radius = radial_step
            while radius <= limit:
                u_cm = plane_center[0] + direction[0] * radius
                v_cm = plane_center[1] + direction[1] * radius
                image_point = mapper.plane_to_image(u_cm, v_cm)
                gray = _sample_gray(image, image_point[0], image_point[1])
                if gray is None:
                    break
                if gray <= threshold:
                    last_dark_radius = radius
                    last_dark_gray = gray
                    first_white_radius = None
                    first_white_gray = None
                    white_run = 0
                else:
                    if white_run == 0:
                        first_white_radius = radius
                        first_white_gray = gray
                    white_run += 1
                    if white_run >= 2:
                        boundary = (last_dark_radius + first_white_radius) * 0.5
                        difference = first_white_gray - last_dark_gray
                        if difference > 1.0e-6:
                            fraction = (
                                (threshold - last_dark_gray) / difference
                            )
                            fraction = _clamp(fraction)
                            boundary = last_dark_radius + fraction * (
                                first_white_radius - last_dark_radius
                            )
                        break
                radius += radial_step
            radii.append(boundary)

        filled, valid_count = _fill_missing_radii(radii)
        return filled, valid_count, radial_step, pixels_per_cm

    def _analyse_candidate(self, image, mapper, candidate):
        threshold_info = self._adaptive_threshold(
            image, mapper, candidate["plane_center"]
        )
        if threshold_info is None:
            return None, "PIXEL_READ_FAILED"
        if threshold_info["contrast"] < self.min_contrast:
            return None, "LOW_BLACK_WHITE_CONTRAST"

        radii, valid_rays, radial_step, pixels_per_cm = self._trace_radii(
            image,
            mapper,
            candidate["plane_center"],
            threshold_info["threshold"],
        )
        if radii is None:
            return None, "INSUFFICIENT_RADIAL_BOUNDARY"

        statistics = _radial_statistics(radii)
        shape_type, model_error, second_error = _classification(
            statistics["radial_cv"],
            statistics["harmonic_3"],
            statistics["harmonic_4"],
        )

        if shape_type == "CIRCLE":
            area_x_cm = math.sqrt(
                4.0 * statistics["area_cm2"] / math.pi
            )
            vertex_count = 0
            harmonic_phase = 0.0
        elif shape_type == "SQUARE":
            area_x_cm = math.sqrt(statistics["area_cm2"])
            vertex_count = 4
            harmonic_phase = statistics["phase_4"]
        else:
            area_x_cm = math.sqrt(
                4.0 * statistics["area_cm2"] / math.sqrt(3.0)
            )
            vertex_count = 3
            harmonic_phase = statistics["phase_3"]
        x_cm = area_x_cm

        plane_outline = []
        image_outline = []
        minimum_u = None
        maximum_u = None
        minimum_v = None
        maximum_v = None
        for index in range(len(radii)):
            angle = 2.0 * math.pi * index / len(radii)
            plane_point = (
                candidate["plane_center"][0] + radii[index] * math.cos(angle),
                candidate["plane_center"][1] + radii[index] * math.sin(angle),
            )
            plane_outline.append(plane_point)
            image_point = mapper.plane_to_image(
                plane_point[0], plane_point[1]
            )
            image_outline.append((
                int(round(image_point[0])),
                int(round(image_point[1])),
            ))
            if minimum_u is None:
                minimum_u = maximum_u = plane_point[0]
                minimum_v = maximum_v = plane_point[1]
            else:
                minimum_u = min(minimum_u, plane_point[0])
                maximum_u = max(maximum_u, plane_point[0])
                minimum_v = min(minimum_v, plane_point[1])
                maximum_v = max(maximum_v, plane_point[1])

        width_cm = maximum_u - minimum_u
        height_cm = maximum_v - minimum_v
        fill_ratio = statistics["area_cm2"] / max(
            width_cm * height_cm, 1.0e-9
        )

        plane_corners = None
        image_corners = None
        corner_refinement = None
        corner_geometry_valid = shape_type == "CIRCLE"
        corner_area_disagreement = 0.0
        if vertex_count > 0:
            corner_refinement = _refine_regular_polygon(
                plane_outline,
                mapper,
                vertex_count,
                harmonic_phase,
            )
            if corner_refinement is not None:
                plane_corners = corner_refinement["plane_corners"]
                image_corners = corner_refinement["image_corners"]
                corner_x_cm = corner_refinement["mean_side_cm"]
                corner_area_disagreement = abs(
                    corner_x_cm - area_x_cm
                ) / max((corner_x_cm + area_x_cm) * 0.5, 1.0e-9)
                corner_geometry_valid = (
                    corner_refinement["side_cv"] <= 0.08
                    and corner_refinement["max_line_rms_cm"] <= 0.20
                    and corner_area_disagreement <= 0.08
                )
                if corner_geometry_valid:
                    # Straight-side intersections carry the actual vertices;
                    # retain a small area contribution to reduce pixel noise.
                    x_cm = 0.75 * corner_x_cm + 0.25 * area_x_cm
            else:
                # Keep a diagnostic fallback overlay, but the quality gate
                # below will request another frame instead of trusting it.
                plane_corners, image_corners = _extract_vertices(
                    radii,
                    candidate["plane_center"],
                    mapper,
                    vertex_count,
                )

        model_quality = 1.0 / (1.0 + 0.5 * model_error)
        margin_quality = _clamp((second_error - model_error) / 1.6)
        contrast_quality = _clamp(
            (threshold_info["contrast"] - self.min_contrast) / 70.0
        )
        centre_quality = _clamp(1.0 - statistics["harmonic_1"] / 0.16)
        confidence = (
            0.45 * model_quality
            + 0.25 * margin_quality
            + 0.15 * contrast_quality
            + 0.15 * centre_quality
        )

        reject_reason = "OK"
        shape_valid = True
        if x_cm < self.minimum_x_cm or x_cm > self.maximum_x_cm:
            shape_valid = False
            reject_reason = "SIZE_OUT_OF_RANGE"
        elif model_error > 4.0:
            shape_valid = False
            reject_reason = "SHAPE_SIGNATURE_MISMATCH"
        elif second_error - model_error < 0.20:
            shape_valid = False
            reject_reason = "AMBIGUOUS_SHAPE"
        elif confidence < 0.55:
            shape_valid = False
            reject_reason = "LOW_SHAPE_CONFIDENCE"
        elif vertex_count > 0 and not corner_geometry_valid:
            shape_valid = False
            reject_reason = "POLYGON_CORNERS_INVALID"

        result = {
            "shape_valid": shape_valid,
            "shape_type": shape_type,
            "x_cm": x_cm,
            "confidence": confidence,
            "reject_reason": reject_reason,
            "image_center": (
                int(round(candidate["image_center"][0])),
                int(round(candidate["image_center"][1])),
            ),
            "plane_center": candidate["plane_center"],
            "image_outline": tuple(image_outline),
            "plane_outline": tuple(plane_outline),
            "image_corners": image_corners,
            "plane_corners": plane_corners,
            "width_cm": width_cm,
            "height_cm": height_cm,
            "area_cm2": statistics["area_cm2"],
            "area_x_cm": area_x_cm,
            "fill_ratio": fill_ratio,
            "radial_cv": statistics["radial_cv"],
            "harmonic_1": statistics["harmonic_1"],
            "harmonic_3": statistics["harmonic_3"],
            "harmonic_4": statistics["harmonic_4"],
            "model_error": model_error,
            "model_margin": second_error - model_error,
            "black_level": threshold_info["black_level"],
            "white_level": threshold_info["white_level"],
            "gray_threshold": threshold_info["threshold"],
            "contrast": threshold_info["contrast"],
            "valid_rays": valid_rays,
            "ray_count": len(radii),
            "radial_step_cm": radial_step,
            "pixels_per_cm": pixels_per_cm,
            "blob_rect": candidate["rect"],
            "blob_pixels": candidate["pixels"],
            "blob_area_cm2": candidate["blob_area_cm2"],
            "corner_refined": corner_refinement is not None,
            "corner_geometry_valid": corner_geometry_valid,
            "corner_area_disagreement": corner_area_disagreement,
            "corner_x_cm": (
                corner_refinement["mean_side_cm"]
                if corner_refinement is not None
                else 0.0
            ),
            "corner_side_cv": (
                corner_refinement["side_cv"]
                if corner_refinement is not None
                else 1.0
            ),
            "corner_max_line_rms_cm": (
                corner_refinement["max_line_rms_cm"]
                if corner_refinement is not None
                else -1.0
            ),
            "corner_side_lengths_cm": (
                corner_refinement["side_lengths_cm"]
                if corner_refinement is not None
                else ()
            ),
            "corner_edge_point_counts": (
                corner_refinement["edge_point_counts"]
                if corner_refinement is not None
                else ()
            ),
        }
        return result, reject_reason

    def detect(self, image, mapper, inner_corners):
        started_ms = _ticks_ms()
        roi = _image_roi(image, inner_corners)
        try:
            blobs = self._find_blobs(image, roi)
        except Exception as error:
            return self._failure(
                "FIND_BLOBS_ERROR:%s" % repr(error), started_ms
            )
        if blobs is None:
            blobs = ()

        candidates = []
        for blob in blobs:
            try:
                candidate = self._candidate(blob, mapper)
            except Exception:
                candidate = None
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(
            key=lambda item: item["candidate_score"], reverse=True
        )

        if not candidates:
            return self._failure(
                "NO_CENTRAL_BLOB", started_ms, len(blobs), 0
            )

        best_result = None
        best_score = None
        last_reason = "NO_VALID_SHAPE"
        for candidate in candidates[:3]:
            result, reason = self._analyse_candidate(image, mapper, candidate)
            last_reason = reason
            if result is None:
                continue
            score = result["confidence"] + (
                0.5 if result["shape_valid"] else 0.0
            )
            if best_score is None or score > best_score:
                best_score = score
                best_result = result

        if best_result is None:
            return self._failure(
                last_reason,
                started_ms,
                len(blobs),
                len(candidates),
            )

        best_result["blob_count"] = len(blobs)
        best_result["candidate_count"] = len(candidates)
        best_result["shape_ms"] = _ticks_diff(_ticks_ms(), started_ms)
        return best_result
