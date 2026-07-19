"""Distance estimation from the known A4 reference-frame dimensions.

The official target is a 21.0 cm x 29.7 cm A4 sheet with a 2.0 cm black
border, leaving a 17.0 cm x 25.7 cm inner white rectangle.

For final accuracy, fill DISTANCE_CALIBRATION_POINTS with measurements from
fixed positions between 100 cm and 200 cm.  Each point is:

    (known_distance_cm, frame_scale_px_per_cm)

The single-shot program prints frame_scale_px_per_cm after every press.
"""

import math


OUTER_WIDTH_CM = 21.0
OUTER_HEIGHT_CM = 29.7
INNER_WIDTH_CM = 17.0
INNER_HEIGHT_CM = 25.7

# Preliminary focal length inferred from the user's roughly 100/150/200 cm
# GC2093 photographs.  Replace it by a fitted table for competition accuracy.
PROVISIONAL_FOCAL_LENGTH_PX = 1155.0

# Example after calibration:
# DISTANCE_CALIBRATION_POINTS = (
#     (100.0, 11.52),
#     (110.0, 10.47),
#     ...
#     (200.0, 5.77),
# )
DISTANCE_CALIBRATION_POINTS = ()


def _distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def quad_pixel_dimensions(corners):
    if corners is None or len(corners) != 4:
        return None
    top = _distance(corners[0], corners[1])
    right = _distance(corners[1], corners[2])
    bottom = _distance(corners[2], corners[3])
    left = _distance(corners[3], corners[0])
    return {
        "width_px": (top + bottom) * 0.5,
        "height_px": (left + right) * 0.5,
        "top_px": top,
        "right_px": right,
        "bottom_px": bottom,
        "left_px": left,
    }


def frame_scale(corners, physical_width_cm, physical_height_cm):
    dimensions = quad_pixel_dimensions(corners)
    if dimensions is None:
        return None
    scale_x = dimensions["width_px"] / physical_width_cm
    scale_y = dimensions["height_px"] / physical_height_cm
    if scale_x <= 0.0 or scale_y <= 0.0:
        return None
    mean_scale = math.sqrt(scale_x * scale_y)
    anisotropy = abs(scale_x - scale_y) / max((scale_x + scale_y) * 0.5, 1e-6)
    dimensions["scale_x"] = scale_x
    dimensions["scale_y"] = scale_y
    dimensions["frame_scale"] = mean_scale
    dimensions["anisotropy"] = anisotropy
    return dimensions


def _interpolate_table(scale, calibration_points):
    points = []
    for distance_cm, measured_scale in calibration_points:
        if distance_cm > 0.0 and measured_scale > 0.0:
            points.append((float(measured_scale), float(distance_cm)))
    if len(points) < 2:
        return None
    points.sort(key=lambda item: item[0], reverse=True)

    if scale >= points[0][0]:
        first = points[0]
        second = points[1]
    elif scale <= points[-1][0]:
        first = points[-2]
        second = points[-1]
    else:
        first = points[0]
        second = points[1]
        for index in range(len(points) - 1):
            high = points[index]
            low = points[index + 1]
            if high[0] >= scale >= low[0]:
                first = high
                second = low
                break

    denominator = second[0] - first[0]
    if abs(denominator) < 1e-9:
        return (first[1] + second[1]) * 0.5
    ratio = (scale - first[0]) / denominator
    return first[1] + (second[1] - first[1]) * ratio


def estimate_distance(high_result):
    if high_result is None:
        return None
    if high_result.get("measurement_valid") is False:
        return None

    outer_measurement = frame_scale(
        high_result.get("outer_corners"),
        OUTER_WIDTH_CM,
        OUTER_HEIGHT_CM,
    )
    inner_measurement = frame_scale(
        high_result.get("inner_corners"),
        INNER_WIDTH_CM,
        INNER_HEIGHT_CM,
    )
    outer_inner_disagreement = 0.0
    if outer_measurement is not None and inner_measurement is not None:
        outer_inner_disagreement = abs(
            outer_measurement["frame_scale"]
            - inner_measurement["frame_scale"]
        ) / max(inner_measurement["frame_scale"], 1e-6)

    use_outer = (
        high_result.get("outer_corners") is not None
        and high_result.get("outer_valid_sides", 0) >= 3
        and outer_measurement is not None
        and outer_measurement["anisotropy"] <= 0.08
        and outer_inner_disagreement <= 0.06
    )
    if use_outer:
        source = "OUTER_A4"
        measurement = outer_measurement
    else:
        source = "INNER_APERTURE"
        measurement = inner_measurement
    if measurement is None:
        return None

    scale = measurement["frame_scale"]
    distance_cm = _interpolate_table(scale, DISTANCE_CALIBRATION_POINTS)
    if distance_cm is None:
        distance_cm = PROVISIONAL_FOCAL_LENGTH_PX / scale
        method = "PROVISIONAL_PINHOLE"
    else:
        method = "CALIBRATION_TABLE"

    return {
        "distance_cm": distance_cm,
        "method": method,
        "source": source,
        "frame_scale": scale,
        "scale_x": measurement["scale_x"],
        "scale_y": measurement["scale_y"],
        "anisotropy": measurement["anisotropy"],
        "width_px": measurement["width_px"],
        "height_px": measurement["height_px"],
        "outer_inner_disagreement": outer_inner_disagreement,
        "in_official_range": 95.0 <= distance_cm <= 205.0,
    }
