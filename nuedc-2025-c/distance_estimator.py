"""Distance estimation from the known A4 reference-frame dimensions.

The official target is a 21.0 cm x 29.7 cm A4 sheet with a 2.0 cm black
border, leaving a 17.0 cm x 25.7 cm inner white rectangle.

For final accuracy, fill DISTANCE_CALIBRATION_POINTS with measurements from
fixed positions between 100 cm and 200 cm.  Each point is:

    (known_distance_cm, frame_scale_px_per_cm)

The distance path uses the geometric mean of the sub-pixel inner-aperture
width and height scales.  The outer A4 edge is retained only as an independent
diagnostic because the 100 cm repeatability test showed that its
background-to-black edge is substantially less stable.
The trigger program requires two consistent STRONG measurements, averages
their frame scales, and only then interpolates the calibration table in
reciprocal-scale space.  This keeps the lookup table while matching the
approximately linear pinhole relationship D = a / scale + b.
"""

import math


OUTER_WIDTH_CM = 21.0
OUTER_HEIGHT_CM = 29.7
INNER_WIDTH_CM = 17.0
INNER_HEIGHT_CM = 25.7

# Preliminary focal length inferred from the user's roughly 100/150/200 cm
# GC2093 photographs.  It is now only a fallback if the calibrated table is
# deliberately removed or contains fewer than two valid points.
PROVISIONAL_FOCAL_LENGTH_PX = 1155.0

# Visible in saved metadata so a temporary table cannot be mistaken for the
# former 2026-07-21 final table.
DISTANCE_CALIBRATION_VERSION = "TEMP-2026-07-22-10CM-KNOTS"

# TEMPORARY calibration for the current fixed camera/target setup.
# Exact knots were measured on 2026-07-22 at 10 cm intervals (110--190 cm)
# from measure_225..measure_233.  The 5 cm points between those knots and the
# 100/105/195/200 cm endpoints are reciprocal-scale interpolation/extrapolation
# placeholders so the complete 100--200 cm path can be tested immediately.
# Replace this block with a fresh 21-point, five-sample calibration before the
# final competition build.
DISTANCE_CALIBRATION_POINTS = (
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


def _distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def _polygon_area(corners):
    total = 0.0
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        total += first[0] * second[1] - second[0] * first[1]
    return abs(total) * 0.5


def quad_pixel_dimensions(corners):
    if corners is None or len(corners) != 4:
        return None
    top = _distance(corners[0], corners[1])
    right = _distance(corners[1], corners[2])
    bottom = _distance(corners[2], corners[3])
    left = _distance(corners[3], corners[0])
    dimension_a = (top + bottom) * 0.5
    dimension_b = (left + right) * 0.5
    return {
        "width_px": min(dimension_a, dimension_b),
        "height_px": max(dimension_a, dimension_b),
        "area_px2": _polygon_area(corners),
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
    area_scale = math.sqrt(
        dimensions["area_px2"]
        / (physical_width_cm * physical_height_cm)
    )
    geometric_mean_scale = math.sqrt(scale_x * scale_y)
    anisotropy = abs(scale_x - scale_y) / max((scale_x + scale_y) * 0.5, 1e-6)
    dimensions["scale_x"] = scale_x
    dimensions["scale_y"] = scale_y
    dimensions["area_scale"] = area_scale
    dimensions["frame_scale"] = geometric_mean_scale
    dimensions["scale_method"] = "SUBPIXEL_SIDE_GEOMETRIC_MEAN"
    dimensions["anisotropy"] = anisotropy
    return dimensions


def _interpolate_table(scale, calibration_points):
    """Interpolate distance linearly in reciprocal frame-scale space."""
    if scale <= 0.0:
        return None
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

    query_reciprocal = 1.0 / scale
    first_reciprocal = 1.0 / first[0]
    second_reciprocal = 1.0 / second[0]
    denominator = second_reciprocal - first_reciprocal
    if abs(denominator) < 1e-9:
        return (first[1] + second[1]) * 0.5
    ratio = (query_reciprocal - first_reciprocal) / denominator
    return first[1] + (second[1] - first[1]) * ratio


def _distance_from_scale(scale):
    distance_cm = _interpolate_table(scale, DISTANCE_CALIBRATION_POINTS)
    if distance_cm is None:
        return PROVISIONAL_FOCAL_LENGTH_PX / scale, "PROVISIONAL_PINHOLE"
    return distance_cm, "CALIBRATION_TABLE"


def estimate_distance(high_result):
    if high_result is None:
        return None
    if high_result.get("measurement_valid") is False:
        return None

    outer_measurement = frame_scale(
        high_result.get(
            "outer_corners_float", high_result.get("outer_corners")
        ),
        OUTER_WIDTH_CM,
        OUTER_HEIGHT_CM,
    )
    inner_measurement = frame_scale(
        high_result.get(
            "inner_corners_float", high_result.get("inner_corners")
        ),
        INNER_WIDTH_CM,
        INNER_HEIGHT_CM,
    )
    outer_inner_disagreement = 0.0
    if outer_measurement is not None and inner_measurement is not None:
        outer_inner_disagreement = abs(
            outer_measurement["frame_scale"]
            - inner_measurement["frame_scale"]
        ) / max(inner_measurement["frame_scale"], 1e-6)

    # The independently fitted inner black-to-white edge is the sole distance
    # scale.  Never silently fall back to an outer-only/reconstructed inner
    # frame: those cases are rejected by high_result.measurement_valid.
    source = "INNER_APERTURE_FLOAT_GEOM"
    measurement = inner_measurement
    if measurement is None:
        return None

    scale = measurement["frame_scale"]
    distance_cm, method = _distance_from_scale(scale)

    return {
        "distance_cm": distance_cm,
        "method": method,
        "calibration_version": DISTANCE_CALIBRATION_VERSION,
        "source": source,
        "scale_method": measurement["scale_method"],
        "frame_scale": scale,
        "scale_x": measurement["scale_x"],
        "scale_y": measurement["scale_y"],
        "anisotropy": measurement["anisotropy"],
        "width_px": measurement["width_px"],
        "height_px": measurement["height_px"],
        "area_px2": measurement["area_px2"],
        "outer_inner_disagreement": outer_inner_disagreement,
        "in_official_range": 95.0 <= distance_cm <= 205.0,
    }


def fuse_distance_results(results):
    """Fuse independent STRONG measurements in frame-scale space.

    Calibration maps image scale to distance, so scales must be averaged
    before applying the calibration table rather than averaging two already
    converted distances.
    """
    valid = []
    for result in results:
        if result is None:
            continue
        scale = result.get("frame_scale", 0.0)
        if scale > 0.0:
            valid.append(result)
    if len(valid) < 2:
        return None

    count = len(valid)
    scales = []
    scale_total = 0.0
    scale_x_total = 0.0
    scale_y_total = 0.0
    width_total = 0.0
    height_total = 0.0
    area_total = 0.0
    disagreement_total = 0.0
    maximum_disagreement = 0.0
    for result in valid:
        scale = result["frame_scale"]
        scales.append(scale)
        scale_total += scale
        scale_x_total += result.get("scale_x", scale)
        scale_y_total += result.get("scale_y", scale)
        width_total += result.get("width_px", 0.0)
        height_total += result.get("height_px", 0.0)
        area_total += result.get("area_px2", 0.0)
        disagreement = result.get("outer_inner_disagreement", 0.0)
        disagreement_total += disagreement
        maximum_disagreement = max(maximum_disagreement, disagreement)

    fused_scale = scale_total / count
    scale_x = scale_x_total / count
    scale_y = scale_y_total / count
    minimum_scale = min(scales)
    maximum_scale = max(scales)
    relative_spread = (maximum_scale - minimum_scale) / max(
        fused_scale, 1e-6
    )
    anisotropy = abs(scale_x - scale_y) / max(
        (scale_x + scale_y) * 0.5, 1e-6
    )
    distance_cm, method = _distance_from_scale(fused_scale)

    return {
        "distance_cm": distance_cm,
        "method": method,
        "calibration_version": DISTANCE_CALIBRATION_VERSION,
        "source": "INNER_APERTURE_FLOAT_GEOM_2FRAME",
        "scale_method": "MEAN_OF_CONSISTENT_SUBPIXEL_GEOMETRIC_SCALES",
        "frame_scale": fused_scale,
        "scale_x": scale_x,
        "scale_y": scale_y,
        "anisotropy": anisotropy,
        "width_px": width_total / count,
        "height_px": height_total / count,
        "area_px2": area_total / count,
        "outer_inner_disagreement": disagreement_total / count,
        "max_outer_inner_disagreement": maximum_disagreement,
        "fusion_method": "MEAN_FRAME_SCALE",
        "fusion_count": count,
        "component_scales": tuple(scales),
        "frame_scale_min": minimum_scale,
        "frame_scale_max": maximum_scale,
        "frame_scale_relative_spread": relative_spread,
        "frame_scale_spread_pct": relative_spread * 100.0,
        "in_official_range": 95.0 <= distance_cm <= 205.0,
    }
