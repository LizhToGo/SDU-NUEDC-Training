"""Shape-independent coarse localization of the bright A4 inner aperture.

The normal fast path uses ``find_rects()``.  At long range that operator may
miss a thin printed frame, and a square target can accidentally help it while a
circle or triangle cannot.  This module supplies a second, independent seed:
find the connected bright aperture, verify a dark band on all four sides, and
let the strict 1080p refiner determine the final measurement corners.

Only the inner-white to border-black transition is required.  The scene beyond
the printed border is deliberately ignored because it is not controlled in the
field and may be blue, black or white.  ``allow_projective=True`` is a bounded
relaxed-seed mode for the 30--60 degree task; it never bypasses the later
1080p edge/geometry validation.
"""

import math


DEFAULT_WHITE_THRESHOLD = (52, 100, -42, 42, -42, 42)

REFERENCE_WIDTH = 640.0
REFERENCE_HEIGHT = 360.0


def _clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def _mean(values):
    if not values:
        return None
    return sum(values) / len(values)


def _gray_from_pixel(pixel):
    if pixel is None:
        return None
    if isinstance(pixel, (tuple, list)):
        if len(pixel) >= 3:
            red = float(pixel[0])
            green = float(pixel[1])
            blue = float(pixel[2])
            return 0.299 * red + 0.587 * green + 0.114 * blue
        if pixel:
            return float(pixel[0])
        return None
    try:
        return float(pixel)
    except (TypeError, ValueError):
        return None


def _sample_gray(image, x, y):
    x = int(round(x))
    y = int(round(y))
    if x < 0 or y < 0 or x >= image.width() or y >= image.height():
        return None
    return _gray_from_pixel(image.get_pixel(x, y))


def _blob_call(blob, name, tuple_index, default=None):
    attribute = getattr(blob, name, None)
    if callable(attribute):
        try:
            return attribute()
        except Exception:
            pass
    if attribute is not None and not callable(attribute):
        return attribute
    try:
        return blob[tuple_index]
    except Exception:
        return default


def _blob_rect(blob):
    rect = _blob_call(blob, "rect", 0, None)
    if rect is not None and len(rect) >= 4:
        return (
            int(rect[0]),
            int(rect[1]),
            int(rect[2]),
            int(rect[3]),
        )
    x = _blob_call(blob, "x", 0, None)
    y = _blob_call(blob, "y", 1, None)
    width = _blob_call(blob, "w", 2, None)
    height = _blob_call(blob, "h", 3, None)
    if None in (x, y, width, height):
        return None
    return (int(x), int(y), int(width), int(height))


def _axis_aligned_corners(rect):
    x, y, width, height = rect
    right = x + width - 1
    bottom = y + height - 1
    return (
        (x, y),
        (right, y),
        (right, bottom),
        (x, bottom),
    )


def _scale_corners(corners, scale_x, scale_y):
    if corners is None:
        return None
    return tuple(
        (
            int(round(point[0] * scale_x)),
            int(round(point[1] * scale_y)),
        )
        for point in corners
    )


def scale_seed(seed, scale_x, scale_y, source=None):
    """Scale a locator seed into the preview coordinate system."""
    if seed is None:
        return None
    scaled = dict(seed)
    scaled["inner_corners"] = _scale_corners(
        seed.get("inner_corners"), scale_x, scale_y
    )
    scaled["outer_corners"] = _scale_corners(
        seed.get("outer_corners"), scale_x, scale_y
    )
    bbox = seed.get("inner_bbox")
    if bbox is not None:
        scaled["inner_bbox"] = (
            int(round(bbox[0] * scale_x)),
            int(round(bbox[1] * scale_y)),
            max(1, int(round(bbox[2] * scale_x))),
            max(1, int(round(bbox[3] * scale_y))),
        )
    if source is not None:
        scaled["seed_source"] = source
    return scaled


class InnerApertureLocator:
    """Find a central bright A4 aperture surrounded by a dark four-side band."""

    def __init__(
        self,
        white_threshold=DEFAULT_WHITE_THRESHOLD,
        expected_aspect=25.7 / 17.0,
        min_short_px=26.0,
        max_short_px=80.0,
        min_long_px=38.0,
        max_long_px=125.0,
        min_aspect=1.25,
        max_aspect=1.78,
        min_border_contrast=18.0,
        min_border_pass_ratio=0.55,
        min_side_pass_ratio=0.40,
        min_fill_ratio=0.32,
    ):
        self.white_threshold = white_threshold
        self.expected_aspect = float(expected_aspect)
        self.min_short_px = float(min_short_px)
        self.max_short_px = float(max_short_px)
        self.min_long_px = float(min_long_px)
        self.max_long_px = float(max_long_px)
        self.min_aspect = float(min_aspect)
        self.max_aspect = float(max_aspect)
        self.min_border_contrast = float(min_border_contrast)
        self.min_border_pass_ratio = float(min_border_pass_ratio)
        self.min_side_pass_ratio = float(min_side_pass_ratio)
        self.min_fill_ratio = float(min_fill_ratio)

        self.last_roi = None
        self.last_blob_count = 0
        self.last_candidates = []
        self.last_rejection_counts = {}
        self.last_error = ""

    def _reject(self, reason):
        self.last_rejection_counts[reason] = (
            self.last_rejection_counts.get(reason, 0) + 1
        )
        return None

    def _image_scale(self, image):
        scale_x = image.width() / REFERENCE_WIDTH
        scale_y = image.height() / REFERENCE_HEIGHT
        return max(0.1, min(scale_x, scale_y))

    def _find_blobs(self, image, roi, scale):
        pixels_threshold = max(40, int(round(100.0 * scale * scale)))
        area_threshold = max(60, int(round(160.0 * scale * scale)))
        try:
            return image.find_blobs(
                [self.white_threshold],
                roi=roi,
                pixels_threshold=pixels_threshold,
                area_threshold=area_threshold,
                merge=False,
            )
        except TypeError:
            return image.find_blobs(
                [self.white_threshold],
                roi=roi,
                pixels_threshold=pixels_threshold,
                area_threshold=area_threshold,
            )

    def _sample_side(self, image, rect, side, scale):
        x, y, width, height = rect
        short_side = min(width, height)
        inside_probe = max(1.0, min(5.0 * scale, short_side * 0.055))
        border_limit = int(round(short_side * 0.18))
        border_limit = max(int(round(3.0 * scale)), border_limit)
        border_limit = min(max(3, border_limit), max(3, int(10 * scale)))
        positions = (0.18, 0.34, 0.50, 0.66, 0.82)

        contrasts = []
        inside_values = []
        border_values = []
        passes = 0
        for position in positions:
            if side == 0:  # top, inward is +y
                base_x = x + (width - 1) * position
                base_y = y
                normal_x, normal_y = 0.0, 1.0
            elif side == 1:  # right, inward is -x
                base_x = x + width - 1
                base_y = y + (height - 1) * position
                normal_x, normal_y = -1.0, 0.0
            elif side == 2:  # bottom, inward is -y
                base_x = x + (width - 1) * (1.0 - position)
                base_y = y + height - 1
                normal_x, normal_y = 0.0, -1.0
            else:  # left, inward is +x
                base_x = x
                base_y = y + (height - 1) * (1.0 - position)
                normal_x, normal_y = 1.0, 0.0

            inside_samples = []
            for factor in (1.0, 1.65):
                value = _sample_gray(
                    image,
                    base_x + normal_x * inside_probe * factor,
                    base_y + normal_y * inside_probe * factor,
                )
                if value is not None:
                    inside_samples.append(value)
            inside_gray = _mean(inside_samples)

            outward_samples = []
            for distance in range(1, border_limit + 1):
                value = _sample_gray(
                    image,
                    base_x - normal_x * distance,
                    base_y - normal_y * distance,
                )
                if value is not None:
                    outward_samples.append(value)
            if inside_gray is None or len(outward_samples) < 2:
                continue

            outward_samples.sort()
            border_gray = _mean(outward_samples[: min(2, len(outward_samples))])
            contrast = inside_gray - border_gray
            contrasts.append(contrast)
            inside_values.append(inside_gray)
            border_values.append(border_gray)
            if contrast >= self.min_border_contrast:
                passes += 1

        valid = len(contrasts)
        return {
            "valid_samples": valid,
            "pass_ratio": passes / max(valid, 1),
            "contrast": _mean(contrasts) if contrasts else -999.0,
            "inside_gray": _mean(inside_values),
            "border_gray": _mean(border_values),
        }

    def _border_metrics(self, image, rect, scale):
        sides = []
        total_valid = 0
        total_pass = 0.0
        weighted_contrast = 0.0
        for side_index in range(4):
            side = self._sample_side(image, rect, side_index, scale)
            sides.append(side)
            total_valid += side["valid_samples"]
            total_pass += side["pass_ratio"] * side["valid_samples"]
            weighted_contrast += side["contrast"] * side["valid_samples"]
        return {
            "valid_samples": total_valid,
            "pass_ratio": total_pass / max(total_valid, 1),
            "min_side_pass_ratio": min(
                side["pass_ratio"] for side in sides
            ),
            "contrast": weighted_contrast / max(total_valid, 1),
            "side_pass_ratios": tuple(
                side["pass_ratio"] for side in sides
            ),
            "side_contrasts": tuple(side["contrast"] for side in sides),
        }

    def _candidate(
        self,
        image,
        blob,
        roi,
        scale,
        source,
        allow_projective=False,
    ):
        rect = _blob_rect(blob)
        if rect is None:
            return self._reject("BLOB_RECT")
        x, y, width, height = rect
        if width < 4 or height < 4:
            return self._reject("BLOB_SIZE")

        short_side = float(min(width, height))
        long_side = float(max(width, height))
        if allow_projective:
            # A 30--60 degree target can compress one physical axis and make
            # the axis-aligned blob box substantially wider/taller than the
            # front-facing A4 envelope.  This path is only a location seed;
            # the 1080p inner-edge refiner and tilt-specific geometry gate are
            # still authoritative.  Keep broad but finite limits so a full
            # image/background blob cannot become a seed.
            min_short = max(8.0, self.min_short_px * scale * 0.35)
            max_short = self.max_short_px * scale * 1.30
            min_long = max(12.0, self.min_long_px * scale * 0.35)
            max_long = self.max_long_px * scale * 1.65
            min_aspect = 1.0
            max_aspect = max(3.8, self.max_aspect * 2.5)
        else:
            min_short = self.min_short_px * scale
            max_short = self.max_short_px * scale
            min_long = self.min_long_px * scale
            max_long = self.max_long_px * scale
            min_aspect = self.min_aspect
            max_aspect = self.max_aspect
        if short_side < min_short or short_side > max_short:
            return self._reject("PHYSICAL_SHORT")
        if long_side < min_long or long_side > max_long:
            return self._reject("PHYSICAL_LONG")

        aspect = long_side / max(short_side, 1.0)
        if aspect < min_aspect or aspect > max_aspect:
            return self._reject("ASPECT")

        pixels = _blob_call(blob, "pixels", 4, width * height)
        try:
            pixels = max(1, int(pixels))
        except (TypeError, ValueError):
            pixels = width * height
        fill_ratio = pixels / max(width * height, 1)
        if fill_ratio < self.min_fill_ratio:
            return self._reject("WHITE_FILL")

        edge_margin = min(
            x,
            y,
            image.width() - (x + width),
            image.height() - (y + height),
        )
        edge_limit = (
            max(1, int(round(1.0 * scale)))
            if allow_projective
            else max(2, int(round(2.0 * scale)))
        )
        if edge_margin < edge_limit:
            return self._reject("IMAGE_EDGE")

        border = self._border_metrics(image, rect, scale)
        if border["valid_samples"] < (4 if allow_projective else 12):
            return self._reject("BORDER_SAMPLES")
        minimum_contrast = (
            self.min_border_contrast * 0.25
            if allow_projective
            else self.min_border_contrast
        )
        minimum_pass_ratio = (
            self.min_border_pass_ratio * 0.20
            if allow_projective
            else self.min_border_pass_ratio
        )
        minimum_side_pass_ratio = (
            self.min_side_pass_ratio * 0.15
            if allow_projective
            else self.min_side_pass_ratio
        )
        if border["contrast"] < minimum_contrast:
            return self._reject("BORDER_CONTRAST")
        if border["pass_ratio"] < minimum_pass_ratio:
            return self._reject("BORDER_COVERAGE")
        if border["min_side_pass_ratio"] < minimum_side_pass_ratio:
            return self._reject("BORDER_SIDE")

        roi_x, _, roi_width, _ = roi
        center_x = x + (width - 1) * 0.5
        roi_center_x = roi_x + (roi_width - 1) * 0.5
        center_error = abs(center_x - roi_center_x) / max(
            roi_width * 0.5, 1.0
        )
        aspect_score = _clamp(
            1.0
            - abs(aspect - self.expected_aspect)
            / (1.20 if allow_projective else 0.42)
        )
        center_score = _clamp(1.0 - center_error)
        contrast_score = _clamp(
            (border["contrast"] - self.min_border_contrast) / 70.0
        )
        border_score = (
            0.55 * border["pass_ratio"]
            + 0.45 * border["min_side_pass_ratio"]
        )
        score = (
            (0.16 if allow_projective else 0.30) * aspect_score
            + (0.16 if allow_projective else 0.20) * center_score
            + (0.18 if allow_projective else 0.20) * contrast_score
            + (0.50 if allow_projective else 0.30) * border_score
        )

        corners = _axis_aligned_corners(rect)
        return {
            "mode": "INNER_APERTURE_SEED",
            "outer_corners": None,
            "inner_corners": corners,
            "outer_bbox": None,
            "inner_bbox": rect,
            "score": score,
            "contrast": border["contrast"],
            "short_ratio": 0.0,
            "long_ratio": 0.0,
            "area_ratio": (width * height) / (
                image.width() * image.height()
            ),
            "outer_valid_sides": 0,
            "outer_edge_response": 0.0,
            "candidate_count": 1,
            "coarse_ring_inside_pass_ratio": border["pass_ratio"],
            "coarse_ring_outside_pass_ratio": 0.0,
            "coarse_ring_min_side_pass_ratio": border[
                "min_side_pass_ratio"
            ],
            "coarse_predicted_outer_margin": edge_margin,
            "coarse_soft_reasons": (),
            "coarse_regularity": {
                "max_opposite_error": 0.0,
                "diagonal_midpoint_error": 0.0,
                "fill_ratio": 1.0,
                "min_corner_angle": 90.0,
                "max_corner_angle": 90.0,
            },
            # The axis-aligned blob box is a location/scale hint. The 1080p
            # refiner regularizes ROI_SEED before fitting the four true sides.
            "seed_class": "ROI_SEED",
            "seed_source": source,
            "projective_seed": bool(allow_projective),
            "location_seed_relaxed": False,
            "aperture_fill_ratio": fill_ratio,
            "aperture_aspect": aspect,
            "aperture_short_px": short_side,
            "aperture_long_px": long_side,
            "aperture_border_contrast": border["contrast"],
            "aperture_border_pass_ratio": border["pass_ratio"],
            "aperture_min_side_pass_ratio": border[
                "min_side_pass_ratio"
            ],
            "aperture_side_pass_ratios": border["side_pass_ratios"],
            "aperture_side_contrasts": border["side_contrasts"],
        }

    def detect_hypotheses(
        self,
        image,
        roi=None,
        max_results=3,
        source="CENTER_APERTURE",
        allow_projective=False,
    ):
        """Return ranked inner-aperture seeds in ``image`` coordinates."""
        self.last_roi = roi
        self.last_blob_count = 0
        self.last_candidates = []
        self.last_rejection_counts = {}
        self.last_error = ""
        if roi is None:
            roi = (0, 0, image.width(), image.height())
            self.last_roi = roi

        scale = self._image_scale(image)
        try:
            blobs = self._find_blobs(image, roi, scale)
        except Exception as error:
            self.last_error = repr(error)
            self._reject("FIND_BLOBS_ERROR")
            return []
        if blobs is None:
            blobs = ()
        self.last_blob_count = len(blobs)

        candidates = []
        for blob in blobs:
            try:
                candidate = self._candidate(
                    image,
                    blob,
                    roi,
                    scale,
                    source,
                    allow_projective=allow_projective,
                )
            except Exception:
                candidate = self._reject("CANDIDATE_ERROR")
            if candidate is not None:
                candidates.append(candidate)
        candidates.sort(key=lambda item: item["score"], reverse=True)
        self.last_candidates = candidates
        return candidates[:max_results]
