import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from inner_aperture_locator import InnerApertureLocator, scale_seed


class SyntheticBlob:
    def __init__(self, rect, fill_ratio=0.82):
        self._rect = tuple(rect)
        self._pixels = int(rect[2] * rect[3] * fill_ratio)

    def rect(self):
        return self._rect

    def pixels(self):
        return self._pixels


class SyntheticApertureImage:
    """Axis-aligned white aperture with optional central black geometry."""

    def __init__(
        self,
        rect=(310, 126, 70, 106),
        shape="CIRCLE",
        background=(20, 70, 200),
        border_sides=(True, True, True, True),
        emit_blob=True,
    ):
        self.rect = tuple(rect)
        self.shape = shape
        self.background = background
        self.border_sides = tuple(border_sides)
        self.emit_blob = emit_blob
        self.find_blob_calls = []

    def width(self):
        return 640

    def height(self):
        return 360

    def find_blobs(self, thresholds, **kwargs):
        self.find_blob_calls.append((thresholds, kwargs))
        if not self.emit_blob:
            return []
        return [SyntheticBlob(self.rect)]

    def _inside_center_shape(self, x, y):
        left, top, width, height = self.rect
        center_x = left + width * 0.5
        center_y = top + height * 0.5
        dx = x - center_x
        dy = y - center_y
        if self.shape == "CIRCLE":
            radius = min(width, height) * 0.18
            return dx * dx + dy * dy <= radius * radius
        if self.shape == "SQUARE":
            half = min(width, height) * 0.18
            return abs(dx) <= half and abs(dy) <= half
        if self.shape == "TRIANGLE":
            half = min(width, height) * 0.22
            if dy < -half or dy > half:
                return False
            allowed_x = (dy + half) * 0.5
            return abs(dx) <= allowed_x
        return False

    def get_pixel(self, x, y):
        left, top, width, height = self.rect
        right = left + width - 1
        bottom = top + height - 1
        if left <= x <= right and top <= y <= bottom:
            if self._inside_center_shape(x, y):
                return (18, 18, 18)
            return (235, 235, 235)

        border = 8
        on_top = (
            self.border_sides[0]
            and left <= x <= right
            and top - border <= y < top
        )
        on_right = (
            self.border_sides[1]
            and right < x <= right + border
            and top <= y <= bottom
        )
        on_bottom = (
            self.border_sides[2]
            and left <= x <= right
            and bottom < y <= bottom + border
        )
        on_left = (
            self.border_sides[3]
            and left - border <= x < left
            and top <= y <= bottom
        )
        if on_top or on_right or on_bottom or on_left:
            return (18, 18, 18)
        return self.background


class InnerApertureLocatorTests(unittest.TestCase):
    def setUp(self):
        self.locator = InnerApertureLocator()
        self.roi = (237, 0, 218, 360)

    def test_blue_background_and_four_black_sides_are_accepted(self):
        image = SyntheticApertureImage(background=(15, 65, 210))

        results = self.locator.detect_hypotheses(image, roi=self.roi)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["seed_class"], "ROI_SEED")
        self.assertEqual(results[0]["inner_bbox"], image.rect)
        self.assertGreater(results[0]["aperture_border_contrast"], 100.0)
        self.assertEqual(self.locator.last_error, "")

    def test_center_shape_does_not_change_aperture_seed(self):
        seeds = []
        for shape in ("CIRCLE", "TRIANGLE", "SQUARE"):
            image = SyntheticApertureImage(shape=shape)
            result = self.locator.detect_hypotheses(
                image,
                roi=self.roi,
            )[0]
            seeds.append((
                result["inner_corners"],
                result["aperture_border_pass_ratio"],
                result["aperture_min_side_pass_ratio"],
            ))

        self.assertEqual(seeds[0], seeds[1])
        self.assertEqual(seeds[1], seeds[2])

    def test_white_rectangle_without_dark_border_is_rejected(self):
        image = SyntheticApertureImage(
            background=(235, 235, 235),
            border_sides=(False, False, False, False),
        )

        results = self.locator.detect_hypotheses(image, roi=self.roi)

        self.assertEqual(results, [])
        self.assertGreaterEqual(
            self.locator.last_rejection_counts.get("BORDER_CONTRAST", 0),
            1,
        )

    def test_size_outside_calibrated_range_is_rejected(self):
        image = SyntheticApertureImage(rect=(338, 160, 18, 28))

        results = self.locator.detect_hypotheses(image, roi=self.roi)

        self.assertEqual(results, [])
        self.assertGreaterEqual(
            self.locator.last_rejection_counts.get("PHYSICAL_SHORT", 0),
            1,
        )

    def test_projective_seed_accepts_compressed_aperture_envelope(self):
        # A steeply tilted sheet can make the white aperture's axis-aligned
        # blob box much wider than the normal A4 aspect envelope.  The
        # projective path is intentionally relaxed because high-res edge
        # refinement remains the authoritative gate.
        image = SyntheticApertureImage(rect=(270, 160, 120, 30))

        self.assertEqual(
            self.locator.detect_hypotheses(image, roi=self.roi),
            [],
        )
        projective = self.locator.detect_hypotheses(
            image,
            roi=self.roi,
            allow_projective=True,
        )

        self.assertEqual(len(projective), 1)
        self.assertTrue(projective[0]["projective_seed"])
        self.assertEqual(projective[0]["seed_class"], "ROI_SEED")

    def test_missing_one_black_side_is_rejected(self):
        image = SyntheticApertureImage(
            background=(235, 235, 235),
            border_sides=(False, True, True, True),
        )

        results = self.locator.detect_hypotheses(image, roi=self.roi)

        self.assertEqual(results, [])
        self.assertGreaterEqual(
            self.locator.last_rejection_counts.get("BORDER_SIDE", 0),
            1,
        )

    def test_scale_seed_maps_1080p_seed_back_to_preview(self):
        seed = {
            "inner_corners": (
                (930, 378),
                (1137, 378),
                (1137, 693),
                (930, 693),
            ),
            "outer_corners": None,
            "inner_bbox": (930, 378, 208, 316),
            "seed_source": "HIGH",
        }

        scaled = scale_seed(
            seed,
            1.0 / 3.0,
            1.0 / 3.0,
            source="HIGH_RES_INNER_APERTURE",
        )

        self.assertEqual(
            scaled["inner_corners"],
            ((310, 126), (379, 126), (379, 231), (310, 231)),
        )
        self.assertEqual(scaled["inner_bbox"], (310, 126, 69, 105))
        self.assertEqual(
            scaled["seed_source"],
            "HIGH_RES_INNER_APERTURE",
        )


if __name__ == "__main__":
    unittest.main()
