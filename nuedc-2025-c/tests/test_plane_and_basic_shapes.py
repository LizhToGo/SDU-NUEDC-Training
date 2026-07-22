import math
import os
import sys
import unittest


PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from basic_shape_detector import BasicShapeDetector, draw_shape_overlay
from plane_mapper import PlaneMapper


IMAGE_CORNERS = (
    (176.0, 72.0),
    (438.0, 55.0),
    (474.0, 421.0),
    (137.0, 443.0),
)


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    previous = len(polygon) - 1
    for current in range(len(polygon)):
        x1, y1 = polygon[current]
        x2, y2 = polygon[previous]
        if (y1 > y) != (y2 > y):
            crossing = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing:
                inside = not inside
        previous = current
    return inside


class FakeBlob:
    def __init__(self, rect, pixels, center):
        self._rect = rect
        self._pixels = pixels
        self._center = center

    def rect(self):
        return self._rect

    def pixels(self):
        return self._pixels

    def cx(self):
        return self._center[0]

    def cy(self):
        return self._center[1]


class NoSteppedSliceSequence:
    """Model the slice limitation of the CanMV MicroPython sequence type."""

    def __init__(self, values):
        self.values = values

    def __len__(self):
        return len(self.values)

    def __getitem__(self, index):
        if isinstance(index, slice) and index.step not in (None, 1):
            raise NotImplementedError("stepped slices are unsupported")
        return self.values[index]


class FakeDrawingImage:
    def __init__(self):
        self.line_count = 0
        self.cross_count = 0
        self.circle_count = 0

    def draw_line(self, *args, **kwargs):
        self.line_count += 1

    def draw_cross(self, *args, **kwargs):
        self.cross_count += 1

    def draw_circle(self, *args, **kwargs):
        self.circle_count += 1


class SyntheticShapeImage:
    def __init__(self, shape_type, x_cm, rotation_deg=0.0):
        self.mapper = PlaneMapper(IMAGE_CORNERS)
        self.shape_type = shape_type
        self.x_cm = float(x_cm)
        self.rotation = math.radians(rotation_deg)
        self.center = (8.5, 12.85)
        self._width = 640
        self._height = 480
        self.vertices = self._shape_vertices()
        boundary = self._boundary_points()
        projected = [
            self.mapper.plane_to_image(point[0], point[1])
            for point in boundary
        ]
        minimum_x = min(point[0] for point in projected)
        maximum_x = max(point[0] for point in projected)
        minimum_y = min(point[1] for point in projected)
        maximum_y = max(point[1] for point in projected)
        rect = (
            int(math.floor(minimum_x)),
            int(math.floor(minimum_y)),
            int(math.ceil(maximum_x - minimum_x)) + 1,
            int(math.ceil(maximum_y - minimum_y)) + 1,
        )
        image_center = self.mapper.plane_to_image(
            self.center[0], self.center[1]
        )
        if shape_type == "CIRCLE":
            area_cm2 = math.pi * (x_cm * 0.5) ** 2
        elif shape_type == "SQUARE":
            area_cm2 = x_cm * x_cm
        else:
            area_cm2 = math.sqrt(3.0) * x_cm * x_cm * 0.25
        one_pixel_area = self.mapper.image_area_to_plane(
            1.0, image_center[0], image_center[1]
        )
        pixels = int(round(area_cm2 / one_pixel_area))
        self.blob = FakeBlob(rect, pixels, image_center)

    def width(self):
        return self._width

    def height(self):
        return self._height

    def find_blobs(self, thresholds, **kwargs):
        return [self.blob]

    def _rotate(self, x, y):
        cosine = math.cos(self.rotation)
        sine = math.sin(self.rotation)
        return (
            self.center[0] + x * cosine - y * sine,
            self.center[1] + x * sine + y * cosine,
        )

    def _shape_vertices(self):
        if self.shape_type == "SQUARE":
            half = self.x_cm * 0.5
            return tuple(
                self._rotate(x, y)
                for x, y in ((-half, -half), (half, -half), (half, half), (-half, half))
            )
        if self.shape_type == "TRIANGLE":
            radius = self.x_cm / math.sqrt(3.0)
            return tuple(
                self._rotate(
                    radius * math.cos(math.radians(angle)),
                    radius * math.sin(math.radians(angle)),
                )
                for angle in (-90.0, 30.0, 150.0)
            )
        return None

    def _boundary_points(self):
        if self.shape_type == "CIRCLE":
            radius = self.x_cm * 0.5
            return tuple(
                self._rotate(
                    radius * math.cos(math.radians(angle)),
                    radius * math.sin(math.radians(angle)),
                )
                for angle in range(360)
            )
        return self.vertices

    def _is_black(self, u_cm, v_cm):
        if self.shape_type == "CIRCLE":
            return (
                (u_cm - self.center[0]) ** 2
                + (v_cm - self.center[1]) ** 2
                <= (self.x_cm * 0.5) ** 2
            )
        return point_in_polygon((u_cm, v_cm), self.vertices)

    def get_pixel(self, x, y):
        u_cm, v_cm = self.mapper.image_to_plane(float(x), float(y))
        if self._is_black(u_cm, v_cm):
            return (32, 32, 32)
        return (220, 220, 220)


class PlaneMapperTests(unittest.TestCase):
    def test_corner_and_round_trip_mapping(self):
        mapper = PlaneMapper(IMAGE_CORNERS)
        expected = ((0.0, 0.0), (17.0, 0.0), (17.0, 25.7), (0.0, 25.7))
        for image_point, plane_point in zip(IMAGE_CORNERS, expected):
            actual = mapper.image_to_plane(*image_point)
            self.assertAlmostEqual(actual[0], plane_point[0], places=6)
            self.assertAlmostEqual(actual[1], plane_point[1], places=6)

        for plane_point in ((1.2, 2.3), (8.5, 12.85), (15.8, 23.0), (4.0, 19.0)):
            image_point = mapper.plane_to_image(*plane_point)
            recovered = mapper.image_to_plane(*image_point)
            self.assertAlmostEqual(recovered[0], plane_point[0], places=6)
            self.assertAlmostEqual(recovered[1], plane_point[1], places=6)

    def test_known_physical_distance(self):
        mapper = PlaneMapper(IMAGE_CORNERS)
        first = mapper.plane_to_image(2.0, 3.0)
        second = mapper.plane_to_image(14.0, 8.0)
        self.assertAlmostEqual(
            mapper.distance_cm(first, second), 13.0, places=6
        )


class BasicShapeDetectorTests(unittest.TestCase):
    def test_circle_triangle_and_rotated_square(self):
        detector = BasicShapeDetector(angular_samples=72)
        cases = (
            ("CIRCLE", 12.0, 0.0),
            ("TRIANGLE", 12.0, 17.0),
            ("SQUARE", 12.0, 23.0),
        )
        for expected_type, expected_x, rotation in cases:
            with self.subTest(shape=expected_type):
                image = SyntheticShapeImage(
                    expected_type, expected_x, rotation
                )
                result = detector.detect(
                    image, image.mapper, IMAGE_CORNERS
                )
                self.assertTrue(result["shape_valid"], result)
                self.assertEqual(result["shape_type"], expected_type)
                self.assertAlmostEqual(result["x_cm"], expected_x, delta=0.35)
                self.assertGreater(result["confidence"], 0.55)

    def test_overlay_does_not_require_stepped_slices(self):
        outline = NoSteppedSliceSequence(
            tuple((index, index) for index in range(72))
        )
        image = FakeDrawingImage()
        draw_shape_overlay(
            image,
            {
                "shape_valid": True,
                "image_outline": outline,
                "image_center": (10, 10),
            },
        )
        self.assertEqual(image.line_count, 36)
        self.assertEqual(image.cross_count, 1)

    def test_polygon_overlay_uses_refined_corner_lines(self):
        image = FakeDrawingImage()
        draw_shape_overlay(
            image,
            {
                "shape_valid": True,
                "shape_type": "SQUARE",
                "image_outline": tuple((index, index) for index in range(72)),
                "image_corners": ((1, 1), (9, 1), (9, 9), (1, 9)),
                "image_center": (5, 5),
            },
        )
        self.assertEqual(image.line_count, 4)
        self.assertEqual(image.circle_count, 4)
        self.assertEqual(image.cross_count, 1)


if __name__ == "__main__":
    unittest.main()
