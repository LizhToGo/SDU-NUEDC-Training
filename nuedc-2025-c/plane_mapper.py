"""Project image points on the A4 aperture to physical centimetres.

The frame refiner returns the four inner-frame corners in TL, TR, BR, BL
order.  Those points describe a perspective projection of the known
17.0 cm x 25.7 cm white aperture.  ``PlaneMapper`` solves the two projective
homographies required to move between image pixels and that physical plane.

This module intentionally uses only ``math`` and plain Python containers so
it runs unchanged in CanMV MicroPython and in the desktop unit tests.
"""

import math


DEFAULT_PLANE_WIDTH_CM = 17.0
DEFAULT_PLANE_HEIGHT_CM = 25.7


def _distance(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return math.sqrt(dx * dx + dy * dy)


def _solve_linear_system(matrix, values):
    """Solve a small dense linear system with partial-pivot elimination."""
    size = len(values)
    rows = []
    for index in range(size):
        row = []
        for value in matrix[index]:
            row.append(float(value))
        row.append(float(values[index]))
        rows.append(row)

    for column in range(size):
        pivot_row = column
        pivot_value = abs(rows[column][column])
        for row_index in range(column + 1, size):
            candidate = abs(rows[row_index][column])
            if candidate > pivot_value:
                pivot_value = candidate
                pivot_row = row_index

        if pivot_value < 1.0e-10:
            raise ValueError("degenerate point set cannot form a homography")

        if pivot_row != column:
            rows[column], rows[pivot_row] = rows[pivot_row], rows[column]

        inverse_pivot = 1.0 / rows[column][column]
        for item in range(column, size + 1):
            rows[column][item] *= inverse_pivot

        for row_index in range(size):
            if row_index == column:
                continue
            factor = rows[row_index][column]
            if abs(factor) < 1.0e-15:
                continue
            for item in range(column, size + 1):
                rows[row_index][item] -= factor * rows[column][item]

    return tuple(rows[index][size] for index in range(size))


def _build_homography(source_points, target_points):
    """Return a row-major 3x3 homography with h[8] fixed to one."""
    if len(source_points) != 4 or len(target_points) != 4:
        raise ValueError("a homography requires exactly four point pairs")

    matrix = []
    values = []
    for index in range(4):
        x = float(source_points[index][0])
        y = float(source_points[index][1])
        u = float(target_points[index][0])
        v = float(target_points[index][1])

        matrix.append((x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y))
        values.append(u)
        matrix.append((0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y))
        values.append(v)

    result = _solve_linear_system(matrix, values)
    return (
        result[0],
        result[1],
        result[2],
        result[3],
        result[4],
        result[5],
        result[6],
        result[7],
        1.0,
    )


def _apply_homography(matrix, first, second):
    denominator = matrix[6] * first + matrix[7] * second + matrix[8]
    if abs(denominator) < 1.0e-12:
        raise ValueError("point projects to infinity")
    output_first = (
        matrix[0] * first + matrix[1] * second + matrix[2]
    ) / denominator
    output_second = (
        matrix[3] * first + matrix[4] * second + matrix[5]
    ) / denominator
    return (output_first, output_second)


class PlaneMapper:
    """Bidirectional mapping between image pixels and the A4 object plane."""

    def __init__(
        self,
        image_corners,
        plane_width_cm=DEFAULT_PLANE_WIDTH_CM,
        plane_height_cm=DEFAULT_PLANE_HEIGHT_CM,
    ):
        if len(image_corners) != 4:
            raise ValueError("image_corners must be TL, TR, BR, BL")
        if plane_width_cm <= 0.0 or plane_height_cm <= 0.0:
            raise ValueError("plane dimensions must be positive")

        self.plane_width_cm = float(plane_width_cm)
        self.plane_height_cm = float(plane_height_cm)
        self.image_corners = tuple(
            (float(point[0]), float(point[1])) for point in image_corners
        )
        self.plane_corners = (
            (0.0, 0.0),
            (self.plane_width_cm, 0.0),
            (self.plane_width_cm, self.plane_height_cm),
            (0.0, self.plane_height_cm),
        )

        self._image_to_plane = _build_homography(
            self.image_corners, self.plane_corners
        )
        self._plane_to_image = _build_homography(
            self.plane_corners, self.image_corners
        )

        # Catch a wrongly ordered or numerically degenerate quadrilateral
        # immediately rather than returning plausible-looking measurements.
        for index in range(4):
            projected = self.image_to_plane(
                self.image_corners[index][0], self.image_corners[index][1]
            )
            if _distance(projected, self.plane_corners[index]) > 1.0e-4:
                raise ValueError("homography corner validation failed")

    def image_to_plane(self, x, y):
        """Map an image pixel coordinate to ``(u_cm, v_cm)``."""
        return _apply_homography(self._image_to_plane, float(x), float(y))

    def plane_to_image(self, u_cm, v_cm):
        """Map a physical plane coordinate to a floating image coordinate."""
        return _apply_homography(
            self._plane_to_image, float(u_cm), float(v_cm)
        )

    def image_points_to_plane(self, points):
        return tuple(self.image_to_plane(point[0], point[1]) for point in points)

    def plane_points_to_image(self, points):
        return tuple(self.plane_to_image(point[0], point[1]) for point in points)

    def distance_cm(self, first_image_point, second_image_point):
        first = self.image_to_plane(
            first_image_point[0], first_image_point[1]
        )
        second = self.image_to_plane(
            second_image_point[0], second_image_point[1]
        )
        return _distance(first, second)

    def contains_plane_point(self, u_cm, v_cm, margin_cm=0.0):
        return (
            u_cm >= margin_cm
            and v_cm >= margin_cm
            and u_cm <= self.plane_width_cm - margin_cm
            and v_cm <= self.plane_height_cm - margin_cm
        )

    def image_area_to_plane(self, pixel_area, x, y):
        """Convert a small image area around ``(x, y)`` to square cm.

        The determinant of the image-to-plane homography Jacobian is used.
        It is primarily a diagnostic for blob filtering; final shape sizes are
        obtained from the perspective-corrected radial outline.
        """
        matrix = self._image_to_plane
        denominator = matrix[6] * x + matrix[7] * y + matrix[8]
        if abs(denominator) < 1.0e-12:
            raise ValueError("area centre projects to infinity")

        numerator_u = matrix[0] * x + matrix[1] * y + matrix[2]
        numerator_v = matrix[3] * x + matrix[4] * y + matrix[5]
        denominator_sq = denominator * denominator
        du_dx = (matrix[0] * denominator - matrix[6] * numerator_u) / denominator_sq
        du_dy = (matrix[1] * denominator - matrix[7] * numerator_u) / denominator_sq
        dv_dx = (matrix[3] * denominator - matrix[6] * numerator_v) / denominator_sq
        dv_dy = (matrix[4] * denominator - matrix[7] * numerator_v) / denominator_sq
        determinant = abs(du_dx * dv_dy - du_dy * dv_dx)
        return float(pixel_area) * determinant

    def local_pixels_per_cm(self, u_cm, v_cm):
        """Return the geometric-mean image scale at a physical point."""
        half_step = 0.5
        left = self.plane_to_image(u_cm - half_step, v_cm)
        right = self.plane_to_image(u_cm + half_step, v_cm)
        top = self.plane_to_image(u_cm, v_cm - half_step)
        bottom = self.plane_to_image(u_cm, v_cm + half_step)
        horizontal = _distance(left, right)
        vertical = _distance(top, bottom)
        if horizontal <= 0.0 or vertical <= 0.0:
            return 0.0
        return math.sqrt(horizontal * vertical)

    def image_rect_plane_bounds(self, rect):
        """Return the plane-axis bounds of an image ``(x, y, w, h)`` rect."""
        x, y, width, height = rect
        points = (
            self.image_to_plane(x, y),
            self.image_to_plane(x + width, y),
            self.image_to_plane(x + width, y + height),
            self.image_to_plane(x, y + height),
        )
        minimum_u = points[0][0]
        maximum_u = points[0][0]
        minimum_v = points[0][1]
        maximum_v = points[0][1]
        for u_cm, v_cm in points[1:]:
            minimum_u = min(minimum_u, u_cm)
            maximum_u = max(maximum_u, u_cm)
            minimum_v = min(minimum_v, v_cm)
            maximum_v = max(maximum_v, v_cm)
        return (minimum_u, minimum_v, maximum_u, maximum_v)

    def quad_dimensions_cm(self, image_corners):
        """Return average opposite-side dimensions for TL, TR, BR, BL."""
        points = self.image_points_to_plane(image_corners)
        width = (_distance(points[0], points[1]) + _distance(points[3], points[2])) * 0.5
        height = (_distance(points[0], points[3]) + _distance(points[1], points[2])) * 0.5
        return (width, height)

