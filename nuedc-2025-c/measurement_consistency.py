"""Small pure-Python helpers shared by measurement and calibration scripts."""

import math


def relative_scale_difference(first, second):
    """Return the symmetric relative difference between two positive scales."""
    return abs(first - second) / max((first + second) * 0.5, 1e-6)


def quad_center(corners):
    if corners is None or len(corners) != 4:
        return None
    total_x = 0.0
    total_y = 0.0
    for x, y in corners:
        total_x += x
        total_y += y
    return total_x * 0.25, total_y * 0.25


def quad_short_side(corners):
    if corners is None or len(corners) != 4:
        return 0.0
    side_lengths = []
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        side_lengths.append(math.sqrt(dx * dx + dy * dy))
    opposite_a = (side_lengths[0] + side_lengths[2]) * 0.5
    opposite_b = (side_lengths[1] + side_lengths[3]) * 0.5
    return min(opposite_a, opposite_b)


def relative_center_shift(first_corners, second_corners):
    """Normalize center translation by the mean inner-aperture short side."""
    first_center = quad_center(first_corners)
    second_center = quad_center(second_corners)
    if first_center is None or second_center is None:
        return 999.0
    dx = second_center[0] - first_center[0]
    dy = second_center[1] - first_center[1]
    center_distance = math.sqrt(dx * dx + dy * dy)
    reference_side = (
        quad_short_side(first_corners) + quad_short_side(second_corners)
    ) * 0.5
    return center_distance / max(reference_side, 1e-6)

