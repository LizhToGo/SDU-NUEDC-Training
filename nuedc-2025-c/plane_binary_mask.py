"""Perspective-corrected binary view of the A4 inner aperture.

The production camera frame is RGB565 and the target plane is described by a
``PlaneMapper``.  This module samples that quadrilateral into a compact regular
grid measured in pixels per centimetre.  All later advanced-target geometry is
therefore performed in physical coordinates and is independent of camera
distance or plane tilt.

Only plain Python containers and ``math`` are used so the same implementation
runs in CanMV MicroPython and in the desktop tests.
"""

import math


DEFAULT_PIXELS_PER_CM = 8.0
DEFAULT_BLACK_THRESHOLD_MIN = 35
DEFAULT_BLACK_THRESHOLD_MAX = 210


def _clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _gray_from_pixel(pixel):
    """Return an 8-bit luminance for CanMV tuples, RGB565 ints or scalars."""
    if pixel is None:
        return None
    if isinstance(pixel, (tuple, list)):
        if len(pixel) >= 3:
            red = int(pixel[0])
            green = int(pixel[1])
            blue = int(pixel[2])
            return (77 * red + 150 * green + 29 * blue) >> 8
        if len(pixel) > 0:
            return int(pixel[0])
        return None
    value = int(pixel)
    if value > 255:
        red = ((value >> 11) & 0x1F) * 255 // 31
        green = ((value >> 5) & 0x3F) * 255 // 63
        blue = (value & 0x1F) * 255 // 31
        return (77 * red + 150 * green + 29 * blue) >> 8
    return _clamp(value, 0, 255)


def _otsu_threshold(histogram, sample_count):
    if sample_count <= 0:
        return 127
    total_sum = 0.0
    for index in range(256):
        total_sum += index * histogram[index]

    background_count = 0
    background_sum = 0.0
    best_variance = -1.0
    best_threshold = 127
    for threshold in range(256):
        count = histogram[threshold]
        background_count += count
        background_sum += threshold * count
        if background_count <= 0:
            continue
        foreground_count = sample_count - background_count
        if foreground_count <= 0:
            break
        background_mean = background_sum / background_count
        foreground_mean = (total_sum - background_sum) / foreground_count
        difference = background_mean - foreground_mean
        variance = background_count * foreground_count * difference * difference
        if variance > best_variance:
            best_variance = variance
            best_threshold = threshold
    return best_threshold


def _percentile(histogram, sample_count, fraction):
    if sample_count <= 0:
        return 0
    target = max(1, int(sample_count * fraction))
    cumulative = 0
    for value in range(256):
        cumulative += histogram[value]
        if cumulative >= target:
            return value
    return 255


class PlaneBinaryMask:
    """Compact grayscale and black/white grids in A4-plane coordinates."""

    def __init__(
        self,
        width,
        height,
        pixels_per_cm,
        gray,
        black,
        threshold,
        valid_samples=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.pixels_per_cm = float(pixels_per_cm)
        self.gray = gray
        self.black = black
        self.threshold = int(threshold)
        self.valid_samples = (
            self.width * self.height
            if valid_samples is None
            else int(valid_samples)
        )
        self.black_count = 0
        for value in black:
            if value:
                self.black_count += 1

    @classmethod
    def from_image(
        cls,
        image,
        mapper,
        pixels_per_cm=DEFAULT_PIXELS_PER_CM,
        threshold=None,
        denoise=True,
    ):
        pixels_per_cm = float(pixels_per_cm)
        width = max(8, int(round(mapper.plane_width_cm * pixels_per_cm)))
        height = max(8, int(round(mapper.plane_height_cm * pixels_per_cm)))
        total = width * height
        gray = bytearray(total)
        histogram = [0] * 256
        valid_samples = 0

        # Increment the homogeneous plane-to-image numerator/denominator along
        # every output row.  This avoids tens of thousands of Python function
        # calls and repeated matrix indexing on the K230.
        matrix = mapper._plane_to_image
        image_width = image.width()
        image_height = image.height()
        plane_step = 1.0 / pixels_per_cm
        first_u = plane_step * 0.5
        index = 0
        for row in range(height):
            v_cm = (row + 0.5) * plane_step
            numerator_x = (
                matrix[0] * first_u + matrix[1] * v_cm + matrix[2]
            )
            numerator_y = (
                matrix[3] * first_u + matrix[4] * v_cm + matrix[5]
            )
            denominator = (
                matrix[6] * first_u + matrix[7] * v_cm + matrix[8]
            )
            increment_x = matrix[0] * plane_step
            increment_y = matrix[3] * plane_step
            increment_denominator = matrix[6] * plane_step
            for _ in range(width):
                value = 255
                if abs(denominator) > 1.0e-10:
                    image_x = int(numerator_x / denominator + 0.5)
                    image_y = int(numerator_y / denominator + 0.5)
                    if (
                        image_x >= 0
                        and image_y >= 0
                        and image_x < image_width
                        and image_y < image_height
                    ):
                        sampled = _gray_from_pixel(
                            image.get_pixel(image_x, image_y)
                        )
                        if sampled is not None:
                            value = _clamp(int(sampled), 0, 255)
                            valid_samples += 1
                gray[index] = value
                histogram[value] += 1
                index += 1
                numerator_x += increment_x
                numerator_y += increment_y
                denominator += increment_denominator

        if threshold is None:
            otsu = _otsu_threshold(histogram, total)
            dark_quartile = _percentile(histogram, total, 0.25)
            bright_quartile = _percentile(histogram, total, 0.75)
            # With an empty or nearly empty target Otsu can sit at an extreme.
            # The percentile midpoint keeps the threshold useful while the
            # explicit limits reject dark shadows from the white paper.
            if bright_quartile - dark_quartile >= 20:
                midpoint = (dark_quartile + bright_quartile) // 2
                otsu = int((2 * otsu + midpoint) / 3)
            threshold = _clamp(
                otsu,
                DEFAULT_BLACK_THRESHOLD_MIN,
                DEFAULT_BLACK_THRESHOLD_MAX,
            )
        else:
            threshold = _clamp(int(threshold), 0, 255)

        black = bytearray(total)
        for item in range(total):
            if gray[item] <= threshold:
                black[item] = 1

        if denoise:
            black = cls._remove_isolated_pixels(black, width, height)
        return cls(
            width,
            height,
            pixels_per_cm,
            gray,
            black,
            threshold,
            valid_samples,
        )

    @classmethod
    def from_binary_grid(cls, rows, pixels_per_cm=DEFAULT_PIXELS_PER_CM):
        """Desktop-test constructor accepting rows of truthy black values."""
        height = len(rows)
        width = len(rows[0]) if height else 0
        gray = bytearray(width * height)
        black = bytearray(width * height)
        index = 0
        for row in rows:
            if len(row) != width:
                raise ValueError("all mask rows must have equal length")
            for value in row:
                is_black = bool(value)
                black[index] = 1 if is_black else 0
                gray[index] = 0 if is_black else 255
                index += 1
        return cls(
            width,
            height,
            float(pixels_per_cm),
            gray,
            black,
            127,
            width * height,
        )

    @staticmethod
    def _remove_isolated_pixels(source, width, height):
        if width < 3 or height < 3:
            return source
        result = bytearray(source)
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                index = row + x
                neighbours = (
                    source[index - width - 1]
                    + source[index - width]
                    + source[index - width + 1]
                    + source[index - 1]
                    + source[index + 1]
                    + source[index + width - 1]
                    + source[index + width]
                    + source[index + width + 1]
                )
                if source[index] and neighbours <= 1:
                    result[index] = 0
                elif not source[index] and neighbours >= 7:
                    result[index] = 1
        return result

    def index(self, x, y):
        return int(y) * self.width + int(x)

    def is_black_pixel(self, x, y):
        x = int(x)
        y = int(y)
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        return bool(self.black[y * self.width + x])

    def is_black_plane(self, u_cm, v_cm):
        x = int(float(u_cm) * self.pixels_per_cm)
        y = int(float(v_cm) * self.pixels_per_cm)
        return self.is_black_pixel(x, y)

    def gray_plane(self, u_cm, v_cm):
        x = int(float(u_cm) * self.pixels_per_cm)
        y = int(float(v_cm) * self.pixels_per_cm)
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return 255
        return self.gray[y * self.width + x]

    def pixel_to_plane(self, x, y):
        return (
            (float(x) + 0.5) / self.pixels_per_cm,
            (float(y) + 0.5) / self.pixels_per_cm,
        )

    def plane_width_cm(self):
        return self.width / self.pixels_per_cm

    def plane_height_cm(self):
        return self.height / self.pixels_per_cm

    def black_fraction(self):
        total = self.width * self.height
        if total <= 0:
            return 0.0
        return self.black_count / float(total)

    def save_pgm(self, path):
        """Save the rectified black/white diagnostic without image libraries."""
        pixels = bytearray(self.width * self.height)
        for index in range(len(pixels)):
            pixels[index] = 0 if self.black[index] else 255
        with open(path, "wb") as file:
            file.write(
                ("P5\n%d %d\n255\n" % (self.width, self.height)).encode()
            )
            file.write(pixels)
        return path
