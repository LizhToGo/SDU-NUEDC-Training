"""Tiny template recognizer for white digits printed inside black squares.

The official numbered target uses high-contrast white glyphs.  Running a KPU
network for ten characters would add latency and power without adding useful
information, so this module extracts the largest internal white component and
matches it against compact binary templates.  Symmetric one-pixel chamfer
overlap, aspect ratio and hole count make the match tolerant of font weight,
small scale changes and anti-aliasing.
"""

import math


TEMPLATE_WIDTH = 18
TEMPLATE_HEIGHT = 26


_DIGIT_PATTERNS = {
    0: (("01110", "10001", "10011", "10101", "11001", "10001", "01110"),),
    1: (
        ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
        ("010", "110", "010", "010", "010", "010", "111"),
    ),
    2: (("01110", "10001", "00001", "00010", "00100", "01000", "11111"),),
    3: (("11110", "00001", "00001", "01110", "00001", "00001", "11110"),),
    4: (
        ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
        ("00110", "01010", "10010", "11111", "00010", "00010", "00010"),
    ),
    5: (("11111", "10000", "10000", "11110", "00001", "00001", "11110"),),
    6: (("00110", "01000", "10000", "11110", "10001", "10001", "01110"),),
    7: (("11111", "00001", "00010", "00100", "01000", "01000", "01000"),),
    8: (("01110", "10001", "10001", "01110", "10001", "10001", "01110"),),
    9: (("01110", "10001", "10001", "01111", "00001", "00010", "11100"),),
}


def _popcount(value):
    try:
        return value.bit_count()
    except AttributeError:
        count = 0
        while value:
            value &= value - 1
            count += 1
        return count


def _normalise_points(points, source_width, source_height, width, height):
    if not points:
        return 0, 0.0
    minimum_x = points[0][0]
    maximum_x = points[0][0]
    minimum_y = points[0][1]
    maximum_y = points[0][1]
    for x, y in points[1:]:
        minimum_x = min(minimum_x, x)
        maximum_x = max(maximum_x, x)
        minimum_y = min(minimum_y, y)
        maximum_y = max(maximum_y, y)
    glyph_width = max(1.0, maximum_x - minimum_x + 1.0)
    glyph_height = max(1.0, maximum_y - minimum_y + 1.0)
    scale = min((width - 3.0) / glyph_width, (height - 3.0) / glyph_height)
    output_width = glyph_width * scale
    output_height = glyph_height * scale
    offset_x = (width - output_width) * 0.5
    offset_y = (height - output_height) * 0.5
    bits = 0
    for x, y in points:
        left = int(math.floor(offset_x + (x - minimum_x) * scale))
        right = int(math.ceil(offset_x + (x - minimum_x + 1.0) * scale))
        top = int(math.floor(offset_y + (y - minimum_y) * scale))
        bottom = int(math.ceil(offset_y + (y - minimum_y + 1.0) * scale))
        left = max(0, left)
        right = min(width, max(left + 1, right))
        top = max(0, top)
        bottom = min(height, max(top + 1, bottom))
        for target_y in range(top, bottom):
            for target_x in range(left, right):
                bits |= 1 << (target_y * width + target_x)
    aspect = glyph_width / glyph_height
    return bits, aspect


def _dilate(bits, width, height):
    result = bits
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if not (bits & (1 << index)):
                continue
            if x > 0:
                result |= 1 << (index - 1)
            if x + 1 < width:
                result |= 1 << (index + 1)
            if y > 0:
                result |= 1 << (index - width)
            if y + 1 < height:
                result |= 1 << (index + width)
    return result


def _hole_count(bits, width, height):
    visited = bytearray(width * height)
    queue = []

    def append_background(x, y):
        index = y * width + x
        if visited[index] or bits & (1 << index):
            return
        visited[index] = 1
        queue.append(index)

    for x in range(width):
        append_background(x, 0)
        append_background(x, height - 1)
    for y in range(height):
        append_background(0, y)
        append_background(width - 1, y)
    cursor = 0
    while cursor < len(queue):
        index = queue[cursor]
        cursor += 1
        x = index % width
        y = index // width
        if x > 0:
            append_background(x - 1, y)
        if x + 1 < width:
            append_background(x + 1, y)
        if y > 0:
            append_background(x, y - 1)
        if y + 1 < height:
            append_background(x, y + 1)

    holes = 0
    for start in range(width * height):
        if visited[start] or bits & (1 << start):
            continue
        holes += 1
        visited[start] = 1
        queue = [start]
        cursor = 0
        while cursor < len(queue):
            index = queue[cursor]
            cursor += 1
            x = index % width
            y = index // width
            neighbours = []
            if x > 0:
                neighbours.append(index - 1)
            if x + 1 < width:
                neighbours.append(index + 1)
            if y > 0:
                neighbours.append(index - width)
            if y + 1 < height:
                neighbours.append(index + width)
            for neighbour in neighbours:
                if visited[neighbour] or bits & (1 << neighbour):
                    continue
                visited[neighbour] = 1
                queue.append(neighbour)
    return holes


def _pattern_template(pattern):
    points = []
    source_height = len(pattern)
    source_width = len(pattern[0])
    for y, row in enumerate(pattern):
        for x, value in enumerate(row):
            if value == "1":
                points.append((x, y))
    bits, aspect = _normalise_points(
        points,
        source_width,
        source_height,
        TEMPLATE_WIDTH,
        TEMPLATE_HEIGHT,
    )
    return {
        "bits": bits,
        "dilated": _dilate(bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT),
        "count": _popcount(bits),
        "aspect": aspect,
        "holes": _hole_count(bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT),
    }


_TEMPLATES = {}
for _digit, _variants in _DIGIT_PATTERNS.items():
    _TEMPLATES[_digit] = tuple(_pattern_template(item) for item in _variants)


class DigitTemplateRecognizer:
    def __init__(
        self,
        sample_width=32,
        sample_height=32,
        minimum_score=0.42,
        minimum_margin=0.025,
    ):
        self.sample_width = int(sample_width)
        self.sample_height = int(sample_height)
        self.minimum_score = float(minimum_score)
        self.minimum_margin = float(minimum_margin)

    def _white_components(self, mask, square):
        width = self.sample_width
        height = self.sample_height
        side = square["side_cm"]
        margin = side * 0.13
        usable = side - 2.0 * margin
        cosine = math.cos(square["angle_rad"])
        sine = math.sin(square["angle_rad"])
        white = bytearray(width * height)
        for row in range(height):
            local_y = (
                square["y0"] + margin + (row + 0.5) * usable / height
            )
            for column in range(width):
                local_x = (
                    square["x0"]
                    + margin
                    + (column + 0.5) * usable / width
                )
                plane_x = cosine * local_x - sine * local_y
                plane_y = sine * local_x + cosine * local_y
                if not mask.is_black_plane(plane_x, plane_y):
                    white[row * width + column] = 1

        visited = bytearray(width * height)
        components = []
        for start in range(width * height):
            if not white[start] or visited[start]:
                continue
            visited[start] = 1
            queue = [start]
            cursor = 0
            points = []
            touches_border = False
            while cursor < len(queue):
                index = queue[cursor]
                cursor += 1
                x = index % width
                y = index // width
                points.append((x, y))
                if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                    touches_border = True
                neighbours = []
                if x > 0:
                    neighbours.append(index - 1)
                if x + 1 < width:
                    neighbours.append(index + 1)
                if y > 0:
                    neighbours.append(index - width)
                if y + 1 < height:
                    neighbours.append(index + width)
                for neighbour in neighbours:
                    if white[neighbour] and not visited[neighbour]:
                        visited[neighbour] = 1
                        queue.append(neighbour)
            components.append((points, touches_border))
        return components

    def _extract_glyph(self, mask, square):
        components = self._white_components(mask, square)
        internal = []
        for points, touches_border in components:
            if touches_border:
                continue
            if len(points) >= 3:
                internal.append(points)
        best = None
        if internal:
            largest_size = max(len(points) for points in internal)
            best = []
            # Some printed fonts and low-resolution resampling split a glyph
            # into two or three nearby stroke components.  Everything inside
            # the black square that is not border-connected belongs to the
            # white digit; discard only tiny isolated highlights.
            for points in internal:
                if len(points) >= max(3, int(largest_size * 0.05)):
                    best.extend(points)
        if not best:
            # A one-pixel square-location error can connect the paper to the
            # glyph.  Keep a conservative fallback, but the low score/margin
            # gates still prevent it from becoming an accepted digit.
            for points, _ in components:
                if best is None or len(points) > len(best):
                    best = points
        if best is None or len(best) < 6:
            return None
        if len(best) > self.sample_width * self.sample_height * 0.55:
            return None
        bits, aspect = _normalise_points(
            best,
            self.sample_width,
            self.sample_height,
            TEMPLATE_WIDTH,
            TEMPLATE_HEIGHT,
        )
        rotated_points = []
        for x, y in best:
            rotated_points.append((
                self.sample_width - 1 - x,
                self.sample_height - 1 - y,
            ))
        rotated_bits, rotated_aspect = _normalise_points(
            rotated_points,
            self.sample_width,
            self.sample_height,
            TEMPLATE_WIDTH,
            TEMPLATE_HEIGHT,
        )
        return {
            "bits": bits,
            "dilated": _dilate(bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT),
            "count": _popcount(bits),
            "aspect": aspect,
            "holes": _hole_count(bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT),
            "rotated_bits": rotated_bits,
            "rotated_dilated": _dilate(
                rotated_bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT
            ),
            "rotated_count": _popcount(rotated_bits),
            "rotated_aspect": rotated_aspect,
            "rotated_holes": _hole_count(
                rotated_bits, TEMPLATE_WIDTH, TEMPLATE_HEIGHT
            ),
            "source_pixels": len(best),
        }

    @staticmethod
    def _match(glyph, template):
        glyph_count = max(glyph["count"], 1)
        template_count = max(template["count"], 1)
        glyph_to_template = _popcount(
            glyph["bits"] & template["dilated"]
        ) / float(glyph_count)
        template_to_glyph = _popcount(
            template["bits"] & glyph["dilated"]
        ) / float(template_count)
        overlap = (
            2.0
            * glyph_to_template
            * template_to_glyph
            / max(glyph_to_template + template_to_glyph, 1.0e-9)
        )
        aspect_delta = abs(
            math.log(max(glyph["aspect"], 0.05) / max(template["aspect"], 0.05))
        )
        aspect_score = math.exp(-1.35 * aspect_delta)
        hole_delta = abs(glyph["holes"] - template["holes"])
        hole_score = 1.0 if hole_delta == 0 else (0.70 if hole_delta == 1 else 0.35)
        return 0.76 * overlap + 0.16 * aspect_score + 0.08 * hole_score

    def recognize(self, mask, square):
        glyph = self._extract_glyph(mask, square)
        result = {
            "valid": False,
            "digit": None,
            "confidence": 0.0,
            "margin": 0.0,
            "second_digit": None,
            "second_score": 0.0,
            "rotation_deg": None,
            "reject_reason": "NO_GLYPH",
        }
        if glyph is None:
            return result

        glyph_variants = (
            (0, glyph),
            (180, {
                "bits": glyph["rotated_bits"],
                "dilated": glyph["rotated_dilated"],
                "count": glyph["rotated_count"],
                "aspect": glyph["rotated_aspect"],
                "holes": glyph["rotated_holes"],
            }),
        )
        scores = []
        for digit in range(10):
            best_variant = 0.0
            best_rotation = 0
            for rotation, glyph_variant in glyph_variants:
                variant_score = 0.0
                for template in _TEMPLATES[digit]:
                    variant_score = max(
                        variant_score,
                        self._match(glyph_variant, template),
                    )
                # The official digit 1 remains markedly narrower than every
                # other glyph, in either accepted camera orientation.
                if glyph_variant["aspect"] < 0.50:
                    if digit == 1:
                        variant_score = min(1.0, variant_score + 0.12)
                    else:
                        variant_score *= 0.94
                elif digit == 1 and glyph_variant["aspect"] > 0.58:
                    variant_score *= 0.88
                if variant_score > best_variant:
                    best_variant = variant_score
                    best_rotation = rotation
            scores.append((best_variant, digit, best_rotation))
        scores.sort(reverse=True)
        best_score, best_digit, best_rotation = scores[0]
        second_score, second_digit, _ = scores[1]
        margin = best_score - second_score
        result.update({
            "digit": best_digit,
            "confidence": best_score,
            "margin": margin,
            "second_digit": second_digit,
            "second_score": second_score,
            "rotation_deg": best_rotation,
            "glyph_aspect": glyph["aspect"],
            "glyph_holes": glyph["holes"],
            "glyph_pixels": glyph["source_pixels"],
            # Keep the complete compact score vector for the target-aware
            # numbered-task gate.  The generic classifier still reports the
            # top digit above, but a printed 3 can occasionally rank just
            # below a visually similar unsupported glyph (often 5) after a
            # low-resolution/projective resampling.  The caller may then
            # compare the user-requested digit directly instead of throwing
            # away this useful evidence.
            "digit_scores": tuple(
                (digit, score) for score, digit, _ in scores
            ),
        })
        if best_score < self.minimum_score:
            result["reject_reason"] = "WEAK_DIGIT_SCORE"
            return result
        if margin < self.minimum_margin:
            result["reject_reason"] = "AMBIGUOUS_DIGIT"
            return result
        result["valid"] = True
        result["reject_reason"] = "OK"
        return result

    def recognize_all(self, mask, squares):
        results = []
        for square in squares:
            digit_result = self.recognize(mask, square)
            square["digit_result"] = digit_result
            square["digit"] = digit_result.get("digit")
            square["digit_confidence"] = digit_result.get("confidence", 0.0)
            square["digit_margin"] = digit_result.get("margin", 0.0)
            square["digit_rotation_deg"] = digit_result.get(
                "rotation_deg"
            )
            results.append(digit_result)
        return tuple(results)
