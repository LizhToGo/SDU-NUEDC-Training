"""Recover one or more filled squares, including partially overlapping ones.

The input is a perspective-corrected :class:`PlaneBinaryMask`; consequently
every coordinate in this module is already measured in centimetres.  The
detector does not depend on connected components because overlapping squares
merge into one component.  Instead it:

1. estimates square orientation families from boundary gradients;
2. clusters collinear boundary evidence;
3. forms square hypotheses from opposite-line pairs and visible L corners;
4. scores each hypothesis using interior fill and visible black/white edges;
5. chooses a small set whose union best explains the observed black mask.

The implementation intentionally avoids NumPy/OpenCV and runs unchanged under
CanMV MicroPython.
"""

import math
import time


MIN_SQUARE_SIDE_CM = 5.0
MAX_SQUARE_SIDE_CM = 14.5


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


def _clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _distance(first, second):
    dx = first[0] - second[0]
    dy = first[1] - second[1]
    return math.sqrt(dx * dx + dy * dy)


def _angle_mod_pi(angle):
    while angle < 0.0:
        angle += math.pi
    while angle >= math.pi:
        angle -= math.pi
    return angle


def _family_angle(angle):
    angle = _angle_mod_pi(angle)
    half_pi = math.pi * 0.5
    while angle >= half_pi:
        angle -= half_pi
    return angle


def _angle_distance_pi(first, second):
    difference = abs(_angle_mod_pi(first) - _angle_mod_pi(second))
    if difference > math.pi * 0.5:
        difference = math.pi - difference
    return abs(difference)


def _family_distance(first, second):
    half_pi = math.pi * 0.5
    difference = abs(_family_angle(first) - _family_angle(second))
    return min(difference, half_pi - difference)


def _popcount(value):
    try:
        return value.bit_count()
    except AttributeError:
        count = 0
        while value:
            value &= value - 1
            count += 1
        return count


def _single_bit_index(value):
    """Return the index of a one-hot integer without CPython-only helpers."""
    index = 0
    while value > 1:
        value >>= 1
        index += 1
    return index


def _subset_sum_table(histogram):
    """Return sums for every subset of an 8-bit membership mask."""
    values = list(histogram)
    size = len(values)
    bit = 1
    while bit < size:
        block = bit << 1
        base = 0
        while base < size:
            for offset in range(bit):
                values[base + bit + offset] += values[base + offset]
            base += block
        bit <<= 1
    return values


def _local_to_plane(x_value, y_value, cosine, sine):
    # e=(cos,sin), n=(-sin,cos), therefore p=e*x+n*y.
    return (
        cosine * x_value - sine * y_value,
        sine * x_value + cosine * y_value,
    )


def _candidate_corners(x0, y0, side, cosine, sine):
    return (
        _local_to_plane(x0, y0, cosine, sine),
        _local_to_plane(x0 + side, y0, cosine, sine),
        _local_to_plane(x0 + side, y0 + side, cosine, sine),
        _local_to_plane(x0, y0 + side, cosine, sine),
    )


class CompositeSquareDetector:
    """Detect and jointly explain up to ``max_squares`` filled squares."""

    def __init__(
        self,
        min_side_cm=MIN_SQUARE_SIDE_CM,
        max_side_cm=MAX_SQUARE_SIDE_CM,
        max_squares=6,
        max_candidates=12,
        orientation_bins=30,
        max_raw_candidates_per_orientation=160,
        max_selection_candidates=8,
    ):
        self.min_side_cm = float(min_side_cm)
        self.max_side_cm = float(max_side_cm)
        self.max_squares = int(max_squares)
        self.max_candidates = int(max_candidates)
        self.orientation_bins = int(orientation_bins)
        self.max_raw_candidates_per_orientation = int(
            max_raw_candidates_per_orientation
        )
        self.max_selection_candidates = max(
            1, int(max_selection_candidates)
        )
        self.last_selection_stats = {}
        self.last_raw_stats = {}

    def _boundary_points(self, mask):
        points = []
        width = mask.width
        height = mask.height
        source = mask.black
        scale = mask.pixels_per_cm
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                index = row + x
                if not source[index]:
                    continue
                if (
                    source[index - 1]
                    and source[index + 1]
                    and source[index - width]
                    and source[index + width]
                ):
                    continue

                # A 3x3 Sobel-like gradient is much less sensitive to the
                # one-pixel stair steps produced by projective resampling.
                gradient_x = (
                    source[index - width + 1]
                    + 2 * source[index + 1]
                    + source[index + width + 1]
                    - source[index - width - 1]
                    - 2 * source[index - 1]
                    - source[index + width - 1]
                )
                gradient_y = (
                    source[index + width - 1]
                    + 2 * source[index + width]
                    + source[index + width + 1]
                    - source[index - width - 1]
                    - 2 * source[index - width]
                    - source[index - width + 1]
                )
                if gradient_x == 0 and gradient_y == 0:
                    continue
                tangent = _angle_mod_pi(
                    math.atan2(gradient_y, gradient_x) + math.pi * 0.5
                )
                weight = abs(gradient_x) + abs(gradient_y)
                points.append((
                    (x + 0.5) / scale,
                    (y + 0.5) / scale,
                    tangent,
                    weight,
                ))
        return points

    def _orientation_families(self, boundary_points):
        bin_count = self.orientation_bins
        histogram = [0.0] * bin_count
        half_pi = math.pi * 0.5
        for _, _, tangent, weight in boundary_points:
            family = _family_angle(tangent)
            index = int(family * bin_count / half_pi) % bin_count
            histogram[index] += weight
        smooth = [0.0] * bin_count
        for index in range(bin_count):
            smooth[index] = (
                histogram[(index - 1) % bin_count]
                + 2.0 * histogram[index]
                + histogram[(index + 1) % bin_count]
            )
        ranked = list(range(bin_count))
        ranked.sort(key=lambda item: smooth[item], reverse=True)
        if not ranked or smooth[ranked[0]] <= 0.0:
            return []
        minimum_peak = smooth[ranked[0]] * 0.16
        selected = []
        minimum_separation = max(2, int(round(bin_count * 8.0 / 90.0)))
        for peak in ranked:
            if smooth[peak] < minimum_peak:
                break
            separated = True
            for previous in selected:
                difference = abs(peak - previous)
                difference = min(difference, bin_count - difference)
                if difference < minimum_separation:
                    separated = False
                    break
            if not separated:
                continue
            selected.append(peak)
            # The supplied targets contain the dominant axis-aligned family and
            # at most one independently rotated square.  A third weak family is
            # almost always staircase/noise evidence and multiplies the raw
            # candidate cost on MicroPython.
            if len(selected) >= 2:
                break

        orientations = []
        for peak in selected:
            cosine_sum = 0.0
            sine_sum = 0.0
            for offset in (-1, 0, 1):
                index = (peak + offset) % bin_count
                angle = (index + 0.5) * half_pi / bin_count
                weight = histogram[index]
                cosine_sum += math.cos(4.0 * angle) * weight
                sine_sum += math.sin(4.0 * angle) * weight
            angle = 0.25 * math.atan2(sine_sum, cosine_sum)
            angle = _family_angle(angle)
            if _family_distance(angle, 0.0) <= math.radians(3.0):
                angle = 0.0
            orientations.append(angle)
        return orientations

    def _cluster_lines(self, samples, bin_width=0.18):
        bins = {}
        for normal, tangent, weight in samples:
            key = int(math.floor(normal / bin_width + 0.5))
            group = bins.get(key)
            if group is None:
                group = [0.0, 0.0, []]
                bins[key] = group
            group[0] += normal * weight
            group[1] += weight
            group[2].append(tangent)
        keys = sorted(bins.keys())
        merged = []
        current = None
        previous_key = None
        for key in keys:
            group = bins[key]
            if current is None or key - previous_key > 2:
                if current is not None:
                    merged.append(current)
                current = [group[0], group[1], list(group[2])]
            else:
                current[0] += group[0]
                current[1] += group[1]
                current[2].extend(group[2])
            previous_key = key
        if current is not None:
            merged.append(current)

        lines = []
        for weighted_normal, total_weight, tangents in merged:
            if len(tangents) < 3 or total_weight <= 0.0:
                continue
            tangents.sort()
            intervals = []
            start = tangents[0]
            last = tangents[0]
            count = 1
            for value in tangents[1:]:
                if value - last > 0.48:
                    if count >= 2:
                        intervals.append((start, last, count))
                    start = value
                    count = 1
                else:
                    count += 1
                last = value
            if count >= 2:
                intervals.append((start, last, count))
            if not intervals:
                continue
            longest = 0.0
            for start, end, _ in intervals:
                longest = max(longest, end - start)
            if longest < 0.45:
                continue
            lines.append({
                "normal": weighted_normal / total_weight,
                "intervals": tuple(intervals),
                "longest": longest,
                "support": len(tangents),
            })
        lines.sort(key=lambda item: item["normal"])
        return lines

    def _lines_for_orientation(self, boundary_points, angle):
        cosine = math.cos(angle)
        sine = math.sin(angle)
        tolerance = math.radians(13.0)
        x_lines = []  # lines parallel to n; constant local x
        y_lines = []  # lines parallel to e; constant local y
        perpendicular = angle + math.pi * 0.5
        for x_cm, y_cm, tangent, weight in boundary_points:
            local_x = cosine * x_cm + sine * y_cm
            local_y = -sine * x_cm + cosine * y_cm
            if _angle_distance_pi(tangent, angle) <= tolerance:
                y_lines.append((local_y, local_x, weight))
            if _angle_distance_pi(tangent, perpendicular) <= tolerance:
                x_lines.append((local_x, local_y, weight))
        return (
            self._cluster_lines(x_lines),
            self._cluster_lines(y_lines),
        )

    @staticmethod
    def _line_covers(line, tangent, margin=0.45):
        for start, end, _ in line["intervals"]:
            if tangent >= start - margin and tangent <= end + margin:
                return True
        return False

    def _side_values(self, x_lines, y_lines):
        values = []
        for lines in (x_lines, y_lines):
            for first_index in range(len(lines)):
                for second_index in range(first_index + 1, len(lines)):
                    difference = abs(
                        lines[second_index]["normal"]
                        - lines[first_index]["normal"]
                    )
                    if (
                        difference >= self.min_side_cm
                        and difference <= self.max_side_cm
                    ):
                        values.append(difference)
            for line in lines:
                for start, end, count in line["intervals"]:
                    length = end - start
                    if (
                        count >= 4
                        and length >= self.min_side_cm
                        and length <= self.max_side_cm
                    ):
                        values.append(length)
        if not values:
            return []
        values.sort()
        clusters = []
        current = [values[0]]
        for value in values[1:]:
            if value - current[-1] <= 0.28:
                current.append(value)
            else:
                clusters.append(current)
                current = [value]
        clusters.append(current)
        result = []
        for cluster in clusters:
            result.append(sum(cluster) / len(cluster))
        return result[:16]

    def _candidate_key(self, center_x, center_y, side, angle):
        return (
            int(round(center_x / 0.20)),
            int(round(center_y / 0.20)),
            int(round(side / 0.20)),
            int(round(_family_angle(angle) / math.radians(3.0))),
        )

    @staticmethod
    def _proposal_line_quality(lines, side, side_mismatch=0.0):
        """Cheaply rank geometric proposals before pixel-level scoring.

        A four-line hypothesis used to receive the same score as every other
        four-line combination.  With several squares this made the fixed raw
        candidate cap depend on loop order.  Boundary support and visible line
        length are inexpensive and give real square sides priority without
        touching the image again.
        """
        supports = [line.get("support", 0) for line in lines]
        lengths = []
        safe_side = max(float(side), 0.1)
        for line in lines:
            lengths.append(min(1.25, line.get("longest", 0.0) / safe_side))
        return (
            4.0 * min(supports)
            + sum(supports)
            + 80.0 * min(lengths)
            + 20.0 * sum(lengths)
            - 200.0 * max(0.0, side_mismatch)
        )

    def _take_diverse_proposals(self, candidates, limit):
        """Keep high-quality proposals plus spatial/size representatives."""
        if limit <= 0 or not candidates:
            return []
        candidates.sort(
            key=lambda item: item.get("_proposal_quality", 0.0),
            reverse=True,
        )
        if len(candidates) <= limit:
            return candidates

        # Most of the budget follows cheap line quality.  The remainder is
        # filled round-robin from centre/side buckets so a valid square in the
        # lower/right part of a multi-target sheet cannot be removed merely by
        # deterministic proposal-generation order.
        quality_count = max(1, (limit * 2) // 3)
        selected = list(candidates[:quality_count])
        buckets = {}
        bucket_order = []
        for candidate in candidates[quality_count:]:
            center = candidate.get("_proposal_center", (0.0, 0.0))
            key = (
                int(center[0] / 3.0),
                int(center[1] / 3.0),
                int((candidate["side_cm"] - self.min_side_cm) / 1.25),
            )
            bucket = buckets.get(key)
            if bucket is None:
                bucket = []
                buckets[key] = bucket
                bucket_order.append(key)
            bucket.append(candidate)

        depth = 0
        while len(selected) < limit:
            added = False
            for key in bucket_order:
                bucket = buckets[key]
                if depth < len(bucket):
                    selected.append(bucket[depth])
                    added = True
                    if len(selected) >= limit:
                        break
            if not added:
                break
            depth += 1
        return selected

    def _limit_raw_candidates(self, candidates):
        """Reserve proposal capacity for complete and partly hidden squares."""
        limit = self.max_raw_candidates_per_orientation
        if len(candidates) <= limit:
            candidates.sort(
                key=lambda item: item.get("_proposal_quality", 0.0),
                reverse=True,
            )
            return candidates

        complete = []
        partial = []
        for candidate in candidates:
            if candidate.get("_proposal_complete", False):
                complete.append(candidate)
            else:
                partial.append(candidate)

        complete_limit = min(len(complete), (limit * 2) // 3)
        partial_limit = min(len(partial), limit - complete_limit)
        if complete_limit + partial_limit < limit:
            complete_limit = min(len(complete), limit - partial_limit)
        if complete_limit + partial_limit < limit:
            partial_limit = min(len(partial), limit - complete_limit)

        selected = self._take_diverse_proposals(complete, complete_limit)
        selected.extend(
            self._take_diverse_proposals(partial, partial_limit)
        )
        selected.sort(
            key=lambda item: item.get("_proposal_quality", 0.0),
            reverse=True,
        )
        return selected

    def _raw_candidates(
        self,
        x_lines,
        y_lines,
        angle,
        plane_width=None,
        plane_height=None,
    ):
        side_values = self._side_values(x_lines, y_lines)
        if not side_values:
            return []
        raw = {}
        cosine = math.cos(angle)
        sine = math.sin(angle)

        def add(
            x0,
            y0,
            side,
            evidence,
            proposal_quality,
            proposal_complete,
        ):
            if side < self.min_side_cm or side > self.max_side_cm:
                return
            center = _local_to_plane(
                x0 + side * 0.5,
                y0 + side * 0.5,
                cosine,
                sine,
            )
            if plane_width is not None and plane_height is not None:
                corners = _candidate_corners(
                    x0, y0, side, cosine, sine
                )
                for corner_x, corner_y in corners:
                    if (
                        corner_x < -0.15
                        or corner_y < -0.15
                        or corner_x > plane_width + 0.15
                        or corner_y > plane_height + 0.15
                    ):
                        return
            key = self._candidate_key(center[0], center[1], side, angle)
            previous = raw.get(key)
            candidate = {
                "x0": x0,
                "y0": y0,
                "side_cm": side,
                "angle_rad": _family_angle(angle),
                "line_evidence": evidence,
                "_proposal_quality": proposal_quality,
                "_proposal_complete": proposal_complete,
                "_proposal_center": center,
            }
            previous_rank = None
            if previous is not None:
                previous_rank = (
                    1 if previous.get("_proposal_complete", False) else 0,
                    previous.get("_proposal_quality", 0.0),
                )
            candidate_rank = (
                1 if proposal_complete else 0,
                proposal_quality,
            )
            if previous is None or candidate_rank > previous_rank:
                raw[key] = candidate

        # Four independently observed bounds are the strongest hypothesis.
        for first_x_index in range(len(x_lines)):
            for second_x_index in range(first_x_index + 1, len(x_lines)):
                x0 = x_lines[first_x_index]["normal"]
                x1 = x_lines[second_x_index]["normal"]
                side_x = x1 - x0
                if side_x < self.min_side_cm or side_x > self.max_side_cm:
                    continue
                for first_y_index in range(len(y_lines)):
                    for second_y_index in range(first_y_index + 1, len(y_lines)):
                        y0 = y_lines[first_y_index]["normal"]
                        y1 = y_lines[second_y_index]["normal"]
                        side_y = y1 - y0
                        side_mean = (side_x + side_y) * 0.5
                        # Short visible pieces can move a clustered support line
                        # by several mask pixels.  Keep a loose *proposal* gate;
                        # fill/edge checks and the global union explanation make
                        # the final decision.
                        if abs(side_x - side_y) > max(
                            0.48, side_mean * 0.04
                        ):
                            continue
                        side = side_mean
                        proposal_quality = self._proposal_line_quality(
                            (
                                x_lines[first_x_index],
                                x_lines[second_x_index],
                                y_lines[first_y_index],
                                y_lines[second_y_index],
                            ),
                            side,
                            abs(side_x - side_y) / max(side, 0.1),
                        )
                        add(
                            x0,
                            y0,
                            side,
                            1.0,
                            proposal_quality,
                            True,
                        )

        # A partially hidden square usually still exposes an L corner.  Pair
        # that corner with every well-supported side value and let the pixel
        # score decide which direction and size explain the union.
        for x_line in x_lines:
            x_value = x_line["normal"]
            for y_line in y_lines:
                y_value = y_line["normal"]
                if not self._line_covers(x_line, y_value):
                    continue
                if not self._line_covers(y_line, x_value):
                    continue
                support = min(
                    1.0,
                    (x_line["support"] + y_line["support"]) / 40.0,
                )
                for side in side_values:
                    proposal_quality = self._proposal_line_quality(
                        (x_line, y_line), side
                    )
                    evidence = 0.55 + 0.20 * support
                    add(
                        x_value,
                        y_value,
                        side,
                        evidence,
                        proposal_quality,
                        False,
                    )
                    add(
                        x_value - side,
                        y_value,
                        side,
                        evidence,
                        proposal_quality,
                        False,
                    )
                    add(
                        x_value,
                        y_value - side,
                        side,
                        evidence,
                        proposal_quality,
                        False,
                    )
                    add(
                        x_value - side,
                        y_value - side,
                        side,
                        evidence,
                        proposal_quality,
                        False,
                    )
        candidates = list(raw.values())
        complete_count = 0
        for candidate in candidates:
            if candidate.get("_proposal_complete", False):
                complete_count += 1
        selected = self._limit_raw_candidates(candidates)
        self.last_raw_stats = {
            "generated_count": len(candidates),
            "complete_count": complete_count,
            "partial_count": len(candidates) - complete_count,
            "selected_count": len(selected),
        }
        return selected

    @staticmethod
    def _sample_local(mask, local_x, local_y, cosine, sine):
        point = _local_to_plane(local_x, local_y, cosine, sine)
        return mask.is_black_plane(point[0], point[1])

    def _score_candidate(self, mask, candidate):
        x0 = candidate["x0"]
        y0 = candidate["y0"]
        side = candidate["side_cm"]
        angle = candidate["angle_rad"]
        cosine = math.cos(angle)
        sine = math.sin(angle)
        corners = _candidate_corners(x0, y0, side, cosine, sine)
        plane_width = mask.plane_width_cm()
        plane_height = mask.plane_height_cm()
        for x_cm, y_cm in corners:
            if (
                x_cm < -0.15
                or y_cm < -0.15
                or x_cm > plane_width + 0.15
                or y_cm > plane_height + 0.15
            ):
                return None

        # Candidate scoring used to sample roughly one point every 0.36 cm,
        # which means more than one thousand Python calls for a large square.
        # Nine-by-nine stratified samples are sufficient after the mask has
        # already been rectified and denoised at 8 px/cm.
        margin = min(0.32, max(0.14, side * 0.035))
        usable = max(0.1, side - 2.0 * margin)
        interior_steps = 9
        filled = 0
        total = interior_steps * interior_steps
        for row in range(interior_steps):
            local_y = y0 + margin + (row + 0.5) * usable / interior_steps
            for column in range(interior_steps):
                local_x = (
                    x0 + margin + (column + 0.5) * usable / interior_steps
                )
                if self._sample_local(
                    mask, local_x, local_y, cosine, sine
                ):
                    filled += 1
        fill_ratio = filled / float(max(total, 1))
        # Numbered targets contain white digit strokes inside their squares, so
        # this gate must be looser than the solid-square-only desktop lab.
        if fill_ratio < 0.70:
            return None

        edge_steps = max(18, min(32, int(side / 0.32)))
        offset = max(0.18, 1.5 / mask.pixels_per_cm)
        side_supports = []
        occluded_supports = []
        inside_ratios = []
        for side_index in range(4):
            supported = 0
            occluded = 0
            inside_count = 0
            for step_index in range(edge_steps):
                position = (step_index + 0.5) * side / edge_steps
                if side_index == 0:  # top
                    edge_x, edge_y = x0 + position, y0
                    inside_x, inside_y = edge_x, edge_y + offset
                    outside_x, outside_y = edge_x, edge_y - offset
                elif side_index == 1:  # right
                    edge_x, edge_y = x0 + side, y0 + position
                    inside_x, inside_y = edge_x - offset, edge_y
                    outside_x, outside_y = edge_x + offset, edge_y
                elif side_index == 2:  # bottom
                    edge_x, edge_y = x0 + position, y0 + side
                    inside_x, inside_y = edge_x, edge_y - offset
                    outside_x, outside_y = edge_x, edge_y + offset
                else:  # left
                    edge_x, edge_y = x0, y0 + position
                    inside_x, inside_y = edge_x + offset, edge_y
                    outside_x, outside_y = edge_x - offset, edge_y
                inside_black = self._sample_local(
                    mask, inside_x, inside_y, cosine, sine
                )
                outside_black = self._sample_local(
                    mask, outside_x, outside_y, cosine, sine
                )
                if inside_black:
                    inside_count += 1
                if inside_black and not outside_black:
                    supported += 1
                elif inside_black and outside_black:
                    occluded += 1
            # A real edge needs black immediately inside as well as white
            # outside; this rejects arbitrary small squares inserted into a
            # large solid union.
            support = supported / float(edge_steps)
            inside_ratio = inside_count / float(edge_steps)
            side_supports.append(support * min(1.0, inside_ratio / 0.65))
            occluded_supports.append(occluded / float(edge_steps))
            inside_ratios.append(inside_ratio)

        visible_total = sum(side_supports)
        edge_support = visible_total / 4.0
        visible_sides = 0
        strong_sides = 0
        for support in side_supports:
            if support >= 0.07:
                visible_sides += 1
            if support >= 0.50:
                strong_sides += 1

        # Count only genuinely exposed convex corners.  Three outside probes
        # prevent an arbitrary L-shaped step in the union from masquerading as
        # the corner of a square.
        exposed_corners = 0
        corner_local = (
            (x0, y0, -1.0, -1.0),
            (x0 + side, y0, 1.0, -1.0),
            (x0 + side, y0 + side, 1.0, 1.0),
            (x0, y0 + side, -1.0, 1.0),
        )
        for corner_x, corner_y, sign_x, sign_y in corner_local:
            inside_black = self._sample_local(
                mask,
                corner_x - sign_x * offset,
                corner_y - sign_y * offset,
                cosine,
                sine,
            )
            outside_diagonal = self._sample_local(
                mask,
                corner_x + sign_x * offset,
                corner_y + sign_y * offset,
                cosine,
                sine,
            )
            outside_x = self._sample_local(
                mask,
                corner_x + sign_x * offset,
                corner_y - sign_y * offset * 0.35,
                cosine,
                sine,
            )
            outside_y = self._sample_local(
                mask,
                corner_x - sign_x * offset * 0.35,
                corner_y + sign_y * offset,
                cosine,
                sine,
            )
            if (
                inside_black
                and not outside_diagonal
                and not outside_x
                and not outside_y
            ):
                exposed_corners += 1

        axis_first = max(side_supports[0], side_supports[2])
        axis_second = max(side_supports[1], side_supports[3])
        strong_two_edge = (
            visible_total >= 1.75
            and exposed_corners >= 2
            and fill_ratio >= 0.88
        )
        if axis_first < 0.07 or axis_second < 0.07:
            return None
        if visible_total < 0.85:
            return None
        if visible_sides < 3 and not strong_two_edge:
            return None

        # Reject the classic false hypothesis obtained by placing a smaller
        # square in the corner of a larger solid square.  Its two apparent
        # sides continue as black/white boundaries beyond the proposed far
        # endpoints, whereas a real (possibly overlapped) side terminates or is
        # occluded there.
        continuation_count = 0
        continuation_tests = 0
        extension = 0.38
        for side_index in range(4):
            if side_supports[side_index] < 0.07:
                continue
            for position in (-extension, side + extension):
                if side_index == 0:
                    edge_x, edge_y = x0 + position, y0
                    inside_x, inside_y = edge_x, edge_y + offset
                    outside_x, outside_y = edge_x, edge_y - offset
                elif side_index == 1:
                    edge_x, edge_y = x0 + side, y0 + position
                    inside_x, inside_y = edge_x - offset, edge_y
                    outside_x, outside_y = edge_x + offset, edge_y
                elif side_index == 2:
                    edge_x, edge_y = x0 + position, y0 + side
                    inside_x, inside_y = edge_x, edge_y - offset
                    outside_x, outside_y = edge_x, edge_y + offset
                else:
                    edge_x, edge_y = x0, y0 + position
                    inside_x, inside_y = edge_x + offset, edge_y
                    outside_x, outside_y = edge_x - offset, edge_y
                continuation_tests += 1
                if self._sample_local(
                    mask, inside_x, inside_y, cosine, sine
                ) and not self._sample_local(
                    mask, outside_x, outside_y, cosine, sine
                ):
                    continuation_count += 1
        continuation_ratio = continuation_count / float(
            max(continuation_tests, 1)
        )
        corner_support = exposed_corners / 4.0
        visible_score = min(1.0, visible_total / 2.0)
        strong_score = min(1.0, strong_sides / 2.0)
        score = (
            0.30 * fill_ratio
            + 0.34 * visible_score
            + 0.20 * min(1.0, exposed_corners / 2.0)
            + 0.06 * strong_score
            + 0.10 * candidate.get("line_evidence", 0.0)
            - 0.05 * continuation_ratio
        )
        center = _local_to_plane(
            x0 + side * 0.5,
            y0 + side * 0.5,
            cosine,
            sine,
        )
        result = dict(candidate)
        result.update({
            "center": center,
            "plane_corners": corners,
            "fill_ratio": fill_ratio,
            "edge_support": edge_support,
            "visible_edge_total": visible_total,
            "side_supports": tuple(side_supports),
            "occluded_supports": tuple(occluded_supports),
            "inside_ratios": tuple(inside_ratios),
            "visible_sides": visible_sides,
            "strong_sides": strong_sides,
            "corner_support": corner_support,
            "exposed_corners": exposed_corners,
            "strong_two_edge": strong_two_edge,
            "edge_continuation_ratio": continuation_ratio,
            "score_samples": (
                total
                + edge_steps * 8
                + 16
                + continuation_tests * 2
            ),
            "score": score,
        })
        return result

    def _deduplicate(self, candidates):
        candidates.sort(key=lambda item: item["score"], reverse=True)
        selected = []
        for candidate in candidates:
            duplicate = False
            for previous in selected:
                if (
                    _distance(candidate["center"], previous["center"]) < 0.55
                    and abs(
                        candidate["side_cm"] - previous["side_cm"]
                    ) < 0.55
                    and _family_distance(
                        candidate["angle_rad"], previous["angle_rad"]
                    ) < math.radians(6.0)
                ):
                    duplicate = True
                    break
            if not duplicate:
                selected.append(candidate)
            if len(selected) >= self.max_candidates:
                break
        return selected

    @staticmethod
    def _point_inside_candidate(point, candidate):
        cosine = math.cos(candidate["angle_rad"])
        sine = math.sin(candidate["angle_rad"])
        local_x = cosine * point[0] + sine * point[1]
        local_y = -sine * point[0] + cosine * point[1]
        return (
            local_x >= candidate["x0"]
            and local_y >= candidate["y0"]
            and local_x <= candidate["x0"] + candidate["side_cm"]
            and local_y <= candidate["y0"] + candidate["side_cm"]
        )

    def _selection_candidates(self, candidates):
        """Keep a small, diverse set before exact subset enumeration."""
        limit = self.max_selection_candidates
        if len(candidates) <= limit:
            return list(candidates)

        by_score = sorted(
            candidates, key=lambda item: item["score"], reverse=True
        )
        by_side = sorted(candidates, key=lambda item: item["side_cm"])
        retained = []

        # Strong evidence protects the obvious complete/mostly complete boxes.
        for candidate in by_score[: max(1, limit // 2)]:
            retained.append(candidate)

        # The judged result is the minimum side, so low-scoring small boxes must
        # not disappear merely because only a short third side is visible.
        for candidate in by_side:
            if candidate not in retained:
                retained.append(candidate)
            if len(retained) >= limit:
                break

        # Normally the loop above fills the list.  Keep this deterministic
        # fallback for custom detector configurations.
        for candidate in by_score:
            if candidate not in retained:
                retained.append(candidate)
            if len(retained) >= limit:
                break
        return retained[:limit]

    def _select_explanation(self, mask, candidates):
        if not candidates:
            self.last_selection_stats = {}
            return [], 0.0, 0.0, 0.0

        candidates = self._selection_candidates(candidates)
        step = 0.55
        columns = max(1, int(mask.plane_width_cm() / step))
        rows = max(1, int(mask.plane_height_cm() / step))
        points = []
        observed_flags = bytearray(columns * rows)
        observed_count = 0
        for row in range(rows):
            y_cm = (row + 0.5) * mask.plane_height_cm() / rows
            for column in range(columns):
                x_cm = (
                    (column + 0.5) * mask.plane_width_cm() / columns
                )
                point = (x_cm, y_cm)
                points.append(point)
                if mask.is_black_plane(x_cm, y_cm):
                    observed_flags[row * columns + column] = 1
                    observed_count += 1
        if observed_count <= 0:
            self.last_selection_stats = {}
            return [], 0.0, 0.0, 0.0

        # Ignore a one-cell uncertainty band around the measured boundary.  The
        # candidate lines come from short visible fragments and can move by a
        # few rectified-mask pixels; core/allowed masks prevent that coarse
        # quantisation from changing the selected square set.
        core_flags = bytearray(columns * rows)
        allowed_flags = bytearray(columns * rows)
        core_count = 0
        for row in range(rows):
            for column in range(columns):
                index = row * columns + column
                if observed_flags[index]:
                    for neighbour_y in range(
                        max(0, row - 1), min(rows, row + 2)
                    ):
                        neighbour_row = neighbour_y * columns
                        for neighbour_x in range(
                            max(0, column - 1),
                            min(columns, column + 2),
                        ):
                            allowed_flags[
                                neighbour_row + neighbour_x
                            ] = 1
                if (
                    column <= 0
                    or row <= 0
                    or column >= columns - 1
                    or row >= rows - 1
                ):
                    continue
                core = True
                for neighbour_y in range(row - 1, row + 2):
                    neighbour_row = neighbour_y * columns
                    for neighbour_x in range(column - 1, column + 2):
                        if not observed_flags[
                            neighbour_row + neighbour_x
                        ]:
                            core = False
                            break
                    if not core:
                        break
                if core:
                    core_flags[index] = 1
                    core_count += 1

        # Quality gates normally leave 4--8 candidates.  Exact enumeration is
        # then at most 255 subsets: cheaper and more reliable than the old
        # 72-wide beam over as many as twenty candidates.
        subset_count = 1 << len(candidates)
        full_mask = subset_count - 1

        # Each grid point belongs to one of at most 256 candidate-membership
        # classes.  Histograms plus a subset-sum transform produce exactly the
        # same missing/false-positive counts as the old 1380-bit integers, but
        # use only small native integers under CanMV MicroPython.
        memberships = bytearray(len(points))
        candidate_bit = 1
        for candidate in candidates:
            cosine = math.cos(candidate["angle_rad"])
            sine = math.sin(candidate["angle_rad"])
            minimum_x = candidate["x0"]
            minimum_y = candidate["y0"]
            maximum_x = minimum_x + candidate["side_cm"]
            maximum_y = minimum_y + candidate["side_cm"]
            for point_index in range(len(points)):
                point = points[point_index]
                local_x = cosine * point[0] + sine * point[1]
                local_y = -sine * point[0] + cosine * point[1]
                if (
                    local_x >= minimum_x
                    and local_y >= minimum_y
                    and local_x <= maximum_x
                    and local_y <= maximum_y
                ):
                    memberships[point_index] |= candidate_bit
            candidate_bit <<= 1

        all_histogram = [0] * subset_count
        observed_histogram = [0] * subset_count
        core_histogram = [0] * subset_count
        outside_histogram = [0] * subset_count
        outside_count = 0
        for point_index in range(len(points)):
            membership = memberships[point_index]
            all_histogram[membership] += 1
            if observed_flags[point_index]:
                observed_histogram[membership] += 1
            if core_flags[point_index]:
                core_histogram[membership] += 1
            if not allowed_flags[point_index]:
                outside_histogram[membership] += 1
                outside_count += 1

        all_subset_sums = _subset_sum_table(all_histogram)
        observed_subset_sums = _subset_sum_table(observed_histogram)
        core_subset_sums = _subset_sum_table(core_histogram)
        outside_subset_sums = _subset_sum_table(outside_histogram)

        counts = bytearray(subset_count)
        evidence_sums = [0.0] * subset_count
        complexity_penalty = max(1, int(round(core_count * 0.0015)))
        best_key = None
        best_subset = 0
        best_missing = 0
        best_false_positive = 0
        evaluated = 0
        for subset in range(1, subset_count):
            least_bit = subset & -subset
            candidate_index = _single_bit_index(least_bit)
            previous = subset ^ least_bit
            selected_count = counts[previous] + 1
            counts[subset] = selected_count
            if selected_count > self.max_squares:
                continue
            evidence_sums[subset] = (
                evidence_sums[previous]
                + candidates[candidate_index]["score"]
            )
            complement = full_mask ^ subset
            missing = core_subset_sums[complement]
            false_positive = (
                outside_count - outside_subset_sums[complement]
            )
            loss = (
                missing
                + 2 * false_positive
                + complexity_penalty * selected_count
            )
            average_evidence = evidence_sums[subset] / selected_count
            key = (loss, selected_count, -average_evidence)
            evaluated += 1
            if best_key is None or key < best_key:
                best_key = key
                best_subset = subset
                best_missing = missing
                best_false_positive = false_positive

        if best_key is None or best_subset == 0:
            self.last_selection_stats = {}
            return [], 0.0, 0.0, 0.0

        selected = []
        for index in range(len(candidates)):
            if best_subset & (1 << index):
                selected.append(candidates[index])
        selected.sort(key=lambda item: item["side_cm"])
        complement = full_mask ^ best_subset
        true_positive = (
            observed_count - observed_subset_sums[complement]
        )
        predicted = len(points) - all_subset_sums[complement]
        recall = true_positive / float(max(observed_count, 1))
        precision = true_positive / float(max(predicted, 1))
        f1 = (
            2.0 * recall * precision
            / max(recall + precision, 1.0e-9)
        )
        evidence = evidence_sums[best_subset] / max(len(selected), 1)
        explanation = (
            0.60 * f1
            + 0.25 * recall
            + 0.12 * precision
            + 0.03 * evidence
        )
        self.last_selection_stats = {
            "grid_columns": columns,
            "grid_rows": rows,
            "observed_count": observed_count,
            "core_count": core_count,
            "input_candidate_count": len(candidates),
            "subset_count": evaluated,
            "selected_count": len(selected),
            "core_missing": best_missing,
            "false_positive": best_false_positive,
            "complexity_penalty": complexity_penalty,
            "backend": "HISTOGRAM_DP",
        }
        return selected, explanation, recall, precision

    def detect(self, mask, mapper=None):
        total_start = _ticks_ms()
        result = {
            "valid": False,
            "reject_reason": "UNKNOWN",
            "squares": (),
            "candidate_count": 0,
            "orientation_count": 0,
            "orientations_deg": (),
            "explanation_score": 0.0,
            "coverage_recall": 0.0,
            "coverage_precision": 0.0,
            "black_fraction": mask.black_fraction(),
            "threshold": mask.threshold,
            "boundary_count": 0,
            "raw_candidate_count": 0,
            "raw_generated_count": 0,
            "raw_complete_count": 0,
            "raw_partial_count": 0,
            "scored_candidate_count": 0,
            "selection_candidate_count": 0,
            "selection_subset_count": 0,
            "selection_backend": "NONE",
            "score_sample_count": 0,
            "selection_stats": {},
            "timing_ms": {
                "boundary": 0,
                "orientation": 0,
                "candidate": 0,
                "selection": 0,
                "total": 0,
            },
        }
        if mask.black_count < max(20, int(mask.pixels_per_cm * 3.0)):
            result["reject_reason"] = "NO_BLACK_TARGET"
            return result
        if mask.black_fraction() > 0.78:
            result["reject_reason"] = "BLACK_AREA_TOO_LARGE"
            return result

        stage_start = _ticks_ms()
        boundary_points = self._boundary_points(mask)
        result["boundary_count"] = len(boundary_points)
        result["timing_ms"]["boundary"] = _ticks_diff(
            _ticks_ms(), stage_start
        )
        stage_start = _ticks_ms()
        orientations = self._orientation_families(boundary_points)
        result["timing_ms"]["orientation"] = _ticks_diff(
            _ticks_ms(), stage_start
        )
        result["orientation_count"] = len(orientations)
        result["orientations_deg"] = tuple(
            math.degrees(angle) for angle in orientations
        )
        if not orientations:
            result["reject_reason"] = "NO_ORIENTATION"
            return result

        stage_start = _ticks_ms()
        scored = []
        raw_count = 0
        raw_generated_count = 0
        raw_complete_count = 0
        raw_partial_count = 0
        for angle in orientations:
            x_lines, y_lines = self._lines_for_orientation(
                boundary_points, angle
            )
            raw = self._raw_candidates(
                x_lines,
                y_lines,
                angle,
                plane_width=mask.plane_width_cm(),
                plane_height=mask.plane_height_cm(),
            )
            raw_count += len(raw)
            raw_generated_count += self.last_raw_stats.get(
                "generated_count", len(raw)
            )
            raw_complete_count += self.last_raw_stats.get(
                "complete_count", 0
            )
            raw_partial_count += self.last_raw_stats.get(
                "partial_count", 0
            )
            for candidate in raw:
                candidate_result = self._score_candidate(mask, candidate)
                if candidate_result is not None:
                    scored.append(candidate_result)
        result["raw_candidate_count"] = raw_count
        result["raw_generated_count"] = raw_generated_count
        result["raw_complete_count"] = raw_complete_count
        result["raw_partial_count"] = raw_partial_count
        result["scored_candidate_count"] = len(scored)
        result["score_sample_count"] = sum(
            candidate.get("score_samples", 0) for candidate in scored
        )
        candidates = self._deduplicate(scored)
        result["timing_ms"]["candidate"] = _ticks_diff(
            _ticks_ms(), stage_start
        )
        result["candidate_count"] = len(candidates)
        if not candidates:
            result["reject_reason"] = "NO_SQUARE_HYPOTHESIS"
            return result

        stage_start = _ticks_ms()
        selected, explanation, recall, precision = self._select_explanation(
            mask, candidates
        )
        result["timing_ms"]["selection"] = _ticks_diff(
            _ticks_ms(), stage_start
        )
        result["selection_stats"] = dict(self.last_selection_stats)
        result["selection_backend"] = self.last_selection_stats.get(
            "backend", "UNKNOWN"
        )
        result["selection_candidate_count"] = self.last_selection_stats.get(
            "input_candidate_count", 0
        )
        result["selection_subset_count"] = self.last_selection_stats.get(
            "subset_count", 0
        )
        result["explanation_score"] = explanation
        result["coverage_recall"] = recall
        result["coverage_precision"] = precision
        if not selected:
            result["reject_reason"] = "NO_UNION_EXPLANATION"
            return result
        if recall < 0.72 or precision < 0.68:
            result["reject_reason"] = "WEAK_UNION_EXPLANATION"
            result["squares"] = tuple(selected)
            return result

        if mapper is not None:
            for square in selected:
                square["image_corners"] = mapper.plane_points_to_image(
                    square["plane_corners"]
                )
        result["valid"] = True
        result["reject_reason"] = "OK"
        result["squares"] = tuple(selected)
        result["timing_ms"]["total"] = _ticks_diff(
            _ticks_ms(), total_start
        )
        return result
