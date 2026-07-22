#!/usr/bin/env python3
"""Desktop experiment for the NUEDC 2025 C overlapping-square task.

The script deliberately depends only on Pillow and the Python standard library.
It can:

1. generate synthetic unions of filled squares together with ground truth;
2. detect square hypotheses from sparse line families;
3. validate the minimum-side result over many randomized scenes;
4. run the same detector on the rendered target-5/target-6 examples.

This is a desktop proof-of-concept.  It is intentionally kept separate from the
CanMV/MicroPython measurement pipeline until the geometry has been validated.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

from PIL import Image, ImageDraw, ImageFilter


@dataclass(frozen=True)
class Square:
    cx: float
    cy: float
    side: float
    angle_deg: float = 0.0


@dataclass(frozen=True)
class LinePeak:
    rho: float
    support: int


@dataclass
class Candidate:
    cx: float
    cy: float
    side: float
    angle_deg: float
    line_support: float
    interior: float = 0.0
    visible_edges: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    occluded_edges: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    exposed_corners: int = 0
    score: float = 0.0
    accepted: bool = False
    reject: str = "UNSCORED"


@dataclass
class Detection:
    candidates: list[Candidate]
    orientations: list[tuple[float, float]]
    boundary_count: int
    elapsed_ms: float


@dataclass
class Selection:
    candidates: list[Candidate]
    loss: float
    core_missing: int
    false_positive: int
    grid_step: int
    evaluated_subsets: int


def angle_distance_mod90(a: float, b: float) -> float:
    d = abs((a - b) % 90.0)
    return min(d, 90.0 - d)


def square_polygon(square: Square) -> list[tuple[float, float]]:
    c = math.cos(math.radians(square.angle_deg))
    s = math.sin(math.radians(square.angle_deg))
    h = square.side * 0.5
    points = []
    for u, v in ((-h, -h), (h, -h), (h, h), (-h, h)):
        points.append((square.cx + c * u - s * v, square.cy + s * u + c * v))
    return points


def _inside_canvas(square: Square, width: int, height: int, margin: int = 10) -> bool:
    return all(
        margin <= x < width - margin and margin <= y < height - margin
        for x, y in square_polygon(square)
    )


def _point_inside_square(x: float, y: float, square: Square, margin: float = 0.0) -> bool:
    angle = math.radians(square.angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    dx = x - square.cx
    dy = y - square.cy
    u = c * dx + s * dy
    v = -s * dx + c * dy
    half = square.side * 0.5 + margin
    return abs(u) <= half and abs(v) <= half


def _truth_observability(square: Square, others: Sequence[Square]) -> tuple[list[float], int]:
    """Return visible fractions and exposed corners using exact generator geometry.

    A square fully hidden in a same-colour union cannot be recovered by any image
    algorithm.  Random validation therefore labels only scenes satisfying the
    same observability condition that is visibly true in target 5/6.
    """

    angle = math.radians(square.angle_deg)
    n1 = (math.cos(angle), math.sin(angle))
    n2 = (-math.sin(angle), math.cos(angle))
    half = square.side * 0.5
    offset = 2.5

    def xy(u: float, v: float) -> tuple[float, float]:
        return (
            square.cx + u * n1[0] + v * n2[0],
            square.cy + u * n1[1] + v * n2[1],
        )

    visible = []
    samples = 40
    for edge in range(4):
        count = 0
        for k in range(samples):
            t = -half + square.side * (k + 0.5) / samples
            if edge == 0:
                point = xy(-half - offset, t)
            elif edge == 1:
                point = xy(half + offset, t)
            elif edge == 2:
                point = xy(t, -half - offset)
            else:
                point = xy(t, half + offset)
            if not any(_point_inside_square(point[0], point[1], other) for other in others):
                count += 1
        visible.append(count / samples)

    exposed_corners = 0
    for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        probes = (
            xy(su * (half + offset), sv * (half + offset)),
            xy(su * (half + offset), sv * (half - offset * 0.3)),
            xy(su * (half - offset * 0.3), sv * (half + offset)),
        )
        if all(not any(_point_inside_square(x, y, other) for other in others) for x, y in probes):
            exposed_corners += 1
    return visible, exposed_corners


def _scene_is_observable(squares: Sequence[Square]) -> bool:
    for index, square in enumerate(squares):
        others = list(squares[:index]) + list(squares[index + 1 :])
        visible, corners = _truth_observability(square, others)
        if sum(value >= 0.07 for value in visible) < 3:
            return False
        if sum(visible) < 0.55 or corners < 1:
            return False
    return True


def _random_axis_chain(rng: random.Random, width: int, height: int) -> list[Square]:
    """Generate the staircase overlap pattern used by target 5/6."""

    count = 3
    for _ in range(200):
        sides = [rng.uniform(62.0, 112.0) for _ in range(count)]
        # Make the minimum square occur at arbitrary positions in the chain.
        rng.shuffle(sides)
        x = rng.uniform(38.0, 70.0)
        y = rng.uniform(35.0, 65.0)
        squares = [Square(x + sides[0] / 2, y + sides[0] / 2, sides[0], 0.0)]
        for i in range(1, count):
            prev = squares[-1]
            # Partial diagonal overlap: enough hidden edges to exercise reconstruction,
            # while keeping the square observable from at least one exposed corner.
            shift_x = rng.uniform(0.52, 0.76) * min(prev.side, sides[i])
            shift_y = rng.uniform(0.50, 0.74) * min(prev.side, sides[i])
            squares.append(Square(prev.cx + shift_x, prev.cy + shift_y, sides[i], 0.0))
        if all(_inside_canvas(sq, width, height, 12) for sq in squares):
            return squares
    raise RuntimeError("could not place an axis-aligned overlap chain")


def generate_scene(
    seed: int,
    width: int = 420,
    height: int = 420,
    mode: str = "mixed",
    noisy: bool = True,
) -> tuple[Image.Image, list[Square], dict]:
    rng = random.Random(seed)
    chosen_mode = mode
    if mode == "mixed":
        chosen_mode = rng.choice(("axis", "axis", "rotated"))

    squares = []
    for _scene_attempt in range(400):
        chain = _random_axis_chain(rng, width, int(height * 0.66))
        trial = list(chain)

        # The examples contain a fourth, larger square.  It is below the chain;
        # target 5 lets its corner overlap the last axis-aligned square.
        side = rng.uniform(96.0, 138.0)
        angle = rng.uniform(16.0, 38.0) if chosen_mode == "rotated" else 0.0
        placed = None
        chain_bottom = max(y for square in chain for _, y in square_polygon(square))
        for _ in range(120):
            gap = rng.uniform(-0.10, 0.18) * side if chosen_mode == "rotated" else rng.uniform(8.0, 28.0)
            candidate = Square(
                rng.uniform(width * 0.42, width * 0.72),
                chain_bottom + gap + side * 0.5,
                side,
                angle,
            )
            if _inside_canvas(candidate, width, height, 12):
                placed = candidate
                break
        if placed is None:
            continue
        trial.append(placed)
        if _scene_is_observable(trial):
            squares = trial
            break
    if not squares:
        raise RuntimeError("could not generate an observable overlap scene")

    supersample = 3
    image = Image.new("L", (width * supersample, height * supersample), 255)
    draw = ImageDraw.Draw(image)
    for sq in squares:
        scaled = Square(
            sq.cx * supersample,
            sq.cy * supersample,
            sq.side * supersample,
            sq.angle_deg,
        )
        draw.polygon(square_polygon(scaled), fill=0)
    image = image.resize((width, height), Image.Resampling.LANCZOS)

    blur_radius = 0.0
    noise_sigma = 0.0
    threshold = 128
    if noisy:
        blur_radius = rng.uniform(0.0, 0.75)
        noise_sigma = rng.uniform(0.0, 7.0)
        threshold += rng.randint(-12, 12)
        if blur_radius > 0.05:
            image = image.filter(ImageFilter.GaussianBlur(blur_radius))
        if noise_sigma > 0.1:
            pixels = bytearray(image.tobytes())
            for i, value in enumerate(pixels):
                pixels[i] = max(0, min(255, int(value + rng.gauss(0.0, noise_sigma))))
            image = Image.frombytes("L", image.size, bytes(pixels))

    metadata = {
        "seed": seed,
        "mode": chosen_mode,
        "width": width,
        "height": height,
        "threshold": threshold,
        "blur_radius": blur_radius,
        "noise_sigma": noise_sigma,
        "squares": [asdict(sq) for sq in squares],
    }
    return image, squares, metadata


def image_to_mask(image: Image.Image, threshold: int = 128) -> tuple[bytearray, int, int]:
    gray = image.convert("L")
    width, height = gray.size
    return bytearray(1 if value < threshold else 0 for value in gray.tobytes()), width, height


def extract_boundary(mask: bytearray, width: int, height: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    for y in range(1, height - 1):
        row = y * width
        for x in range(1, width - 1):
            i = row + x
            if mask[i] and not (mask[i - 1] and mask[i + 1] and mask[i - width] and mask[i + width]):
                points.append((x, y))
    return points


def _projection_histogram(
    points: Sequence[tuple[int, int]], normal_deg: float
) -> dict[int, int]:
    radians = math.radians(normal_deg)
    c = math.cos(radians)
    s = math.sin(radians)
    hist: dict[int, int] = {}
    for x, y in points:
        rho = int(round(c * x + s * y))
        hist[rho] = hist.get(rho, 0) + 1
    return hist


def _histogram_peaks(
    hist: dict[int, int], min_support: int, max_peaks: int, nms_radius: int = 3
) -> list[LinePeak]:
    ranked = sorted(hist.items(), key=lambda item: item[1], reverse=True)
    peaks: list[LinePeak] = []
    for rho, support in ranked:
        if support < min_support:
            break
        if any(abs(rho - peak.rho) <= nms_radius for peak in peaks):
            continue
        peaks.append(LinePeak(float(rho), support))
        if len(peaks) >= max_peaks:
            break
    return sorted(peaks, key=lambda peak: peak.rho)


def _orientation_strength(hist: dict[int, int], keep: int = 10) -> float:
    # Squaring makes several long, coherent lines beat many short accidental ones.
    peaks = _histogram_peaks(hist, min_support=3, max_peaks=keep, nms_radius=2)
    return sum(float(peak.support * peak.support) for peak in peaks)


def find_orientation_families(
    boundary: Sequence[tuple[int, int]],
    max_families: int = 2,
    angle_step: int = 1,
) -> list[tuple[float, float]]:
    if not boundary:
        return []
    # Bound desktop runtime without changing the line locations materially.
    if len(boundary) > 3200:
        stride = max(1, len(boundary) // 3200)
        sampled = boundary[::stride]
    else:
        sampled = boundary

    scores: list[tuple[float, float]] = []
    for angle in range(0, 90, angle_step):
        h1 = _projection_histogram(sampled, angle)
        h2 = _projection_histogram(sampled, angle + 90.0)
        scores.append((float(angle), _orientation_strength(h1) + _orientation_strength(h2)))
    scores.sort(key=lambda item: item[1], reverse=True)
    if not scores or scores[0][1] <= 0:
        return []

    selected: list[tuple[float, float]] = []
    strongest = scores[0][1]
    for angle, score in scores:
        if score < strongest * 0.16:
            break
        if any(angle_distance_mod90(angle, old_angle) < 7.0 for old_angle, _ in selected):
            continue
        selected.append((angle, score))
        if len(selected) >= max_families:
            break
    return selected


def _pair_lines(
    lines: Sequence[LinePeak], min_side: float, max_side: float
) -> list[tuple[LinePeak, LinePeak, float, float]]:
    pairs = []
    for i, first in enumerate(lines):
        for second in lines[i + 1 :]:
            distance = second.rho - first.rho
            if min_side <= distance <= max_side:
                pairs.append((first, second, distance, first.support + second.support))
    pairs.sort(key=lambda item: item[3], reverse=True)
    return pairs[:100]


def _mask_at(mask: bytearray, width: int, height: int, x: float, y: float) -> int:
    ix = int(round(x))
    iy = int(round(y))
    if ix < 0 or iy < 0 or ix >= width or iy >= height:
        return 0
    return mask[iy * width + ix]


def _evaluate_candidate(candidate: Candidate, mask: bytearray, width: int, height: int) -> None:
    angle = math.radians(candidate.angle_deg)
    n1 = (math.cos(angle), math.sin(angle))
    n2 = (-math.sin(angle), math.cos(angle))
    uc = candidate.cx * n1[0] + candidate.cy * n1[1]
    vc = candidate.cx * n2[0] + candidate.cy * n2[1]
    half = candidate.side * 0.5
    u0, u1 = uc - half, uc + half
    v0, v1 = vc - half, vc + half

    # Filled-square evidence.  An actual square must remain black even where it
    # overlaps another square; this rejects line combinations spanning white gaps.
    inside_hits = 0
    inside_total = 0
    for iu in range(1, 10):
        u = u0 + candidate.side * iu / 10.0
        for iv in range(1, 10):
            v = v0 + candidate.side * iv / 10.0
            x = u * n1[0] + v * n2[0]
            y = u * n1[1] + v * n2[1]
            inside_hits += _mask_at(mask, width, height, x, y)
            inside_total += 1
    candidate.interior = inside_hits / float(max(1, inside_total))

    offset = max(1.5, min(3.0, candidate.side * 0.018))
    sample_count = max(18, min(48, int(candidate.side / 2.5)))
    visible: list[float] = []
    occluded: list[float] = []

    # Edges are ordered u-min, u-max, v-min, v-max.
    for edge_index in range(4):
        vis = 0
        occ = 0
        valid_inside = 0
        for k in range(sample_count):
            t = (k + 0.5) / sample_count
            if edge_index == 0:
                u, v = u0, v0 + t * candidate.side
                ui, vi, uo, vo = u + offset, v, u - offset, v
            elif edge_index == 1:
                u, v = u1, v0 + t * candidate.side
                ui, vi, uo, vo = u - offset, v, u + offset, v
            elif edge_index == 2:
                u, v = u0 + t * candidate.side, v0
                ui, vi, uo, vo = u, v + offset, u, v - offset
            else:
                u, v = u0 + t * candidate.side, v1
                ui, vi, uo, vo = u, v - offset, u, v + offset
            xi = ui * n1[0] + vi * n2[0]
            yi = ui * n1[1] + vi * n2[1]
            xo = uo * n1[0] + vo * n2[0]
            yo = uo * n1[1] + vo * n2[1]
            inner = _mask_at(mask, width, height, xi, yi)
            outer = _mask_at(mask, width, height, xo, yo)
            if inner:
                valid_inside += 1
                if outer:
                    occ += 1
                else:
                    vis += 1
        visible.append(vis / float(sample_count))
        occluded.append(occ / float(sample_count))
    candidate.visible_edges = tuple(visible)  # type: ignore[assignment]
    candidate.occluded_edges = tuple(occluded)  # type: ignore[assignment]

    # A genuine exposed corner has a black inward diagonal and white space in
    # both outward directions.  This prevents arbitrary squares inside a solid
    # black union from passing merely as "fully occluded".
    corners = 0
    corner_offset = max(2.0, min(4.0, candidate.side * 0.025))
    for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        u = uc + su * half
        v = vc + sv * half
        inward_u = u - su * corner_offset
        inward_v = v - sv * corner_offset
        out_u = u + su * corner_offset
        out_v = v + sv * corner_offset
        side_u = (u + su * corner_offset, v - sv * corner_offset * 0.35)
        side_v = (u - su * corner_offset * 0.35, v + sv * corner_offset)

        def uv_value(pair: tuple[float, float]) -> int:
            pu, pv = pair
            return _mask_at(
                mask,
                width,
                height,
                pu * n1[0] + pv * n2[0],
                pu * n1[1] + pv * n2[1],
            )

        if (
            uv_value((inward_u, inward_v))
            and not uv_value((out_u, out_v))
            and not uv_value(side_u)
            and not uv_value(side_v)
        ):
            corners += 1
    candidate.exposed_corners = corners

    visible_total = sum(visible)
    axis_a = max(visible[0], visible[1])
    axis_b = max(visible[2], visible[3])
    # With a same-colour overlap, two adjacent exposed edges alone are
    # geometrically ambiguous: an L-shaped step made by two *different* squares
    # produces exactly the same local evidence.  The supplied targets leave at
    # least a short piece of a third side visible, so require three supporting
    # sides.  This is an observability gate, not merely a tuned score.
    visible_edge_count = sum(1 for value in visible if value >= 0.07)
    candidate.score = (
        0.28 * candidate.interior
        + 0.34 * min(1.0, visible_total / 2.0)
        + 0.23 * min(1.0, corners / 2.0)
        + 0.15 * min(1.0, candidate.line_support)
    )

    if candidate.interior < 0.84:
        candidate.reject = "INTERIOR"
    elif axis_a < 0.10 or axis_b < 0.10:
        candidate.reject = "ONE_DIRECTION_ONLY"
    elif visible_total < 0.85:
        candidate.reject = "TOO_LITTLE_VISIBLE_EDGE"
    elif visible_edge_count < 3 and not (
        visible_total >= 1.75
        and corners >= 2
        and candidate.interior >= 0.95
    ):
        candidate.reject = "TWO_EDGE_AMBIGUITY"
    elif corners < 1 and visible_total < 1.05:
        candidate.reject = "NO_EXPOSED_CORNER"
    elif candidate.score < 0.68:
        candidate.reject = "LOW_SCORE"
    else:
        candidate.accepted = True
        candidate.reject = "OK"


def _deduplicate(candidates: Iterable[Candidate]) -> list[Candidate]:
    kept: list[Candidate] = []
    for candidate in sorted(candidates, key=lambda c: c.score, reverse=True):
        duplicate = False
        for old in kept:
            if (
                math.hypot(candidate.cx - old.cx, candidate.cy - old.cy)
                <= max(3.0, 0.035 * candidate.side)
                and abs(candidate.side - old.side) <= max(3.0, 0.035 * candidate.side)
                and angle_distance_mod90(candidate.angle_deg, old.angle_deg) <= 2.5
            ):
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
    return kept


def detect_squares(
    image: Image.Image,
    threshold: int = 128,
    min_side: float | None = None,
    max_side: float | None = None,
) -> Detection:
    started = time.perf_counter()
    mask, width, height = image_to_mask(image, threshold)
    boundary = extract_boundary(mask, width, height)
    families = find_orientation_families(boundary)
    if min_side is None:
        min_side = min(width, height) * 0.10
    if max_side is None:
        max_side = min(width, height) * 0.72

    raw_candidates: list[Candidate] = []
    min_line_support = max(6, int(min(width, height) * 0.018))
    for angle, family_score in families:
        hist_u = _projection_histogram(boundary, angle)
        hist_v = _projection_histogram(boundary, angle + 90.0)
        lines_u = _histogram_peaks(hist_u, min_line_support, max_peaks=16, nms_radius=2)
        lines_v = _histogram_peaks(hist_v, min_line_support, max_peaks=16, nms_radius=2)
        pairs_u = _pair_lines(lines_u, min_side, max_side)
        pairs_v = _pair_lines(lines_v, min_side, max_side)

        radians = math.radians(angle)
        n1 = (math.cos(radians), math.sin(radians))
        n2 = (-math.sin(radians), math.cos(radians))
        for u0, u1, du, u_support in pairs_u:
            for v0, v1, dv, v_support in pairs_v:
                side = 0.5 * (du + dv)
                # Coarse Hough peaks can move by several pixels when a visible
                # edge is very short or lies next to another square's edge.  A
                # deliberately loose equality gate only proposes the square;
                # the localized edge checks below (and later 1080p refinement)
                # decide whether it is real.
                tolerance = max(4.0, side * 0.07)
                if abs(du - dv) > tolerance:
                    continue
                uc = 0.5 * (u0.rho + u1.rho)
                vc = 0.5 * (v0.rho + v1.rho)
                cx = uc * n1[0] + vc * n2[0]
                cy = uc * n1[1] + vc * n2[1]
                if not (-side * 0.1 <= cx <= width + side * 0.1 and -side * 0.1 <= cy <= height + side * 0.1):
                    continue
                line_quality = (u_support + v_support) / max(1.0, 4.0 * side)
                candidate = Candidate(cx, cy, side, angle, line_quality)
                _evaluate_candidate(candidate, mask, width, height)
                raw_candidates.append(candidate)

    candidates = _deduplicate(raw_candidates)
    # The task asks for the minimum side; retain diagnostics in size order so the
    # first accepted hypothesis is exactly the one the embedded version would refine.
    candidates.sort(key=lambda item: (item.side, -item.score))
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return Detection(candidates, families, len(boundary), elapsed_ms)


def match_candidate(candidate: Candidate, truths: Sequence[Square]) -> tuple[int | None, float]:
    best_index = None
    best_cost = float("inf")
    for index, truth in enumerate(truths):
        side_error = abs(candidate.side - truth.side) / truth.side
        center_error = math.hypot(candidate.cx - truth.cx, candidate.cy - truth.cy) / truth.side
        angle_error = angle_distance_mod90(candidate.angle_deg, truth.angle_deg) / 10.0
        cost = side_error * 3.0 + center_error * 2.0 + angle_error
        if cost < best_cost:
            best_cost = cost
            best_index = index
    if best_index is None:
        return None, best_cost
    truth = truths[best_index]
    if (
        abs(candidate.side - truth.side) / truth.side <= 0.055
        and math.hypot(candidate.cx - truth.cx, candidate.cy - truth.cy) <= max(5.0, truth.side * 0.08)
        and angle_distance_mod90(candidate.angle_deg, truth.angle_deg) <= 4.0
    ):
        return best_index, best_cost
    return None, best_cost


def choose_minimum(detection: Detection) -> Candidate | None:
    return next((candidate for candidate in detection.candidates if candidate.accepted), None)


def _byte_mask_to_bits(values: bytearray) -> int:
    bits = 0
    for index, value in enumerate(values):
        if value:
            bits |= 1 << index
    return bits


def _morphology_band(values: bytearray, width: int, height: int) -> tuple[bytearray, bytearray]:
    """Return a one-cell erosion and dilation of a coarse black mask."""

    eroded = bytearray(width * height)
    dilated = bytearray(width * height)
    for y in range(height):
        for x in range(width):
            index = y * width + x
            if values[index]:
                for ny in range(max(0, y - 1), min(height, y + 2)):
                    row = ny * width
                    for nx in range(max(0, x - 1), min(width, x + 2)):
                        dilated[row + nx] = 1
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                continue
            keep = True
            for ny in range(y - 1, y + 2):
                row = ny * width
                for nx in range(x - 1, x + 2):
                    if not values[row + nx]:
                        keep = False
                        break
                if not keep:
                    break
            if keep:
                eroded[index] = 1
    return eroded, dilated


def _candidate_grid_bits(
    candidate: Candidate,
    image_width: int,
    image_height: int,
    grid_width: int,
    grid_height: int,
    grid_step: int,
) -> int:
    angle = math.radians(candidate.angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    half = candidate.side * 0.5
    radius = candidate.side * math.sqrt(0.5)
    min_gx = max(0, int((candidate.cx - radius) // grid_step) - 1)
    max_gx = min(grid_width - 1, int((candidate.cx + radius) // grid_step) + 1)
    min_gy = max(0, int((candidate.cy - radius) // grid_step) - 1)
    max_gy = min(grid_height - 1, int((candidate.cy + radius) // grid_step) + 1)
    bits = 0
    for gy in range(min_gy, max_gy + 1):
        y = min(image_height - 1, gy * grid_step + grid_step * 0.5)
        row = gy * grid_width
        for gx in range(min_gx, max_gx + 1):
            x = min(image_width - 1, gx * grid_step + grid_step * 0.5)
            dx = x - candidate.cx
            dy = y - candidate.cy
            u = c * dx + s * dy
            v = -s * dx + c * dy
            if abs(u) <= half and abs(v) <= half:
                bits |= 1 << (row + gx)
    return bits


def select_square_explanation(
    image: Image.Image,
    detection: Detection,
    threshold: int = 128,
    grid_step: int = 4,
    max_candidates: int = 14,
) -> Selection:
    """Select the smallest candidate subset explaining the observed black union.

    Local geometry gates intentionally over-generate a few hypotheses.  The
    final image, however, should contain only squares needed to reconstruct the
    observed black target.  Because the quality gate normally leaves 4--8
    candidates, exact subset enumeration is both simpler and more reliable than
    a wide beam search.
    """

    accepted = [candidate for candidate in detection.candidates if candidate.accepted]
    if not accepted:
        return Selection([], float("inf"), 0, 0, grid_step, 0)
    if len(accepted) > max_candidates:
        # Keep strong evidence while preserving both small and large hypotheses.
        by_score = sorted(accepted, key=lambda candidate: candidate.score, reverse=True)
        by_size = sorted(accepted, key=lambda candidate: candidate.side)
        retained: list[Candidate] = []
        for candidate in by_score[: max_candidates // 2] + by_size:
            if candidate not in retained:
                retained.append(candidate)
            if len(retained) >= max_candidates:
                break
        accepted = retained

    gray = image.convert("L")
    width, height = gray.size
    pixels = gray.load()
    grid_width = math.ceil(width / grid_step)
    grid_height = math.ceil(height / grid_step)
    observed = bytearray(grid_width * grid_height)
    for gy in range(grid_height):
        y = min(height - 1, int(gy * grid_step + grid_step * 0.5))
        row = gy * grid_width
        for gx in range(grid_width):
            x = min(width - 1, int(gx * grid_step + grid_step * 0.5))
            observed[row + gx] = 1 if pixels[x, y] < threshold else 0

    core_values, allowed_values = _morphology_band(observed, grid_width, grid_height)
    core_bits = _byte_mask_to_bits(core_values)
    allowed_bits = _byte_mask_to_bits(allowed_values)
    candidate_bits = [
        _candidate_grid_bits(
            candidate,
            width,
            height,
            grid_width,
            grid_height,
            grid_step,
        )
        for candidate in accepted
    ]

    # A small model-description penalty removes candidates whose area is already
    # explained by other squares.  Missing black core costs less than drawing in
    # genuinely white space, because false-positive geometry is stronger evidence.
    core_count = core_bits.bit_count()
    complexity_penalty = max(2, int(round(core_count * 0.0015)))
    subset_count = 1 << len(accepted)
    unions = [0] * subset_count
    best_key = None
    best_mask = 0
    best_missing = 0
    best_false_positive = 0
    for subset in range(1, subset_count):
        least_bit = subset & -subset
        candidate_index = least_bit.bit_length() - 1
        previous = subset ^ least_bit
        union = unions[previous] | candidate_bits[candidate_index]
        unions[subset] = union
        missing = (core_bits & ~union).bit_count()
        false_positive = (union & ~allowed_bits).bit_count()
        selected_count = subset.bit_count()
        loss = missing + 2.0 * false_positive + complexity_penalty * selected_count
        evidence = sum(accepted[index].score for index in range(len(accepted)) if subset & (1 << index))
        key = (loss, selected_count, -evidence)
        if best_key is None or key < best_key:
            best_key = key
            best_mask = subset
            best_missing = missing
            best_false_positive = false_positive

    selected = [accepted[index] for index in range(len(accepted)) if best_mask & (1 << index)]
    selected.sort(key=lambda candidate: candidate.side)
    return Selection(
        selected,
        float(best_key[0]) if best_key is not None else float("inf"),
        best_missing,
        best_false_positive,
        grid_step,
        subset_count - 1,
    )


def draw_overlay(
    image: Image.Image,
    detection: Detection,
    truths: Sequence[Square] = (),
    limit: int = 20,
    show_truth: bool = True,
    show_rejected: bool = True,
    only_minimum: bool = False,
    selected: Sequence[Candidate] | None = None,
    highlight_minimum: bool = False,
) -> Image.Image:
    output = image.convert("RGB")
    draw = ImageDraw.Draw(output)
    if show_truth:
        for truth in truths:
            polygon = square_polygon(truth)
            draw.line(polygon + [polygon[0]], fill=(40, 120, 255), width=2)
    accepted = [candidate for candidate in detection.candidates if candidate.accepted]
    rejected = [candidate for candidate in detection.candidates if not candidate.accepted]
    minimum = choose_minimum(detection)
    if selected is not None:
        shown = list(selected)[:limit]
    elif only_minimum:
        shown = [minimum] if minimum is not None else []
    elif show_rejected:
        shown = accepted[:limit] + rejected[: max(0, limit - len(accepted))]
    else:
        shown = accepted[:limit]
    shown_minimum = min(shown, key=lambda candidate: candidate.side) if shown else None
    for candidate in shown:
        square = Square(candidate.cx, candidate.cy, candidate.side, candidate.angle_deg)
        polygon = square_polygon(square)
        is_minimum = highlight_minimum and candidate is shown_minimum
        color = (255, 40, 40) if is_minimum else ((20, 200, 40) if candidate.accepted else (230, 130, 20))
        draw.line(polygon + [polygon[0]], fill=color, width=3 if is_minimum else 2)
        draw.text(
            polygon[0],
            ("MIN " if is_minimum else "") + f"{candidate.side:.1f}/{candidate.score:.2f}",
            fill=color,
        )
    result_minimum = shown_minimum if selected is not None else minimum
    if result_minimum is not None:
        draw.text((8, 8), f"MIN {result_minimum.side:.2f}px", fill=(220, 0, 0) if highlight_minimum else (0, 150, 0))
    else:
        draw.text((8, 8), "MIN NONE", fill=(220, 0, 0))
    return output


def save_scene(output_dir: Path, seed: int, mode: str, noisy: bool) -> dict:
    image, truths, metadata = generate_scene(seed, mode=mode, noisy=noisy)
    threshold = int(metadata["threshold"])
    detection = detect_squares(image, threshold=threshold)
    selection = select_square_explanation(image, detection, threshold=threshold)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"scene_{seed:04d}_{metadata['mode']}"
    image.save(output_dir / f"{stem}.png")
    draw_overlay(
        image,
        detection,
        show_truth=False,
        show_rejected=False,
        selected=selection.candidates,
        highlight_minimum=True,
    ).save(output_dir / f"{stem}_result.png")
    draw_overlay(
        image,
        detection,
        show_truth=False,
        show_rejected=False,
    ).save(output_dir / f"{stem}_accepted.png")
    draw_overlay(image, detection, truths).save(output_dir / f"{stem}_debug.png")
    payload = dict(metadata)
    payload["detection"] = {
        "elapsed_ms": detection.elapsed_ms,
        "boundary_count": detection.boundary_count,
        "orientations": detection.orientations,
        "candidates": [asdict(candidate) for candidate in detection.candidates],
    }
    payload["selection"] = {
        "loss": selection.loss,
        "core_missing": selection.core_missing,
        "false_positive": selection.false_positive,
        "grid_step": selection.grid_step,
        "evaluated_subsets": selection.evaluated_subsets,
        "candidates": [asdict(candidate) for candidate in selection.candidates],
    }
    (output_dir / f"{stem}.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return payload


def validate_batch(
    output_dir: Path,
    count: int,
    start_seed: int,
    mode: str,
    noisy: bool,
    save_failures: int = 20,
) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    failure_count = 0
    times = []
    detected_truths = 0
    total_truths = 0
    identity_successes = 0
    selected_correct = 0
    selected_total = 0
    exact_explanations = 0
    total_times = []

    for offset in range(count):
        seed = start_seed + offset
        image, truths, metadata = generate_scene(seed, mode=mode, noisy=noisy)
        threshold = int(metadata["threshold"])
        total_started = time.perf_counter()
        detection = detect_squares(image, threshold=threshold)
        selection = select_square_explanation(image, detection, threshold=threshold)
        total_times.append((time.perf_counter() - total_started) * 1000.0)
        times.append(detection.elapsed_ms)
        minimum = selection.candidates[0] if selection.candidates else None
        truth_min_index = min(range(len(truths)), key=lambda index: truths[index].side)
        matched_min_index = None
        min_side_error = None
        if minimum is not None:
            matched_min_index, _ = match_candidate(minimum, truths)
            min_side_error = abs(minimum.side - truths[truth_min_index].side) / truths[truth_min_index].side
        identity_success = (
            minimum is not None
            and matched_min_index == truth_min_index
            and min_side_error is not None
            and min_side_error <= 0.04
        )
        # The judged output is the minimum side length, not the identity of the
        # square providing it.  If two true squares differ by less than the
        # coarse raster error, measuring either one is a correct output.  Still
        # require the selected geometry to match *some* real square so a lucky
        # false hypothesis is not counted as success.
        success = (
            minimum is not None
            and matched_min_index is not None
            and min_side_error is not None
            and min_side_error <= 0.04
        )
        identity_successes += int(identity_success)

        matched = set()
        correct_candidates = 0
        for candidate in selection.candidates:
            index, _ = match_candidate(candidate, truths)
            if index is not None and index not in matched:
                matched.add(index)
                correct_candidates += 1
        detected_truths += len(matched)
        total_truths += len(truths)
        selected_correct += correct_candidates
        selected_total += len(selection.candidates)
        explanation_success = (
            correct_candidates == len(selection.candidates)
            and len(selection.candidates) == len(truths)
            and len(matched) == len(truths)
        )
        exact_explanations += int(explanation_success)

        records.append(
            {
                "seed": seed,
                "mode": metadata["mode"],
                "success": int(success),
                "explanation_success": int(explanation_success),
                "truth_min": truths[truth_min_index].side,
                "predicted_min": minimum.side if minimum else "",
                "relative_error": min_side_error if min_side_error is not None else "",
                "accepted": sum(candidate.accepted for candidate in detection.candidates),
                "selected": len(selection.candidates),
                "candidates": len(detection.candidates),
                "truth_recall": len(matched) / len(truths),
                "selection_precision": correct_candidates / max(1, len(selection.candidates)),
                "elapsed_ms": detection.elapsed_ms,
                "total_elapsed_ms": total_times[-1],
            }
        )

        if (not success or not explanation_success) and failure_count < save_failures:
            stem = f"failure_{seed:04d}_{metadata['mode']}"
            image.save(output_dir / f"{stem}.png")
            draw_overlay(image, detection, truths).save(output_dir / f"{stem}_debug.png")
            detail = {
                "metadata": metadata,
                "record": records[-1],
                "orientations": detection.orientations,
                "candidates": [asdict(candidate) for candidate in detection.candidates[:40]],
                "selection": [asdict(candidate) for candidate in selection.candidates],
            }
            (output_dir / f"{stem}.json").write_text(
                json.dumps(detail, indent=2, ensure_ascii=False), encoding="utf-8"
            )
            failure_count += 1

    with (output_dir / "results.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    successes = sum(record["success"] for record in records)
    errors = [
        float(record["relative_error"])
        for record in records
        if record["relative_error"] != ""
    ]
    summary = {
        "cases": count,
        "minimum_successes": successes,
        "minimum_success_rate": successes / count,
        "minimum_identity_success_rate": identity_successes / count,
        "all_square_recall": detected_truths / max(1, total_truths),
        "all_square_precision": selected_correct / max(1, selected_total),
        "exact_explanation_rate": exact_explanations / count,
        "minimum_side_error_median": statistics.median(errors) if errors else None,
        "minimum_side_error_max": max(errors) if errors else None,
        "runtime_ms_mean": statistics.mean(times),
        "runtime_ms_p95": sorted(times)[max(0, math.ceil(len(times) * 0.95) - 1)],
        "total_runtime_ms_mean": statistics.mean(total_times),
        "total_runtime_ms_p95": sorted(total_times)[max(0, math.ceil(len(total_times) * 0.95) - 1)],
        "mode": mode,
        "noisy": noisy,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return summary


def _contiguous_runs(values: Sequence[bool], min_length: int = 3) -> list[tuple[int, int]]:
    runs = []
    start = None
    for index, value in enumerate(values):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if index - start >= min_length:
                runs.append((start, index - 1))
            start = None
    if start is not None and len(values) - start >= min_length:
        runs.append((start, len(values) - 1))
    return runs


def find_inner_aperture(image: Image.Image) -> tuple[int, int, int, int]:
    """Find the white opening inside the thick A4 black frame in a rendered PDF."""

    gray = image.convert("L")
    width, height = gray.size
    pixels = gray.load()
    col_counts = [sum(1 for y in range(height) if pixels[x, y] < 64) for x in range(width)]
    row_counts = [sum(1 for x in range(width) if pixels[x, y] < 64) for y in range(height)]
    col_runs = _contiguous_runs([count > height * 0.48 for count in col_counts], 5)
    row_runs = _contiguous_runs([count > width * 0.48 for count in row_counts], 5)
    if len(col_runs) < 2 or len(row_runs) < 2:
        raise ValueError("could not locate the two vertical and horizontal A4 frame bands")
    left, right = col_runs[0], col_runs[-1]
    top, bottom = row_runs[0], row_runs[-1]
    x0, x1 = left[1] + 1, right[0]
    y0, y1 = top[1] + 1, bottom[0]
    if x1 - x0 < width * 0.25 or y1 - y0 < height * 0.25:
        raise ValueError("detected A4 aperture is implausibly small")
    return x0, y0, x1, y1


def validate_example(image_path: Path, output_dir: Path, max_dimension: int = 640) -> dict:
    source = Image.open(image_path).convert("L")
    aperture = find_inner_aperture(source)
    cropped = source.crop(aperture)
    scale = min(1.0, max_dimension / max(cropped.size))
    if scale < 1.0:
        cropped = cropped.resize(
            (max(1, round(cropped.width * scale)), max(1, round(cropped.height * scale))),
            Image.Resampling.LANCZOS,
        )
    detection = detect_squares(cropped, threshold=128, min_side=min(cropped.size) * 0.08)
    selection = select_square_explanation(cropped, detection, threshold=128)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = image_path.stem
    cropped.save(output_dir / f"{stem}_aperture.png")
    draw_overlay(
        cropped,
        detection,
        show_truth=False,
        show_rejected=False,
        selected=selection.candidates,
        highlight_minimum=True,
    ).save(output_dir / f"{stem}_result.png")
    draw_overlay(
        cropped,
        detection,
        show_truth=False,
        show_rejected=False,
    ).save(output_dir / f"{stem}_accepted.png")
    draw_overlay(cropped, detection).save(output_dir / f"{stem}_debug.png")
    accepted = [candidate for candidate in detection.candidates if candidate.accepted]
    payload = {
        "source": str(image_path),
        "aperture": aperture,
        "processed_size": cropped.size,
        "elapsed_ms": detection.elapsed_ms,
        "orientations": detection.orientations,
        "accepted": [asdict(candidate) for candidate in accepted],
        "selected": [asdict(candidate) for candidate in selection.candidates],
        "selection_loss": selection.loss,
        "selection_core_missing": selection.core_missing,
        "selection_false_positive": selection.false_positive,
        "selection_subsets": selection.evaluated_subsets,
        "candidate_count": len(detection.candidates),
    }
    (output_dir / f"{stem}_result.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="generate and detect one synthetic scene")
    generate.add_argument("--seed", type=int, default=1)
    generate.add_argument("--mode", choices=("mixed", "axis", "rotated"), default="mixed")
    generate.add_argument("--clean", action="store_true")
    generate.add_argument("--output", type=Path, default=Path("overlap_lab_output/single"))

    batch = subparsers.add_parser("batch", help="run randomized known-ground-truth validation")
    batch.add_argument("--count", type=int, default=100)
    batch.add_argument("--start-seed", type=int, default=1)
    batch.add_argument("--mode", choices=("mixed", "axis", "rotated"), default="mixed")
    batch.add_argument("--clean", action="store_true")
    batch.add_argument("--output", type=Path, default=Path("overlap_lab_output/batch"))

    example = subparsers.add_parser("example", help="detect squares in a rendered target PDF image")
    example.add_argument("images", nargs="+", type=Path)
    example.add_argument("--output", type=Path, default=Path("overlap_lab_output/examples"))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "generate":
        payload = save_scene(args.output, args.seed, args.mode, not args.clean)
        accepted = [item for item in payload["detection"]["candidates"] if item["accepted"]]
        print(json.dumps({"seed": args.seed, "accepted": len(accepted), "selected": len(payload["selection"]["candidates"]), "output": str(args.output)}, ensure_ascii=False))
        return 0
    if args.command == "batch":
        summary = validate_batch(args.output, args.count, args.start_seed, args.mode, not args.clean)
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return 0 if summary["minimum_success_rate"] >= 0.90 else 2
    if args.command == "example":
        results = [validate_example(path, args.output) for path in args.images]
        compact = [
            {
                "source": result["source"],
                "orientations": result["orientations"],
                "accepted_sides": [round(item["side"], 2) for item in result["accepted"]],
                "selected_sides": [round(item["side"], 2) for item in result["selected"]],
                "elapsed_ms": round(result["elapsed_ms"], 1),
            }
            for result in results
        ]
        print(json.dumps(compact, indent=2, ensure_ascii=False))
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
