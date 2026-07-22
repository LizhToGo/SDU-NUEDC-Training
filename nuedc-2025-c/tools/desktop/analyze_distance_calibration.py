"""Analyze a CSV produced by distance_calibration_capture.py on a PC."""

import argparse
import csv
import statistics
from collections import defaultdict


def load_records(path):
    records = []
    with open(path, "r", encoding="utf-8-sig", newline="") as file:
        for row in csv.DictReader(file):
            records.append({
                "distance_cm": float(row["distance_cm"]),
                "frame_scale": float(row["frame_scale"]),
                "scale_spread_pct": float(row["scale_spread_pct"]),
                "center_shift_ratio": float(row["center_shift_ratio"]),
                "total_ms": int(row["total_ms"]),
            })
    return records


def summarize(records):
    groups = defaultdict(list)
    for record in records:
        groups[record["distance_cm"]].append(record)
    summaries = []
    for distance_cm in sorted(groups):
        group = groups[distance_cm]
        scales = [record["frame_scale"] for record in group]
        center = statistics.median(scales)
        deviations = [abs(scale - center) for scale in scales]
        summaries.append({
            "distance_cm": distance_cm,
            "count": len(group),
            "median_scale": center,
            "mad_scale": statistics.median(deviations),
            "min_scale": min(scales),
            "max_scale": max(scales),
            "mean_spread_pct": statistics.fmean(
                record["scale_spread_pct"] for record in group
            ),
            "max_spread_pct": max(
                record["scale_spread_pct"] for record in group
            ),
            "mean_center_shift_pct": statistics.fmean(
                record["center_shift_ratio"] for record in group
            )
            * 100.0,
            "mean_time_ms": statistics.fmean(
                record["total_ms"] for record in group
            ),
        })
    return summaries


def fit_reciprocal(summaries, with_intercept=True):
    xs = [1.0 / item["median_scale"] for item in summaries]
    ys = [item["distance_cm"] for item in summaries]
    if with_intercept:
        mean_x = statistics.fmean(xs)
        mean_y = statistics.fmean(ys)
        denominator = sum((x - mean_x) ** 2 for x in xs)
        if denominator <= 1e-12:
            raise ValueError("calibration scales do not span a useful range")
        coefficient = sum(
            (x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)
        ) / denominator
        intercept = mean_y - coefficient * mean_x
    else:
        denominator = sum(x * x for x in xs)
        coefficient = sum(x * y for x, y in zip(xs, ys)) / denominator
        intercept = 0.0
    return coefficient, intercept


def model_errors(summaries, coefficient, intercept):
    errors = []
    for item in summaries:
        predicted = coefficient / item["median_scale"] + intercept
        errors.append(predicted - item["distance_cm"])
    return errors


def leave_one_distance_out_errors(summaries):
    if len(summaries) < 4:
        return []
    errors = []
    for omitted_index in range(len(summaries)):
        training = [
            item
            for index, item in enumerate(summaries)
            if index != omitted_index
        ]
        coefficient, intercept = fit_reciprocal(training, with_intercept=True)
        omitted = summaries[omitted_index]
        predicted = coefficient / omitted["median_scale"] + intercept
        errors.append(predicted - omitted["distance_cm"])
    return errors


def interpolate_table(scale, summaries):
    """Mirror production reciprocal-scale table interpolation."""
    if scale <= 0.0:
        return None
    points = [
        (item["median_scale"], item["distance_cm"])
        for item in summaries
        if item["median_scale"] > 0.0 and item["distance_cm"] > 0.0
    ]
    if len(points) < 2:
        return None
    points.sort(reverse=True)
    if scale >= points[0][0]:
        first, second = points[0], points[1]
    elif scale <= points[-1][0]:
        first, second = points[-2], points[-1]
    else:
        first, second = points[0], points[1]
        for high, low in zip(points, points[1:]):
            if high[0] >= scale >= low[0]:
                first, second = high, low
                break
    query_reciprocal = 1.0 / scale
    first_reciprocal = 1.0 / first[0]
    second_reciprocal = 1.0 / second[0]
    denominator = second_reciprocal - first_reciprocal
    if abs(denominator) < 1e-12:
        return (first[1] + second[1]) * 0.5
    ratio = (query_reciprocal - first_reciprocal) / denominator
    return first[1] + (second[1] - first[1]) * ratio


def leave_one_distance_out_table_errors(summaries):
    """Validate the actual production interpolation, one distance at a time."""
    if len(summaries) < 3:
        return []
    errors = []
    for omitted_index, omitted in enumerate(summaries):
        training = [
            item
            for index, item in enumerate(summaries)
            if index != omitted_index
        ]
        predicted = interpolate_table(omitted["median_scale"], training)
        errors.append(predicted - omitted["distance_cm"])
    return errors


def monotonic_violations(summaries):
    violations = []
    for previous, current in zip(summaries, summaries[1:]):
        if current["median_scale"] >= previous["median_scale"]:
            violations.append((previous["distance_cm"], current["distance_cm"]))
    return violations


def error_summary(errors):
    if not errors:
        return 0.0, 0.0
    return (
        statistics.fmean(abs(error) for error in errors),
        max(abs(error) for error in errors),
    )


DEFAULT_EXPECTED_POINTS = 21


def print_report(
    records,
    expected_samples,
    expected_points=DEFAULT_EXPECTED_POINTS,
):
    summaries = summarize(records)
    if not summaries:
        raise ValueError("CSV contains no calibration records")

    print("distance  n  median_scale  MAD       spread_mean/max  center_mean  time_mean")
    for item in summaries:
        marker = "" if item["count"] >= expected_samples else "  INCOMPLETE"
        print(
            "%7.1f %2d  %12.6f  %.6f  %6.3f/%6.3f%%  %7.3f%%  %7.0fms%s"
            % (
                item["distance_cm"],
                item["count"],
                item["median_scale"],
                item["mad_scale"],
                item["mean_spread_pct"],
                item["max_spread_pct"],
                item["mean_center_shift_pct"],
                item["mean_time_ms"],
                marker,
            )
        )

    violations = monotonic_violations(summaries)
    print("\nmonotonic:", "PASS" if not violations else "FAIL %s" % violations)

    reciprocal_cross_errors = []
    if len(summaries) >= 2:
        coefficient, intercept = fit_reciprocal(
            summaries,
            with_intercept=True,
        )
        fit_errors = model_errors(summaries, coefficient, intercept)
        fit_mae, fit_max = error_summary(fit_errors)
        print(
            "two-parameter model: D = %.9f / scale %+.9f"
            % (coefficient, intercept)
        )
        print("  in-sample MAE=%.3fcm max=%.3fcm" % (fit_mae, fit_max))

        coefficient_zero, _ = fit_reciprocal(
            summaries,
            with_intercept=False,
        )
        zero_errors = model_errors(summaries, coefficient_zero, 0.0)
        zero_mae, zero_max = error_summary(zero_errors)
        print("one-parameter model: D = %.9f / scale" % coefficient_zero)
        print("  in-sample MAE=%.3fcm max=%.3fcm" % (zero_mae, zero_max))

        reciprocal_cross_errors = leave_one_distance_out_errors(summaries)
        reciprocal_cross_mae, reciprocal_cross_max = error_summary(
            reciprocal_cross_errors
        )
        if reciprocal_cross_errors:
            print(
                "reciprocal leave-one-distance-out: MAE=%.3fcm max=%.3fcm"
                % (reciprocal_cross_mae, reciprocal_cross_max)
            )
    else:
        print("reciprocal models: NEED_AT_LEAST_2_DISTANCE_POINTS")

    table_cross_errors = leave_one_distance_out_table_errors(summaries)
    table_cross_mae, table_cross_max = error_summary(table_cross_errors)
    if table_cross_errors:
        print(
            "reciprocal-scale table leave-one-distance-out: MAE=%.3fcm max=%.3fcm"
            % (table_cross_mae, table_cross_max)
        )

    print("\nDISTANCE_CALIBRATION_POINTS = (")
    for item in summaries:
        print(
            "    (%.1f, %.6f),"
            % (item["distance_cm"], item["median_scale"])
        )
    print(")")

    advanced_ready = (
        not violations
        and all(item["count"] >= expected_samples for item in summaries)
        and len(summaries) >= expected_points
        and table_cross_max <= 2.0
    )
    print(
        "\npoint coverage: %d/%d"
        % (len(summaries), expected_points)
    )
    print(
        "\nadvanced-distance readiness:",
        "PASS" if advanced_ready else "NEEDS_MORE_DATA_OR_REVIEW",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--expected-samples", type=int, default=5)
    parser.add_argument(
        "--expected-points",
        type=int,
        default=DEFAULT_EXPECTED_POINTS,
    )
    args = parser.parse_args()
    records = load_records(args.csv_path)
    print_report(
        records,
        args.expected_samples,
        args.expected_points,
    )


if __name__ == "__main__":
    main()
