#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""逐帧比较参考短线点簇的RANSAC与PCA几何特征。"""

from __future__ import print_function

import argparse
import csv
import math

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


TOPIC = "/perception/debug/reference/cloud_cluster"
RANSAC_DISTANCE = 0.01
RANSAC_ITERATIONS = 100


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else float("nan")


def normalize_direction(direction):
    norm = np.linalg.norm(direction)
    if norm < 1e-12:
        return None
    direction = direction / norm
    if direction[0] < 0.0:
        direction = -direction
    return direction


def pca_line(points):
    center = np.mean(points, axis=0)
    covariance = np.cov((points - center).T, bias=True)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    direction = normalize_direction(eigenvectors[:, order[0]])
    return center, direction, eigenvalues


def angle_degrees(direction):
    return math.degrees(math.atan2(direction[1], direction[0]))


def line_angle_difference(first, second):
    difference = abs(angle_degrees(first) - angle_degrees(second)) % 180.0
    return min(difference, 180.0 - difference)


def ransac_line(points, frame_index):
    random_state = np.random.RandomState(20260924 + frame_index)
    best_mask = None
    best_count = -1
    best_error = float("inf")
    point_count = len(points)
    for _ in range(RANSAC_ITERATIONS):
        first_index, second_index = random_state.choice(point_count, 2, replace=False)
        direction = normalize_direction(points[second_index] - points[first_index])
        if direction is None:
            continue
        relative = points - points[first_index]
        distances = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
        mask = distances <= RANSAC_DISTANCE
        count = int(np.count_nonzero(mask))
        error = float(np.mean(distances[mask] ** 2)) if count else float("inf")
        if count > best_count or (count == best_count and error < best_error):
            best_mask = mask
            best_count = count
            best_error = error
    if best_mask is None or best_count < 2:
        return None

    # 与在线optimizeModelCoefficients意图一致：使用全部RANSAC内点重新估计最终直线。
    center, direction, _ = pca_line(points[best_mask])
    relative = points - center
    distances = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
    optimized_mask = distances <= RANSAC_DISTANCE
    if np.count_nonzero(optimized_mask) >= 2:
        center, direction, _ = pca_line(points[optimized_mask])
        relative = points - center
        distances = np.abs(relative[:, 0] * direction[1] - relative[:, 1] * direction[0])
        best_mask = distances <= RANSAC_DISTANCE
    return center, direction, best_mask, distances


def analyze_cloud(msg, frame_index, candidate_index):
    raw_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    xyz = np.asarray(raw_points, dtype=float)
    xy = xyz[:, :2]
    point_count = len(xy)
    if point_count < 2:
        return None

    pca_center, pca_direction, eigenvalues = pca_line(xy)
    normal = np.asarray((-pca_direction[1], pca_direction[0]))
    relative = xy - pca_center
    longitudinal = np.dot(relative, pca_direction)
    transverse = np.dot(relative, normal)
    q_long = np.percentile(longitudinal, [2.0, 98.0])
    q_transverse = np.percentile(transverse, [2.0, 98.0])
    robust_length = float(q_long[1] - q_long[0])
    robust_width = float(q_transverse[1] - q_transverse[0])
    full_length = float(np.max(longitudinal) - np.min(longitudinal))
    full_width = float(np.max(transverse) - np.min(transverse))

    ransac_result = ransac_line(xy, frame_index)
    if ransac_result is None:
        ransac_center = np.asarray((float("nan"), float("nan")))
        ransac_direction = np.asarray((float("nan"), float("nan")))
        inlier_mask = np.zeros(point_count, dtype=bool)
        distances = np.full(point_count, float("nan"))
    else:
        ransac_center, ransac_direction, inlier_mask, distances = ransac_result

    inlier_count = int(np.count_nonzero(inlier_mask))
    inlier_ratio = float(inlier_count) / point_count
    eigen_ratio = float(eigenvalues[0] / max(eigenvalues[1], 1e-15))
    delta_yaw = line_angle_difference(pca_direction, ransac_direction) if ransac_result is not None else float("nan")
    center_delta = float(np.linalg.norm(ransac_center - pca_center)) if ransac_result is not None else float("nan")

    sorted_longitudinal = np.sort(longitudinal)
    longitudinal_gaps = np.diff(sorted_longitudinal)
    direct_distance = float(np.linalg.norm(xy[-1] - xy[0]))
    online_failed = inlier_count < 10 or inlier_ratio < 0.8

    return {
        "frame_index": frame_index,
        "candidate_index": candidate_index,
        "stamp": msg.header.stamp.to_sec(),
        "frame_id": msg.header.frame_id,
        "point_count": point_count,
        "direct_distance_m": direct_distance,
        "robust_length_m": robust_length,
        "robust_width_m": robust_width,
        "width_length_ratio": robust_width / robust_length if robust_length > 1e-12 else float("inf"),
        "full_length_m": full_length,
        "full_width_m": full_width,
        "pca_lambda1": float(eigenvalues[0]),
        "pca_lambda2": float(eigenvalues[1]),
        "pca_lambda_ratio": eigen_ratio,
        "pca_center_x": float(pca_center[0]),
        "pca_center_y": float(pca_center[1]),
        "pca_yaw_deg": angle_degrees(pca_direction),
        "ransac_inliers": inlier_count,
        "ransac_inlier_ratio": inlier_ratio,
        "ransac_center_x": float(ransac_center[0]),
        "ransac_center_y": float(ransac_center[1]),
        "ransac_yaw_deg": angle_degrees(ransac_direction) if ransac_result is not None else float("nan"),
        "ransac_pca_delta_yaw_deg": delta_yaw,
        "ransac_pca_center_delta_m": center_delta,
        "ransac_distance_p95_m": percentile(distances.tolist(), 95),
        "longitudinal_max_gap_m": float(np.max(longitudinal_gaps)) if len(longitudinal_gaps) else 0.0,
        "online_failed": int(online_failed),
    }


def describe(label, rows, field, scale=1.0):
    values = [row[field] * scale for row in rows if np.isfinite(row[field])]
    print("{}: min={:.6f} median={:.6f} p95={:.6f} p99={:.6f} max={:.6f}".format(
        label, min(values), percentile(values, 50), percentile(values, 95),
        percentile(values, 99), max(values)))


def wrapped_line_delta(first, second):
    difference = abs(first - second) % 180.0
    return min(difference, 180.0 - difference)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("csv_path")
    args = parser.parse_args()

    rows = []
    stamp_counts = {}
    with rosbag.Bag(args.bag, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[TOPIC]):
            stamp_key = int(round(msg.header.stamp.to_sec() * 1000000.0))
            candidate_index = stamp_counts.get(stamp_key, 0)
            stamp_counts[stamp_key] = candidate_index + 1
            row = analyze_cloud(msg, len(rows), candidate_index)
            if row is not None:
                rows.append(row)

    fieldnames = list(rows[0].keys())
    with open(args.csv_path, "wb") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("messages={} unique_stamps={} duplicate_candidates={} output={}".format(
        len(rows), len(stamp_counts), sum(count - 1 for count in stamp_counts.values()), args.csv_path))
    describe("N", rows, "point_count")
    describe("L_Q2_Q98_mm", rows, "robust_length_m", 1000.0)
    describe("W_Q2_Q98_mm", rows, "robust_width_m", 1000.0)
    describe("W_over_L", rows, "width_length_ratio")
    describe("PCA_lambda1_over_lambda2", rows, "pca_lambda_ratio")
    describe("RANSAC_inlier_ratio", rows, "ransac_inlier_ratio")
    describe("RANSAC_PCA_delta_yaw_deg", rows, "ransac_pca_delta_yaw_deg")
    describe("RANSAC_PCA_center_delta_mm", rows, "ransac_pca_center_delta_m", 1000.0)
    describe("longitudinal_max_gap_mm", rows, "longitudinal_max_gap_m", 1000.0)

    pca_center_steps = []
    pca_yaw_steps = []
    ransac_center_steps = []
    ransac_yaw_steps = []
    for previous, current in zip(rows, rows[1:]):
        if current["candidate_index"] != 0 or previous["candidate_index"] != 0:
            continue
        pca_center_steps.append(1000.0 * math.hypot(
            current["pca_center_x"] - previous["pca_center_x"],
            current["pca_center_y"] - previous["pca_center_y"]))
        pca_yaw_steps.append(wrapped_line_delta(current["pca_yaw_deg"], previous["pca_yaw_deg"]))
        ransac_center_steps.append(1000.0 * math.hypot(
            current["ransac_center_x"] - previous["ransac_center_x"],
            current["ransac_center_y"] - previous["ransac_center_y"]))
        ransac_yaw_steps.append(wrapped_line_delta(current["ransac_yaw_deg"], previous["ransac_yaw_deg"]))
    for label, values in (
            ("PCA_center_step_mm", pca_center_steps),
            ("PCA_yaw_step_deg", pca_yaw_steps),
            ("RANSAC_center_step_mm", ransac_center_steps),
            ("RANSAC_yaw_step_deg", ransac_yaw_steps)):
        print("{}: median={:.6f} p95={:.6f} p99={:.6f} max={:.6f}".format(
            label, percentile(values, 50), percentile(values, 95), percentile(values, 99), max(values)))

    failures = [row for row in rows if row["online_failed"]]
    print("online_failures={} failure_rate={:.6f}%".format(
        len(failures), 100.0 * len(failures) / len(rows)))
    for label, field, reverse in (
            ("lowest_inlier_ratio", "ransac_inlier_ratio", False),
            ("largest_W_over_L", "width_length_ratio", True),
            ("largest_delta_yaw", "ransac_pca_delta_yaw_deg", True)):
        print(label)
        for row in sorted(rows, key=lambda item: item[field], reverse=reverse)[:10]:
            print("  frame={frame_index} stamp={stamp:.3f} N={point_count} W/L={width_length_ratio:.5f} "
                  "lambda={pca_lambda_ratio:.1f} inlier={ransac_inlier_ratio:.5f} "
                  "dyaw={ransac_pca_delta_yaw_deg:.5f}deg center_delta={ransac_pca_center_delta_m:.6f}m".format(**row))


if __name__ == "__main__":
    main()
