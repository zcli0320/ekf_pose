#!/usr/bin/env python3
import argparse
import csv
import json
import os

import matplotlib.pyplot as plt


def load_row(spec):
    if "=" not in spec:
        raise ValueError("input must be LABEL=PATH, got: {}".format(spec))
    label, path = spec.split("=", 1)
    with open(path, encoding="utf-8") as source:
        report = json.load(source)
    if not report.get("results"):
        raise ValueError("benchmark JSON has no results: {}".format(path))
    metrics = report["results"][0]["metrics"]
    ekf_vs_odom = metrics["ekf_vs_odom"]
    return {
        "label": label,
        "path": path,
        "count": ekf_vs_odom["count"],
        "mean_m": ekf_vs_odom["mean"],
        "mse_m2": ekf_vs_odom["mse"],
        "rmse_m": ekf_vs_odom["rmse"],
        "p95_m": ekf_vs_odom["p95"],
        "max_m": ekf_vs_odom["max"],
        "ekf_step_p95_m": metrics["ekf_step_p95"],
        "ekf_step_max_m": metrics["ekf_step_max"],
        "reset_count": metrics.get("reset_count", 0),
        "odom_realign_count": metrics.get("odom_realign_count", 0),
        "gnss_reject_count": metrics.get("gnss_reject_count", 0),
    }


def write_csv(rows, path):
    fieldnames = [
        "label",
        "count",
        "mean_m",
        "mse_m2",
        "rmse_m",
        "p95_m",
        "max_m",
        "ekf_step_p95_m",
        "ekf_step_max_m",
        "reset_count",
        "odom_realign_count",
        "gnss_reject_count",
        "path",
    ]
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def annotate_bars(axis, bars, values, fmt):
    ymax = max(values) if values else 1.0
    for bar, value in zip(bars, values):
        axis.text(
            bar.get_x() + bar.get_width() / 2.0,
            value + ymax * 0.03,
            fmt.format(value),
            ha="center",
            va="bottom",
            fontsize=9,
        )


def plot(rows, path):
    labels = [row["label"] for row in rows]
    p95 = [row["p95_m"] for row in rows]
    mse = [row["mse_m2"] for row in rows]

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), constrained_layout=True)
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756"]

    p95_bars = axes[0].bar(labels, p95, color=colors[: len(rows)], width=0.55)
    axes[0].set_title("EKF vs Odom P95")
    axes[0].set_ylabel("Position error P95 (m)")
    axes[0].grid(axis="y", alpha=0.3)
    annotate_bars(axes[0], p95_bars, p95, "{:.3f}")

    mse_bars = axes[1].bar(labels, mse, color=colors[: len(rows)], width=0.55)
    axes[1].set_title("EKF vs Odom MSE")
    axes[1].set_ylabel("Position error MSE (m^2)")
    axes[1].grid(axis="y", alpha=0.3)
    annotate_bars(axes[1], mse_bars, mse, "{:.5f}")

    for axis in axes:
        axis.set_xlabel("Dataset")
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    fig.suptitle("Fused localization error against odom reference")
    fig.savefig(path, dpi=200)


def main():
    parser = argparse.ArgumentParser(description="Plot EKF-vs-odom P95 and MSE from benchmark JSON files.")
    parser.add_argument("--input", action="append", required=True, help="LABEL=benchmark.json")
    parser.add_argument("--output-dir", default="results/ekf_odom_error_stats")
    args = parser.parse_args()

    rows = [load_row(spec) for spec in args.input]
    os.makedirs(args.output_dir, exist_ok=True)
    csv_path = os.path.join(args.output_dir, "ekf_odom_error_stats.csv")
    png_path = os.path.join(args.output_dir, "ekf_odom_error_stats.png")
    write_csv(rows, csv_path)
    plot(rows, png_path)
    print("csv={}".format(csv_path))
    print("figure={}".format(png_path))


if __name__ == "__main__":
    main()
