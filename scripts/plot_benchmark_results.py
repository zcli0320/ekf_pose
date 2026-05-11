#!/usr/bin/env python3
import argparse
import csv
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


TRIAL_LABELS = {
    "no_gnss": "No GNSS",
    "gnss_very_conservative": "Weak GNSS",
    "gnss_conservative": "Conservative",
    "gnss_medium": "Medium",
    "gnss_balanced": "Strong GNSS",
    "gnss_trusted": "Trusted GNSS",
    "gnss_trusted_odom_loose": "Trusted GNSS + loose odom",
    "gnss_strong_odom_loose": "Strong GNSS + loose odom",
}


def flatten_results(report):
    rows = []
    for item in report["results"]:
        trial = item["trial"]
        metrics = item["metrics"]
        name = trial["name"]
        rows.append(
            {
                "name": name,
                "label": TRIAL_LABELS.get(name, name),
                "use_gnss": trial.get("use_gnss", ""),
                "gnss_min_interval": trial.get("gnss_min_interval", ""),
                "gnss_min_cov_xy": trial.get("gnss_min_cov_xy", ""),
                "gnss_min_cov_z": trial.get("gnss_min_cov_z", ""),
                "gnss_position_covariance_floor_xy": trial.get("gnss_position_covariance_floor_xy", ""),
                "gnss_position_covariance_floor_z": trial.get("gnss_position_covariance_floor_z", ""),
                "gnss_alignment_min_samples": trial.get("gnss_alignment_min_samples", ""),
                "gnss_alignment_min_motion": trial.get("gnss_alignment_min_motion", ""),
                "ekf_vs_odom_mean": metrics["ekf_vs_odom"]["mean"],
                "ekf_vs_odom_p95": metrics["ekf_vs_odom"]["p95"],
                "ekf_vs_odom_max": metrics["ekf_vs_odom"]["max"],
                "ekf_vs_ground_truth_mean": metrics.get("ekf_vs_ground_truth", {}).get("mean", ""),
                "ekf_vs_ground_truth_p95": metrics.get("ekf_vs_ground_truth", {}).get("p95", ""),
                "ekf_vs_ground_truth_max": metrics.get("ekf_vs_ground_truth", {}).get("max", ""),
                "odom_vs_ground_truth_mean": metrics.get("odom_vs_ground_truth", {}).get("mean", ""),
                "odom_vs_ground_truth_p95": metrics.get("odom_vs_ground_truth", {}).get("p95", ""),
                "gps_vs_ground_truth_mean": metrics.get("gps_vs_ground_truth", {}).get("mean", ""),
                "gps_vs_ground_truth_p95": metrics.get("gps_vs_ground_truth", {}).get("p95", ""),
                "ekf_vs_gnss_mean": metrics["ekf_vs_aligned_gnss"]["mean"],
                "ekf_vs_gnss_p95": metrics["ekf_vs_aligned_gnss"]["p95"],
                "ekf_vs_gnss_max": metrics["ekf_vs_aligned_gnss"]["max"],
                "ekf_vs_rigid_gnss_mean": metrics.get("ekf_vs_rigid_gnss", {}).get("mean", ""),
                "ekf_vs_rigid_gnss_p95": metrics.get("ekf_vs_rigid_gnss", {}).get("p95", ""),
                "ekf_vs_node_gnss_path_mean": metrics.get("ekf_vs_node_gnss_path", {}).get("mean", ""),
                "ekf_vs_node_gnss_path_p95": metrics.get("ekf_vs_node_gnss_path", {}).get("p95", ""),
                "gnss_rigid_yaw_rad": metrics.get("gnss_rigid_yaw_rad", ""),
                "ekf_step_p95": metrics["ekf_step_p95"],
                "ekf_step_max": metrics["ekf_step_max"],
                "reset_count": metrics["reset_count"],
                "odom_realign_count": metrics.get("odom_realign_count", 0),
                "odom_weak_count": metrics.get("odom_weak_count", 0),
                "gnss_reject_count": metrics["gnss_reject_count"],
                "gnss_weak_count": metrics.get("gnss_weak_count", 0),
                "gnss_yaw_alignment_count": metrics.get("gnss_yaw_alignment_count", 0),
                "score": metrics["score"],
            }
        )
    return rows


def write_csv(rows, path):
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_bar_chart(rows, output_dir):
    labels = [row["label"] for row in rows]
    x = range(len(rows))
    width = 0.35

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar([i - width / 2 for i in x], [row["ekf_vs_odom_p95"] for row in rows], width, label="EKF vs odom P95")
    ax.bar([i + width / 2 for i in x], [row["ekf_vs_gnss_p95"] for row in rows], width, label="EKF vs aligned GNSS P95")
    ax.set_ylabel("Position error (m)")
    ax.set_title("EKF Fusion Accuracy Comparison on all_gps.bag")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "fusion_accuracy_p95.png"), dpi=300)
    fig.savefig(os.path.join(output_dir, "fusion_accuracy_p95.pdf"))
    plt.close(fig)


def save_step_chart(rows, output_dir):
    labels = [row["label"] for row in rows]
    x = range(len(rows))
    width = 0.35

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar([i - width / 2 for i in x], [row["ekf_step_p95"] for row in rows], width, label="EKF step P95")
    ax.bar([i + width / 2 for i in x], [row["ekf_step_max"] for row in rows], width, label="EKF step max")
    ax.set_ylabel("Step distance (m)")
    ax.set_title("EKF Output Smoothness Comparison on all_gps.bag")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "fusion_step_smoothness.png"), dpi=300)
    fig.savefig(os.path.join(output_dir, "fusion_step_smoothness.pdf"))
    plt.close(fig)


def save_improvement_chart(rows, output_dir):
    baseline = next(row for row in rows if row["name"] == "no_gnss")
    best = next(row for row in rows if row["name"] == "gnss_very_conservative")
    metrics = [
        ("Odom P95", baseline["ekf_vs_odom_p95"], best["ekf_vs_odom_p95"]),
        ("GNSS P95", baseline["ekf_vs_gnss_p95"], best["ekf_vs_gnss_p95"]),
        ("Step max", baseline["ekf_step_max"], best["ekf_step_max"]),
    ]
    labels = [item[0] for item in metrics]
    before = [item[1] for item in metrics]
    after = [item[2] for item in metrics]
    x = range(len(metrics))
    width = 0.36

    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    ax.bar([i - width / 2 for i in x], before, width, label="Before: no GNSS")
    ax.bar([i + width / 2 for i in x], after, width, label="After: optimized weak GNSS")
    ax.set_ylabel("Distance (m)")
    ax.set_title("Improvement After EKF/GNSS Optimization")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.5)
    ax.legend()

    for i, (_, base_value, best_value) in enumerate(metrics):
        reduction = (base_value - best_value) / base_value * 100.0
        ax.text(i + width / 2, best_value, "-{:.1f}%".format(reduction), ha="center", va="bottom", fontsize=9)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "fusion_before_after_improvement.png"), dpi=300)
    fig.savefig(os.path.join(output_dir, "fusion_before_after_improvement.pdf"))
    plt.close(fig)


def write_latex_table(rows, output_dir):
    path = os.path.join(output_dir, "fusion_benchmark_table.tex")
    with open(path, "w") as table:
        table.write("\\begin{tabular}{lrrrrr}\n")
        table.write("\\hline\n")
        table.write("Method & Odom P95 & GNSS P95 & Step Max & Reset & Odom Align & GNSS Reject \\\\\n")
        table.write("\\hline\n")
        for row in rows:
            line = "{} & {:.4f} & {:.4f} & {:.4f} & {} & {} & {} \\\\".format(
                row["label"],
                row["ekf_vs_odom_p95"],
                row["ekf_vs_gnss_p95"],
                row["ekf_step_max"],
                row["reset_count"],
                row["odom_realign_count"],
                row["gnss_reject_count"],
            )
            table.write(line + "\n")
        table.write("\\hline\n")
        table.write("\\end{tabular}\n")


def main():
    parser = argparse.ArgumentParser(description="Create publication-ready plots from EKF benchmark JSON.")
    parser.add_argument("benchmark_json", help="Path to benchmark JSON")
    parser.add_argument("--output-dir", default="results/all_gps_benchmark", help="Directory for generated figures and tables")
    args = parser.parse_args()

    with open(args.benchmark_json) as json_file:
        report = json.load(json_file)

    os.makedirs(args.output_dir, exist_ok=True)
    rows = flatten_results(report)
    write_csv(rows, os.path.join(args.output_dir, "fusion_benchmark_metrics.csv"))
    write_latex_table(rows, args.output_dir)
    save_bar_chart(rows, args.output_dir)
    save_step_chart(rows, args.output_dir)
    save_improvement_chart(rows, args.output_dir)
    print("wrote figures and tables to {}".format(args.output_dir))


if __name__ == "__main__":
    main()
