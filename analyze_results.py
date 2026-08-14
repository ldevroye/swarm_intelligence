#!/usr/bin/env python3

import csv
import os
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV_PATH = "results.csv"
OUTPUT_PATH = "analytics.csv"
PLOTS_DIR = "plots"


def parse_float(value):
    value = (value or "").strip()
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def controller_label(name):
    name = (name or "").strip().lower()
    if name == "main":
        return "main (controller.lua)"
    if name == "baseline":
        return "baseline (baseline.lua)"
    return name


def normalize_header(value):
    return (value or "").strip().lower().replace("#", "").replace(" ", "_")


def build_summary_rows(rows):
    grouped = defaultdict(list)
    metric_groups = defaultdict(list)

    for row in rows:
        if not row or all((cell or "").strip() == "" for cell in row):
            continue
        if row[0].strip().lower() == "controller":
            continue

        record = {}
        for index, cell in enumerate(row):
            if index >= len(rows[0]):
                break
            key = normalize_header(rows[0][index])
            record[key] = (cell or "").strip()

        controller = record.get("controller", "").strip()
        swarm_size = record.get("swarm_size", "").strip()
        score = parse_float(record.get("score", ""))
        objects = parse_float(record.get("objects", ""))
        robots = parse_float(record.get("robots", ""))

        if controller == "" or swarm_size == "" or score is None:
            continue

        try:
            size = int(float(swarm_size))
        except ValueError:
            continue

        grouped[(controller.lower(), size)].append(score)
        if objects is not None:
            metric_groups[(controller.lower(), size, "objects")].append(objects)
        if robots is not None:
            metric_groups[(controller.lower(), size, "robots")].append(robots)

    rows_out = [["controller", "swarm_size", "mean_score", "median_score", "std_score", "n_runs"]]
    for (controller_name, size), scores in sorted(grouped.items()):
        scores = sorted(scores)
        mean_value = statistics.mean(scores)
        median_value = statistics.median(scores)
        std_value = statistics.pstdev(scores) if len(scores) > 1 else 0.0
        rows_out.append([controller_label(controller_name), str(size), f"{mean_value:.6f}", f"{median_value:.6f}", f"{std_value:.6f}", str(len(scores))])

    return rows_out, metric_groups


def ensure_plot_dir():
    os.makedirs(PLOTS_DIR, exist_ok=True)


def series_for_metric(metric_name, grouped_metrics, controller_name=None):
    series = []
    for key, values in sorted(grouped_metrics.items()):
        if controller_name is not None and key[0].lower() != controller_name.lower():
            continue
        if key[2] != metric_name:
            continue
        if not values:
            continue
        mean_value = statistics.mean(values)
        series.append((key[1], mean_value))
    return sorted(series)


def save_png_plot(file_path, title, x_label, y_label, series_list):
    plt.figure(figsize=(8, 5))
    for series in series_list:
        x_values = [x for x, _ in series["points"]]
        y_values = [y for _, y in series["points"]]
        plt.plot(x_values, y_values, marker="o", label=series["label"])

    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(file_path, dpi=200)
    plt.close()


def generate_metric_plots(metric_name, grouped_metrics):
    ensure_plot_dir()
    controller_names = sorted({key[0].lower() for key in grouped_metrics.keys()})
    if not controller_names:
        return []

    file_paths = []
    for controller_name in controller_names:
        series = series_for_metric(metric_name, grouped_metrics, controller_name)
        if not series:
            continue
        file_path = os.path.join(PLOTS_DIR, f"{metric_name}_by_swarm_size_{controller_name}.png")
        save_png_plot(
            file_path,
            f"{metric_name.capitalize()} count by swarm size ({controller_label(controller_name)})",
            "swarm size",
            f"Mean {metric_name}",
            [{"label": controller_label(controller_name), "points": series}],
        )
        file_paths.append(file_path)

    combined_series = []
    for controller_name in controller_names:
        points = series_for_metric(metric_name, grouped_metrics, controller_name)
        if points:
            combined_series.append({
                "label": controller_label(controller_name),
                "points": points,
            })

    if combined_series:
        combined_file = os.path.join(PLOTS_DIR, f"{metric_name}_by_swarm_size_all_controllers.png")
        save_png_plot(
            combined_file,
            f"{metric_name.capitalize()} count by swarm size and controller",
            "swarm size",
            f"Mean {metric_name}",
            combined_series,
        )
        file_paths.append(combined_file)

    return file_paths


def generate_variance_plot(metric_name, grouped_metrics):
    ensure_plot_dir()
    controller_names = sorted({key[0].lower() for key in grouped_metrics.keys()})
    if not controller_names:
        return []

    file_paths = []
    for controller_name in controller_names:
        sizes = []
        means = []
        stds = []
        for key, values in sorted(grouped_metrics.items()):
            if key[0].lower() != controller_name.lower() or key[2] != metric_name:
                continue
            if not values:
                continue
            sizes.append(key[1])
            means.append(statistics.mean(values))
            stds.append(statistics.pstdev(values) if len(values) > 1 else 0.0)

        if not sizes:
            continue

        file_path = os.path.join(PLOTS_DIR, f"{metric_name}_variance_by_swarm_size_{controller_name}.png")
        plt.figure(figsize=(8, 5))
        color = "tab:blue" if controller_name == "baseline" else "tab:orange"
        lower = [max(0.0, m - s) for m, s in zip(means, stds)]
        upper = [m + s for m, s in zip(means, stds)]
        plt.fill_between(sizes, lower, upper, color=color, alpha=0.12, linewidth=0)
        plt.plot(sizes, means, '-', color=color, linewidth=2, label=controller_label(controller_name))
        plt.title(f"{metric_name.capitalize()} in target: mean and variance across 10 runs ({controller_label(controller_name)})")
        plt.xlabel("swarm size")
        plt.ylabel(f"Mean {metric_name} in target")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(file_path, dpi=200)
        plt.close()
        file_paths.append(file_path)

    combined_sizes = set()
    combined_data = {}
    for controller_name in controller_names:
        for key, values in grouped_metrics.items():
            if key[0].lower() != controller_name.lower() or key[2] != metric_name:
                continue
            combined_sizes.add(key[1])
            combined_data.setdefault(controller_name, {})[key[1]] = values

    if combined_data:
        combined_file = os.path.join(PLOTS_DIR, f"{metric_name}_variance_by_swarm_size_all_controllers.png")
        plt.figure(figsize=(8, 5))
        for controller_name in controller_names:
            if controller_name not in combined_data:
                continue
            sizes = sorted(combined_data[controller_name].keys())
            means = [statistics.mean(combined_data[controller_name][size]) for size in sizes]
            stds = [statistics.pstdev(combined_data[controller_name][size]) if len(combined_data[controller_name][size]) > 1 else 0.0 for size in sizes]
            color = "tab:blue" if controller_name == "baseline" else "tab:orange"
            lower = [max(0.0, m - s) for m, s in zip(means, stds)]
            upper = [m + s for m, s in zip(means, stds)]
            plt.fill_between(sizes, lower, upper, color=color, alpha=0.10, linewidth=0)
            plt.plot(sizes, means, '-', color=color, linewidth=2, label=controller_label(controller_name))
        plt.title(f"{metric_name.capitalize()} in target: mean and variance across 10 runs")
        plt.xlabel("swarm size")
        plt.ylabel(f"Mean {metric_name} in target")
        plt.grid(True, linestyle="--", alpha=0.5)
        plt.legend()
        plt.tight_layout()
        plt.savefig(combined_file, dpi=200)
        plt.close()
        file_paths.append(combined_file)

    return file_paths


def main():
    with open(CSV_PATH, newline="") as csv_file:
        reader = csv.reader(csv_file)
        rows = [row for row in reader if row and any(cell.strip() for cell in row)]

    if not rows:
        raise ValueError(f"No data found in {CSV_PATH}")

    rows_out, metric_groups = build_summary_rows(rows)

    if not rows_out[1:]:
        raise ValueError(f"No valid score rows found in {CSV_PATH}")

    with open(OUTPUT_PATH, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(rows_out)

    print("controller,swarm_size,mean_score,median_score,std_score,n_runs")
    for row in rows_out[1:]:
        print(",".join(row))

    generated = []
    for metric_name in ["robots", "objects"]:
        generated.extend(generate_metric_plots(metric_name, metric_groups))
        generated.extend(generate_variance_plot(metric_name, metric_groups))

    print("\nSaved files:")
    for item in generated:
        print(item)


if __name__ == "__main__":
    main()
