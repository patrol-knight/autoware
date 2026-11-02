#!/usr/bin/env python3
"""Plot observed topic frequency and stamped-message age from recorder CSV."""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REQUIRED_FIELDS = {"elapsed_s", "topic", "frequency_hz", "latency_ms"}


def read_metrics(path, selected_topics=None):
    data = defaultdict(lambda: {"frequency": ([], []), "latency": ([], [])})
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames or not REQUIRED_FIELDS.issubset(reader.fieldnames):
            raise ValueError(f"CSV must contain: {', '.join(sorted(REQUIRED_FIELDS))}")
        for row in reader:
            topic = row["topic"]
            if selected_topics and topic not in selected_topics:
                continue
            elapsed = float(row["elapsed_s"])
            for column, key in (("frequency_hz", "frequency"), ("latency_ms", "latency")):
                if row[column]:
                    data[topic][key][0].append(elapsed)
                    data[topic][key][1].append(float(row[column]))
    return data


def plot_metrics(data, output):
    if not data or not any(values["frequency"][0] or values["latency"][0]
                           for values in data.values()):
        raise ValueError("No metric samples found in the CSV")

    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    colors = plt.get_cmap("tab10")
    for index, (topic, values) in enumerate(sorted(data.items())):
        color = colors(index % colors.N)
        times, frequencies = values["frequency"]
        if times:
            axes[0].scatter(times, frequencies, s=8, color=color, label=topic)
        times, latencies = values["latency"]
        if times:
            axes[1].scatter(times, latencies, s=8, color=color, label=topic)

    axes[0].set_title("Observed topic frequency (rolling receive window)")
    axes[0].set_ylabel("Hz")
    axes[1].set_title("Message age at recorder (receive time minus header stamp)")
    axes[1].set_ylabel("ms")
    axes[1].set_xlabel("Seconds since recording started")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        if axis.has_data():
            axis.legend(fontsize=8, loc="best")
        else:
            axis.text(0.5, 0.5, "No stamped samples", transform=axis.transAxes,
                      ha="center", va="center")

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="CSV from record_topic_metrics.py")
    parser.add_argument("--output", type=Path, help="PNG path (default: input name with .png)")
    parser.add_argument("--topics", nargs="+", help="Plot only these topic names")
    args = parser.parse_args()
    output = args.output or args.input.with_suffix(".png")
    try:
        data = read_metrics(args.input, set(args.topics) if args.topics else None)
        plot_metrics(data, output)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
