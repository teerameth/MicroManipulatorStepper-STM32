#!/usr/bin/env python3
"""Plot STM32 motor-test MAGDATA records from one or more serial log files."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_records(paths: list[Path]) -> dict[int, dict[str, np.ndarray]]:
    records: dict[int, list[tuple[str, float, int, float, int]]] = {}
    for path in paths:
        for line in path.read_text(errors="replace").splitlines():
            if not line.startswith("MAGDATA,"):
                continue
            fields = line.split(",")
            if len(fields) != 7:
                continue
            try:
                axis = int(fields[1])
                item = (
                    fields[2],
                    float(fields[3]),
                    int(fields[4]),
                    float(fields[5]),
                    int(fields[6]),
                )
            except ValueError:
                continue
            records.setdefault(axis, []).append(item)

    result: dict[int, dict[str, np.ndarray]] = {}
    for axis, rows in records.items():
        result[axis] = {
            "pass": np.asarray([row[0] for row in rows]),
            "command": np.asarray([row[1] for row in rows], dtype=float),
            "raw": np.asarray([row[2] for row in rows], dtype=np.int64),
            "rotor": np.asarray([row[3] for row in rows], dtype=float),
            "status": np.asarray([row[4] for row in rows], dtype=np.uint8),
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path, help="serial logs containing MAGDATA lines")
    parser.add_argument("-o", "--output", type=Path, required=True, help="output PNG path")
    args = parser.parse_args()

    data = load_records(args.logs)
    if not data:
        raise SystemExit("No MAGDATA records found")

    axes = sorted(data)
    fig, panels = plt.subplots(len(axes), 2, figsize=(13, 3.7 * len(axes)), squeeze=False)
    colors = {"F": "#1261a0", "R": "#e07a1f"}

    for row_index, axis in enumerate(axes):
        item = data[axis]
        response_ax, residual_ax = panels[row_index]
        weak = (item["status"] & 0x02) != 0

        fit_mask = item["pass"] == "F"
        if fit_mask.sum() < 2:
            fit_mask = np.ones(len(item["command"]), dtype=bool)
        fit = np.polyfit(item["command"][fit_mask], item["rotor"][fit_mask], 1)
        residual = item["rotor"] - np.polyval(fit, item["command"])

        for direction, label in (("F", "forward"), ("R", "reverse")):
            mask = item["pass"] == direction
            order = np.argsort(item["command"][mask])
            response_ax.plot(
                item["command"][mask][order],
                item["rotor"][mask][order],
                color=colors[direction],
                linewidth=1.2,
                label=label,
            )
            residual_ax.plot(
                item["command"][mask][order],
                residual[mask][order],
                color=colors[direction],
                linewidth=1.0,
                label=label,
            )

        if weak.any():
            response_ax.scatter(
                item["command"][weak], item["rotor"][weak], s=15, color="#d62728",
                zorder=4, label="weak field",
            )
            residual_ax.scatter(
                item["command"][weak], residual[weak], s=15, color="#d62728", zorder=4,
            )

        reference_max = max(1.0, float(item["command"].max()))
        response_ax.plot(
            [0, reference_max],
            [0, reference_max],
            "--",
            color="0.65",
            linewidth=0.8,
            label="ideal 1:1",
        )
        response_ax.set_title(
            f"Axis {axis}: encoder response  "
            f"(fit slope {fit[0]:.4f}, weak {weak.sum()}/{len(weak)})"
        )
        response_ax.set_xlabel("Commanded position from home (deg)")
        response_ax.set_ylabel("Encoder-derived rotor position (deg)")
        response_ax.grid(True, alpha=0.25)
        response_ax.legend(loc="best", fontsize=8)

        residual_ax.axhline(0, color="0.45", linewidth=0.8)
        residual_ax.set_title("Encoder residual after removing linear scale/offset")
        residual_ax.set_xlabel("Commanded position from home (deg)")
        residual_ax.set_ylabel("Residual (deg)")
        residual_ax.grid(True, alpha=0.25)
        residual_ax.legend(loc="best", fontsize=8)

    fig.suptitle(
        "STM32 calibration-style MT6835 sweep\n"
        "Red points are samples with the encoder WEAK_FIELD status bit",
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
