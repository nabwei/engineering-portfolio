"""Plot completed relative-VIO evidence with full-interval rigid alignment only.

No estimator is run. Each trajectory has its own full-interval SE(3) position
alignment; missing visual poses remain gaps. Segment-specific report metrics
use different alignment fits and must not be read from these error curves.
"""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
from matplotlib import pyplot as plt

from vio.euroc import ground_truth_state, numeric_records
from vio.geometry import rigid_alignment

COLORS = {"Reference": "black", "Relative VIO": "tab:blue", "Stereo only": "tab:orange"}


def load_completed(directory):
    report = json.loads((directory / "report.json").read_text(encoding="utf-8"))
    if "all" not in report.get("metrics", {}):
        raise ValueError("report has not completed scoring; wait for the replay to finish")
    with np.load(directory / "trajectory.npz", allow_pickle=False) as saved:
        stamps, poses, visual = (saved[key] for key in ("timestamps", "poses", "visual"))
    if stamps.ndim != 1 or stamps.dtype.kind not in "iu" or np.any(np.diff(stamps) <= 0):
        raise ValueError("trajectory timestamps must be increasing integer nanoseconds")
    if poses.shape != (len(stamps), 4, 4) or visual.shape != poses.shape:
        raise ValueError("pose arrays must match timestamps with shape Nx4x4")
    if not np.isfinite(poses).all():
        raise ValueError("inertial trajectory contains nonfinite values")
    return report, (stamps, poses, visual)


def align_recorded(trajectory, truth_path):
    stamps, poses, visual = trajectory
    records = numeric_records(truth_path, 17)
    selected = (stamps >= records[0][0]) & (stamps <= records[0][-1])
    stamps, poses, visual = stamps[selected], poses[selected], visual[selected]
    truth = np.array([ground_truth_state(int(stamp), records).position for stamp in stamps])
    fused = rigid_alignment(poses[:, :3, 3], truth)
    available = np.isfinite(visual).all(axis=(1, 2))
    aligned_visual = np.full(truth.shape, np.nan)
    if available.sum() >= 3:
        aligned_visual[available] = rigid_alignment(visual[available, :3, 3], truth[available])
    return stamps, {"Reference": truth, "Relative VIO": fused, "Stereo only": aligned_visual}


def draw_trajectories(figure, positions, boundary_index):
    xy, xyz = figure.add_subplot(221), figure.add_subplot(222, projection="3d")
    for name, points in positions.items():
        xy.plot(points[:, 0], points[:, 1], label=name, color=COLORS[name], linewidth=1.2)
        xyz.plot(*points.T, label=name, color=COLORS[name], linewidth=1.2)
    if boundary_index is not None:
        point = positions["Reference"][boundary_index]
        xy.scatter(*point[:2], marker="x", color="black", s=60, label="Dev/reserved boundary")
        xyz.scatter(*point, marker="x", color="black", s=60)
    xy.set(xlabel="Reference-world x (m)", ylabel="Reference-world y (m)",
           title="XY trajectory — full-interval rigid alignment", aspect="equal")
    xyz.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)", title="XYZ trajectory — same aligned positions")
    xyz.set_box_aspect(np.maximum(np.ptp(positions["Reference"], axis=0), .1))
    xy.legend(fontsize=8)
    xy.grid(alpha=.25)


def mark_partition(axis, elapsed, boundary):
    if elapsed[0] < boundary < elapsed[-1]:
        axis.axvspan(elapsed[0], boundary, color="tab:blue", alpha=.045, label="Development interval")
        axis.axvspan(boundary, elapsed[-1], color="tab:orange", alpha=.065, label="Reserved interval")
        axis.axvline(boundary, color="black", linestyle="--", linewidth=1, label="Dev/reserved boundary")
    axis.set_xlim(elapsed[0], elapsed[-1])
    axis.set_xlabel("Time from first truth-overlapping estimate (s)")
    axis.grid(alpha=.25)


def draw_errors(axis, positions, elapsed, boundary):
    summary = {}
    for name in ("Relative VIO", "Stereo only"):
        errors = np.linalg.norm(positions[name]-positions["Reference"], axis=1)
        finite = np.isfinite(errors)
        if finite.any():
            summary[name] = {"frames": int(finite.sum()), "full_alignment_rmse_m": float(np.sqrt(np.mean(errors[finite]**2)))}
            axis.plot(elapsed, errors, color=COLORS[name], label=f"{name}: RMSE {summary[name]['full_alignment_rmse_m']:.4f} m", linewidth=1.2)
    mark_partition(axis, elapsed, boundary)
    axis.set(ylabel="Aligned position error norm (m)", title="Full-interval alignment; gaps denote missing visual estimates")
    axis.legend(fontsize=8)
    return summary


def draw_innovations(axis, report, stamps, elapsed):
    rows = [row for row in report["diagnostics"] if "nis" in row and stamps[0] <= row["timestamp_ns"] <= stamps[-1]]
    for accepted, color in [(True, "tab:green"), (False, "tab:red")]:
        subset = [row for row in rows if row["accepted"] == accepted]
        times = [(row["timestamp_ns"]-int(stamps[0]))*1e-9 for row in subset]
        axis.scatter(times, [row["nis"] for row in subset], s=8, color=color,
                     label=f"{'Accepted' if accepted else 'Rejected'} ({len(subset)})")
    axis.axhline(22.458, color="black", linestyle=":", label="6-DOF gate: 22.458")
    mark_partition(axis, elapsed, (report["development_end_ns"]-int(stamps[0]))*1e-9)
    axis.set(yscale="symlog", ylabel="Normalized innovation squared (dimensionless)",
             title="Relative-pose update decisions (symmetric-log scale)")
    axis.set_ylim(bottom=0)
    axis.legend(fontsize=8)


def save_figure(output, report, trajectory):
    stamps, positions = trajectory
    elapsed = (stamps-stamps[0])*1e-9
    boundary = (report["development_end_ns"]-int(stamps[0]))*1e-9
    index = int(np.searchsorted(stamps, report["development_end_ns"])) if 0 < boundary < elapsed[-1] else None
    figure = plt.figure(figsize=(15, 11), layout="constrained")
    draw_trajectories(figure, positions, index)
    summary = draw_errors(figure.add_subplot(223), positions, elapsed, boundary)
    draw_innovations(figure.add_subplot(224), report, stamps, elapsed)
    initialization = ("ASSISTED DIAGNOSTIC — NOT QUALIFICATION" if report["initialization"].get("ground_truth_used")
                      else "autonomous initialization (see report assumptions)")
    figure.suptitle(f"Recorded EuRoC stereo + IMU — {initialization}\n"
                   "Separate full-interval SE(3) position fits, no scale fitting; report segment ATE uses separate fits", fontsize=12)
    for extension in ("png", "svg"):
        figure.savefig(output / f"relative_vio.{extension}", dpi=160)
    plt.close(figure)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True, help="Completed trajectory.npz/report.json directory")
    parser.add_argument("--mav0", type=Path, required=True, help="EuRoC mav0 containing reference CSV")
    parser.add_argument("--output", type=Path, required=True, help="Fresh figure directory; existing paths are rejected")
    arguments = parser.parse_args()
    report, trajectory = load_completed(arguments.run)
    truth_path = arguments.mav0 / "state_groundtruth_estimate0/data.csv"
    aligned = align_recorded(trajectory, truth_path)
    arguments.output.mkdir(parents=True, exist_ok=False)
    summary = {"alignment": "Each trajectory independently fitted across its entire truth-overlapping interval; SE(3), no scale", "curves": save_figure(arguments.output, report, aligned)}
    sources = [arguments.run / "trajectory.npz", arguments.run / "report.json", truth_path, Path(__file__)]
    summary["source_sha256"] = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    (arguments.output / "plot_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary["curves"], indent=2))


if __name__ == "__main__":
    main()
