"""Reproducible evidence figure from recorded mission and reconstruction outputs."""

import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def visualize(directory):
    cloud = np.loadtxt(directory / "reconstruction.ply", skiprows=7)
    frames = json.loads((directory / "mission_replay.json").read_text())
    report = json.loads((directory / "coverage.json").read_text())
    positions = np.asarray([frame["position_m"] for frame in frames])
    figure = plt.figure(figsize=(12, 6.4))
    figure.subplots_adjust(top=0.79, bottom=0.13, wspace=0.15)
    scene = figure.add_subplot(1, 2, 1, projection="3d")
    selected = cloud[:: max(1, len(cloud) // 4000)]
    scene.scatter(*selected.T, s=1, color="#187f91", label="Depth reconstruction")
    scene.plot(*positions.T, color="#d37536", linewidth=1, label="Recorded camera path")
    scene.set(
        xlabel="x [m]",
        ylabel="y [m]",
        zlabel="z [m]",
        title="Synthetic blade and recorded inspection path",
    )
    scene.set_box_aspect((1, 1, 1.1))
    scene.legend(loc="upper left", fontsize=8)
    camera = figure.add_subplot(1, 2, 2)
    camera.imshow(plt.imread(directory / "camera_020.png"))
    camera.axis("off")
    camera.set_title("Actual rendered RGB frame 20")
    figure.suptitle(
        f"Inspection core: {report['coverage_fraction']:.2%} surface coverage at 4 cm\nPyBullet point-mass plant · calibrated simulator depth · no real-flight accuracy claim",
        fontsize=12,
    )
    figure.savefig(
        directory / "inspection_overview.png",
        dpi=180,
        bbox_inches="tight",
        pad_inches=0.25,
    )
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    visualize(parser.parse_args().directory)
