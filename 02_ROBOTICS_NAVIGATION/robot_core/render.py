"""Render saved robot replay with standard-library SVG; no GUI dependency."""

import csv
import sys
from pathlib import Path


def render(trajectory_path, output_path):
    with Path(trajectory_path).open() as stream:
        samples = list(csv.DictReader(stream))
    elements = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="700" viewBox="0 0 1000 700">',
        '<rect width="1000" height="700" fill="#102030"/>',
        '<text x="45" y="35" fill="white" font-size="22">Differential-drive simulation — deterministic replay</text>',
    ]
    for column, rows in [(13, range(13)), (22, range(7, 20))]:
        for row in rows:
            elements.append(
                f'<rect x="{50 + column * 30}" y="{650 - (row + 1) * 30}" width="30" height="30" fill="#e58661"/>'
            )
    for x_key, y_key, color in [
        ("x", "y", "#5de0ca"),
        ("estimated_x", "estimated_y", "#ffce73"),
    ]:
        points = " ".join(
            f"{50 + float(sample[x_key]) * 100:.2f},{650 - float(sample[y_key]) * 100:.2f}"
            for sample in samples
        )
        elements.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>'
        )
    elements.extend(
        [
            '<text x="50" y="680" fill="white" font-size="15">Teal: truth | Gold: EKF | Orange: obstacles | Units: metres (100 px/m)</text>',
            "</svg>",
        ]
    )
    Path(output_path).write_text("\n".join(elements), encoding="utf-8")


if __name__ == "__main__":
    render(sys.argv[1], sys.argv[2])
