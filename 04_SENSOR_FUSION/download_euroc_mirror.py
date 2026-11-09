"""Download only data/calibration and a bounded prefix from a public EuRoC mirror."""

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
from zlib import crc32

from probe_euroc_archive import HttpRangeFile

REVISION = "d05bd6a92a5f456562153f83d65805fea27cb57d"
SEQUENCE = "V1_01_easy"
URL = f"https://huggingface.co/datasets/pepijn223/euroc-mirror/resolve/{REVISION}/{SEQUENCE}.zip"
SIZE = 1149702102
SEQUENCES = ["MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
             "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium", "V2_03_difficult"]


def archive_url(sequence):
    if sequence not in SEQUENCES:
        raise ValueError("unknown EuRoC sequence name")
    return f"https://huggingface.co/datasets/pepijn223/euroc-mirror/resolve/{REVISION}/{sequence}.zip"


class CachedRangeFile(HttpRangeFile):
    def __init__(self, url=URL, size=SIZE):
        super().__init__(url, size)
        self.cache_start, self.cache = 0, b""

    def read(self, count=-1):
        count = self.size-self.position if count < 0 else min(count, self.size-self.position)
        if count == 0:
            return b""
        start = self.position
        offset = start-self.cache_start
        if not (0 <= offset and offset+count <= len(self.cache)):
            request_size = min(max(count, 16_777_216), self.size-start)
            if self.transferred+request_size > 1_500_000_000:
                raise ValueError("1.5 GB acquisition budget exceeded")
            self.cache_start, self.cache = start, super().read(request_size)
            offset = 0
        self.position = start+count
        return self.cache[offset:offset+count]


def save_member(bundle, name, destination):
    target = (destination / name).resolve()
    if not target.is_relative_to(destination.resolve()):
        raise ValueError("ZIP path escapes dataset directory")
    entry = bundle.getinfo(name)
    cached = target.read_bytes() if target.is_file() else b""
    if len(cached) == entry.file_size and crc32(cached) == entry.CRC:
        payload = cached
    else:
        payload = bundle.read(name)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    return {"file": name, "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}


def download(destination, frames, sequence=SEQUENCE, archive_size=SIZE, advertised_sha256=None):
    if frames < 3 or frames > 4000:
        raise ValueError("bounded acquisition must request 3..4000 stereo frames")
    if not isinstance(archive_size, int) or archive_size <= 0:
        raise ValueError("archive size must be the mirror's advertised positive byte count")
    url = archive_url(sequence)
    remote = CachedRangeFile(url, archive_size)
    files = []
    with ZipFile(remote) as bundle:
        metadata = [f"mav0/{sensor}/{filename}" for sensor in ["cam0", "cam1", "imu0", "state_groundtruth_estimate0"] for filename in ["data.csv", "sensor.yaml"]]
        files.extend(save_member(bundle, name, destination) for name in metadata)
        truth_rows = (destination / "mav0/state_groundtruth_estimate0/data.csv").read_text().splitlines()
        truth_start = int(next(row.split(",")[0] for row in truth_rows if not row.startswith("#")))
        for camera in ["cam0", "cam1"]:
            names = sorted(name for name in bundle.namelist() if name.startswith(f"mav0/{camera}/data/") and name.endswith(".png") and int(Path(name).stem) >= truth_start)[:frames]
            for index, name in enumerate(names):
                files.append(save_member(bundle, name, destination))
                if index % 25 == 0:
                    print(f"{camera}: {index+1}/{len(names)} images; transferred={remote.transferred}", flush=True)
    if advertised_sha256 is None and sequence == SEQUENCE:
        advertised_sha256 = "a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa"
    manifest = {"sequence": sequence, "source": url, "revision": REVISION, "archive_size": archive_size,
                "mirror_advertised_archive_sha256": advertised_sha256,
                "whole_archive_hash_verified": False, "official_archive_byte_equivalence_verified": False,
                "rights": "Original ETH record: In Copyright - Non-Commercial Use Permitted, http://rightsstatements.org/page/InC-NC/1.0/",
                "provenance": f"community mirror of recorded EuRoC {sequence}; partial ZIP entries checked by ZIP CRC and SHA-256 per file",
                "requested_frames_per_camera": frames,
                "frames_per_camera": sum(entry["file"].startswith("mav0/cam0/data/") for entry in files),
                "bytes_transferred": remote.transferred, "files": files}
    (destination / "acquisition_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"files": len(files), "bytes_transferred": remote.transferred}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", type=Path, default=Path("datasets/V1_01_easy_mirror"))
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--sequence", choices=SEQUENCES, default=SEQUENCE)
    parser.add_argument("--archive-size", type=int, default=SIZE,
                        help="mirror-advertised ZIP byte size for the selected sequence (required for exact Range checks)")
    parser.add_argument("--advertised-sha256", default=None, help="mirror-advertised whole-archive SHA-256 (recorded, not verified)")
    arguments = parser.parse_args()
    download(arguments.destination, arguments.frames, arguments.sequence, arguments.archive_size, arguments.advertised_sha256)
