"""Acquire one reserved EuRoC sequence by bounded ranges; never evaluate ground truth.

Defaults reproduce the historical V2_01_easy acquisition exactly; other sequences of the same
pinned mirror revision are selected with --bundle/--sequence/--member-size/--member-crc32.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile
from zlib import crc32

from probe_euroc_archive import HttpRangeFile

REVISION = "29d08aeb1c8d9dc5c30497cc538d9cc61f872bb7"
REPOSITORY = "GlowBond/EuRoC_MAV_Dataset"
BUNDLES = {
    "machine_hall": {
        "size": 12_683_729_426,
        "sha256": "5ed7d07903f8d19b6c8808e2ae8a0872b281f6e34ef5497023b8ac58c3de0f6f",
        "prefix": "MH",
    },
    "vicon_room1": {
        "size": 6_042_263_426,
        "sha256": "fe73c27be6dc8ac00493b78b750d36b144daf49eea7fdf3163e934527c1b5297",
        "prefix": "V1",
    },
    "vicon_room2": {
        "size": 6_013_384_949,
        "sha256": "6daf2cbc2de9a6bc4e02866c99ed01c29a5c7c164756f06c4f72656192977cfc",
        "prefix": "V2",
    },
}
DEFAULT_BUNDLE = "vicon_room2"
DEFAULT_SEQUENCE = "V2_01_easy"
URL = f"https://huggingface.co/datasets/{REPOSITORY}/resolve/{REVISION}/{DEFAULT_BUNDLE}.zip"
OUTER_SIZE = BUNDLES[DEFAULT_BUNDLE]["size"]
OUTER_ADVERTISED_SHA256 = BUNDLES[DEFAULT_BUNDLE]["sha256"]
MEMBER = f"{DEFAULT_BUNDLE}/{DEFAULT_SEQUENCE}/{DEFAULT_SEQUENCE}.zip"
MEMBER_SIZE = 883_081_407
MEMBER_CRC32 = 0x53D8D375
TRANSFER_LIMIT = 1_500_000_000


class SequenceSpec:
    """Pinned outer bundle plus the nested member expected for one sequence."""

    def __init__(
        self,
        bundle=DEFAULT_BUNDLE,
        sequence=DEFAULT_SEQUENCE,
        member_size=MEMBER_SIZE,
        member_crc32=MEMBER_CRC32,
        member_offset=None,
    ):
        if bundle not in BUNDLES:
            raise ValueError(f"unknown bundle {bundle!r}")
        if not sequence.startswith(BUNDLES[bundle]["prefix"] + "_"):
            raise ValueError(f"sequence {sequence!r} does not belong to bundle {bundle!r}")
        if member_size <= 0 or not 0 <= member_crc32 <= 0xFFFFFFFF:
            raise ValueError("member size/CRC32 out of range")
        self.bundle, self.sequence = bundle, sequence
        self.member_size, self.member_crc32 = member_size, member_crc32
        self.member_offset = member_offset

    @property
    def url(self):
        return f"https://huggingface.co/datasets/{REPOSITORY}/resolve/{REVISION}/{self.bundle}.zip"

    @property
    def outer_size(self):
        return BUNDLES[self.bundle]["size"]

    @property
    def outer_advertised_sha256(self):
        return BUNDLES[self.bundle]["sha256"]

    @property
    def member(self):
        return f"{self.bundle}/{self.sequence}/{self.sequence}.zip"

    @property
    def crc32_hex(self):
        return f"{self.member_crc32:08x}"


DEFAULT_SPEC = SequenceSpec()


class BudgetedRangeFile(HttpRangeFile):
    def __init__(self, prior_transfer_bound, url=URL, size=OUTER_SIZE):
        super().__init__(url, size)
        self.prior_transfer_bound = prior_transfer_bound

    def read(self, count=-1):
        count = (
            self.size - self.position
            if count < 0
            else min(count, self.size - self.position)
        )
        if self.prior_transfer_bound + self.transferred + count > TRANSFER_LIMIT:
            raise ValueError("1.5 GB cumulative range transfer budget exceeded")
        return super().read(count)


def file_integrity(path):
    sha256, checksum, size = hashlib.sha256(), 0, 0
    with path.open("rb") as stream:
        while chunk := stream.read(8_388_608):
            sha256.update(chunk)
            checksum = crc32(chunk, checksum)
            size += len(chunk)
    return {"bytes": size, "sha256": sha256.hexdigest(), "crc32": f"{checksum:08x}"}


def acquire_archive(
    destination, prior_transfer_bound, spec=DEFAULT_SPEC, remote_factory=None
):
    archive = destination / f"{spec.sequence}.zip"
    if archive.exists():
        integrity = file_integrity(archive)
        if (
            integrity["bytes"] != spec.member_size
            or integrity["crc32"] != spec.crc32_hex
        ):
            raise ValueError(
                "existing archive differs from pinned member; preserve and inspect it"
            )
        return archive, integrity, 0
    if remote_factory is None:
        remote = BudgetedRangeFile(prior_transfer_bound, spec.url, spec.outer_size)
    else:
        remote = remote_factory(prior_transfer_bound, spec.url, spec.outer_size)
    partial = destination / f"{spec.sequence}.zip.partial"
    with ZipFile(remote) as outer:
        entry = outer.getinfo(spec.member)
        if entry.file_size != spec.member_size or entry.CRC != spec.member_crc32:
            raise ValueError("pinned member metadata does not match expected sequence")
        if spec.member_offset is not None and entry.header_offset != spec.member_offset:
            raise ValueError("pinned member offset does not match the probed directory")
        with outer.open(entry) as source, partial.open("xb") as target:
            copied = 0
            while chunk := source.read(8_388_608):
                target.write(chunk)
                copied += len(chunk)
                if copied % 67_108_864 == 0:
                    print(
                        f"{spec.sequence} archive: {copied}/{spec.member_size} bytes; network={remote.transferred}",
                        flush=True,
                    )
    integrity = file_integrity(partial)
    if integrity["bytes"] != spec.member_size or integrity["crc32"] != spec.crc32_hex:
        raise ValueError("extracted nested archive failed size/CRC verification")
    partial.rename(archive)
    return archive, integrity, remote.transferred


def selected_entries(bundle):
    return [
        entry
        for entry in bundle.infolist()
        if not entry.is_dir()
        and entry.filename.startswith("mav0/")
        and (
            entry.filename.endswith(".png")
            or Path(entry.filename).name in ("data.csv", "sensor.yaml")
        )
    ]


def extract_sensor_files(archive, destination):
    records = []
    with ZipFile(archive) as bundle:
        entries = selected_entries(bundle)
        if not entries:
            raise ValueError("archive has no expected EuRoC sensor entries")
        required_space = sum(entry.file_size for entry in entries) + 200_000_000
        if shutil.disk_usage(destination).free < required_space:
            raise OSError("insufficient free space for verified sensor extraction")
        for index, entry in enumerate(entries):
            target = (destination / entry.filename).resolve()
            if not target.is_relative_to(destination.resolve()):
                raise ValueError("ZIP member escapes dataset directory")
            if not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                with bundle.open(entry) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output, length=1_048_576)
            integrity = file_integrity(target)
            if (
                integrity["bytes"] != entry.file_size
                or integrity["crc32"] != f"{entry.CRC:08x}"
            ):
                raise ValueError(
                    f"sensor file failed ZIP CRC/size verification: {entry.filename}"
                )
            records.append({"file": entry.filename, **integrity})
            if index % 500 == 0:
                print(f"Verified {index + 1}/{len(entries)} sensor files", flush=True)
    return records


def timestamp_inventory(destination):
    inventory = {}
    for sensor in ("cam0", "cam1", "imu0", "state_groundtruth_estimate0"):
        timestamps = []
        with (destination / "mav0" / sensor / "data.csv").open() as stream:
            for line in stream:
                if line.strip() and not line.startswith("#"):
                    timestamps.append(int(line.split(",", 1)[0]))
        if not timestamps or any(b <= a for a, b in zip(timestamps, timestamps[1:])):
            raise ValueError(f"{sensor} timestamps are empty or nonmonotonic")
        inventory[sensor] = {
            "records": len(timestamps),
            "first_ns": timestamps[0],
            "last_ns": timestamps[-1],
            "duration_seconds": (timestamps[-1] - timestamps[0]) / 1e9,
        }
    return inventory


def execute(destination, prior_transfer_bound, spec=DEFAULT_SPEC):
    if not 0 <= prior_transfer_bound < TRANSFER_LIMIT:
        raise ValueError("prior transfer bound must lie within the transfer budget")
    destination.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(destination).free < 3_000_000_000:
        raise OSError("need at least 3 GB free before bounded acquisition")
    archive, integrity, transferred = acquire_archive(
        destination, prior_transfer_bound, spec
    )
    records = extract_sensor_files(archive, destination)
    manifest = {
        "sequence": spec.sequence,
        "reserved_for_frozen_frontend_evaluation": True,
        "estimator_executed": False,
        "ground_truth_state_values_inspected": False,
        "source": spec.url,
        "revision": REVISION,
        "bundle": spec.bundle,
        "outer_archive_size": spec.outer_size,
        "outer_archive_advertised_sha256": spec.outer_advertised_sha256,
        "whole_outer_archive_hash_verified": False,
        "official_archive_byte_equivalence_verified": False,
        "nested_member": spec.member,
        "nested_member_offset_cross_checked": spec.member_offset,
        "nested_archive_integrity": integrity,
        "nested_archive_crc_verified": True,
        "nested_archive_sha256_is_computed_not_independently_advertised": True,
        "bytes_transferred_this_run": transferred,
        "prior_attempt_transfer_upper_bound": prior_transfer_bound,
        "cumulative_transfer_upper_bound": prior_transfer_bound + transferred,
        "transfer_budget": TRANSFER_LIMIT,
        "rights": "ETH original record: In Copyright - Non-Commercial Use Permitted",
        "rights_source": "https://www.research-collection.ethz.ch/items/bcaf173e-5dac-484b-bc37-faf97a594f1f",
        "rights_statement": "http://rightsstatements.org/page/InC-NC/1.0/",
        "timestamp_inventory_only": timestamp_inventory(destination),
        "files": records,
    }
    (destination / "acquisition_manifest.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "sequence": manifest["sequence"],
                "files": len(records),
                "network_bytes": transferred,
                "archive": integrity,
                "timestamps": manifest["timestamp_inventory_only"],
            },
            indent=2,
        ),
        flush=True,
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", choices=sorted(BUNDLES), default=DEFAULT_BUNDLE)
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--member-size", type=int, default=None)
    parser.add_argument("--member-crc32", default=None, help="hex CRC32 from the outer directory")
    parser.add_argument("--member-offset", type=int, default=None, help="optional local-header offset cross-check")
    parser.add_argument("--destination", type=Path, default=None)
    parser.add_argument("--prior-transfer-upper-bound", type=int, default=0)
    arguments = parser.parse_args(argv)
    is_default = arguments.bundle == DEFAULT_BUNDLE and arguments.sequence == DEFAULT_SEQUENCE
    if arguments.member_size is None or arguments.member_crc32 is None:
        if not is_default:
            parser.error("--member-size and --member-crc32 are required for a non-default sequence")
        member_size, member_crc32 = MEMBER_SIZE, MEMBER_CRC32
    else:
        member_size, member_crc32 = arguments.member_size, int(arguments.member_crc32, 16)
    spec = SequenceSpec(
        arguments.bundle, arguments.sequence, member_size, member_crc32, arguments.member_offset
    )
    if arguments.destination is None:
        arguments.destination = Path(__file__).parent / "datasets" / f"{spec.sequence}_mirror"
    return arguments, spec


if __name__ == "__main__":
    arguments, spec = parse_arguments()
    execute(arguments.destination, arguments.prior_transfer_upper_bound, spec)
