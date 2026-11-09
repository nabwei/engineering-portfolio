"""Synthetic regressions for the bounded EuRoC downloader; no reserved sensor data is read."""

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
from zlib import crc32

import pytest

from download_euroc_sequence import (
    DEFAULT_SPEC,
    MEMBER,
    MEMBER_CRC32,
    MEMBER_SIZE,
    OUTER_SIZE,
    TRANSFER_LIMIT,
    URL,
    BudgetedRangeFile,
    SequenceSpec,
    acquire_archive,
    extract_sensor_files,
    parse_arguments,
)


class LocalRangeFile(BudgetedRangeFile):
    """Serves a synthetic outer bundle from memory with the same budget accounting."""

    payload = b""

    def read(self, count=-1):
        count = self.size - self.position if count < 0 else min(count, self.size - self.position)
        if self.prior_transfer_bound + self.transferred + count > TRANSFER_LIMIT:
            raise ValueError("1.5 GB cumulative range transfer budget exceeded")
        chunk = self.payload[self.position:self.position + count]
        self.position += count
        self.transferred += count
        return chunk


def nested_sequence_bytes(tmp_path, name):
    nested = tmp_path / f"{name}.zip"
    with ZipFile(nested, "w") as bundle:
        bundle.writestr("mav0/cam0/data.csv", "#timestamp,filename\n10,10.png\n")
        bundle.writestr("mav0/cam0/data/10.png", b"synthetic left image bytes " * 50)
        bundle.writestr("mav0/cam1/data.csv", "#timestamp,filename\n10,10.png\n")
        bundle.writestr("mav0/cam1/data/10.png", b"synthetic right image bytes " * 50)
        bundle.writestr("mav0/imu0/sensor.yaml", "sensor_type: imu\n")
        bundle.writestr("mav0/state_groundtruth_estimate0/data.csv", "#timestamp\n10,0,0,0\n")
    return nested.read_bytes()


def synthetic_outer(tmp_path, bundle_name, sequence):
    payload = nested_sequence_bytes(tmp_path, sequence)
    outer = tmp_path / f"{bundle_name}.zip"
    with ZipFile(outer, "w", compression=ZIP_DEFLATED) as container:
        container.writestr(f"{bundle_name}/", "")
        container.writestr(f"{bundle_name}/{sequence}/{sequence}.bag", b"rosbag bytes never used " * 200)
        container.writestr(f"{bundle_name}/{sequence}/{sequence}.zip", payload)
    with ZipFile(outer) as container:
        offset = container.getinfo(f"{bundle_name}/{sequence}/{sequence}.zip").header_offset
    return outer.read_bytes(), payload, offset


def local_factory(payload):
    class Remote(LocalRangeFile):
        pass

    Remote.payload = payload
    return lambda prior, url, size: Remote(prior, url, len(payload))


def test_transfer_budget_rejects_before_network_access():
    remote = BudgetedRangeFile(TRANSFER_LIMIT - 5)
    with pytest.raises(ValueError, match="cumulative"):
        remote.read(6)
    assert remote.transferred == 0


def test_zip_escape_is_rejected(tmp_path):
    archive = tmp_path / "fixture.zip"
    destination = tmp_path / "dataset"
    destination.mkdir()
    with ZipFile(archive, "w") as bundle:
        bundle.writestr("mav0/../../outside.png", b"untrusted member")
    with pytest.raises(ValueError, match="escapes"):
        extract_sensor_files(archive, destination)
    assert not (tmp_path / "outside.png").exists()


def test_verified_resume_and_corrupt_existing_file_preserved(tmp_path):
    archive = tmp_path / "fixture.zip"
    destination = tmp_path / "dataset"
    destination.mkdir()
    with ZipFile(archive, "w") as bundle:
        bundle.writestr("mav0/cam0/data.csv", "#timestamp,filename\n10,10.png\n")
        bundle.writestr("mav0/cam0/data/10.png", b"synthetic image bytes")
    first = extract_sensor_files(archive, destination)
    assert first == extract_sensor_files(archive, destination)
    target = destination / "mav0/cam0/data/10.png"
    target.write_bytes(b"corrupted existing file")
    with pytest.raises(ValueError, match="CRC/size"):
        extract_sensor_files(archive, destination)
    assert target.read_bytes() == b"corrupted existing file"


def test_defaults_reproduce_historical_v2_01_command():
    assert URL.endswith("/29d08aeb1c8d9dc5c30497cc538d9cc61f872bb7/vicon_room2.zip")
    assert OUTER_SIZE == 6_013_384_949
    assert MEMBER == "vicon_room2/V2_01_easy/V2_01_easy.zip"
    assert (MEMBER_SIZE, MEMBER_CRC32) == (883_081_407, 0x53D8D375)
    assert (DEFAULT_SPEC.url, DEFAULT_SPEC.outer_size, DEFAULT_SPEC.member) == (URL, OUTER_SIZE, MEMBER)
    assert (DEFAULT_SPEC.member_size, DEFAULT_SPEC.crc32_hex, DEFAULT_SPEC.member_offset) == (MEMBER_SIZE, "53d8d375", None)
    arguments, spec = parse_arguments([])
    assert arguments.destination == Path(__file__).resolve().parents[1] / "datasets" / "V2_01_easy_mirror"
    assert arguments.prior_transfer_upper_bound == 0
    assert (spec.bundle, spec.sequence, spec.member_size, spec.member_crc32, spec.member_offset) == (
        "vicon_room2", "V2_01_easy", MEMBER_SIZE, MEMBER_CRC32, None
    )
    remote = BudgetedRangeFile(0)
    assert (remote.url, remote.size) == (URL, OUTER_SIZE)


def test_cli_selects_other_sequence_with_explicit_member_metadata(tmp_path):
    with pytest.raises(SystemExit):
        parse_arguments(["--bundle", "machine_hall", "--sequence", "MH_02_easy"])
    arguments, spec = parse_arguments([
        "--bundle", "machine_hall", "--sequence", "MH_02_easy", "--member-size", "1292413952",
        "--member-crc32", "05bd2c2f", "--member-offset", "5241192924",
    ])
    assert spec.url.endswith("/29d08aeb1c8d9dc5c30497cc538d9cc61f872bb7/machine_hall.zip")
    assert spec.outer_size == 12_683_729_426
    assert spec.member == "machine_hall/MH_02_easy/MH_02_easy.zip"
    assert (spec.member_size, spec.member_crc32, spec.member_offset) == (1292413952, 0x05BD2C2F, 5241192924)
    assert arguments.destination.name == "MH_02_easy_mirror"
    _, explicit = parse_arguments(["--destination", str(tmp_path), "--member-size", "1", "--member-crc32", "ff"])
    assert (explicit.sequence, explicit.member_size, explicit.member_crc32) == ("V2_01_easy", 1, 0xFF)


@pytest.mark.parametrize("bundle, sequence", [("machine_hall", "V2_01_easy"), ("vicon_room1", "MH_01_easy"), ("vicon_room3", "V2_01_easy")])
def test_bundle_sequence_mismatch_is_rejected(bundle, sequence):
    with pytest.raises(ValueError):
        SequenceSpec(bundle, sequence, 1, 0)


def test_nested_member_is_acquired_verified_and_extracted(tmp_path):
    outer, payload, offset = synthetic_outer(tmp_path, "vicon_room1", "V1_02_medium")
    destination = tmp_path / "dataset"
    destination.mkdir()
    spec = SequenceSpec("vicon_room1", "V1_02_medium", len(payload), crc32(payload), offset)
    archive, integrity, transferred = acquire_archive(destination, 0, spec, local_factory(outer))
    assert archive == destination / "V1_02_medium.zip" and archive.read_bytes() == payload
    assert (integrity["bytes"], integrity["crc32"]) == (len(payload), f"{crc32(payload):08x}")
    assert 0 < transferred < len(outer)
    assert not (destination / "V1_02_medium.zip.partial").exists()
    records = extract_sensor_files(archive, destination)
    assert {r["file"] for r in records} == {"mav0/cam0/data.csv", "mav0/cam0/data/10.png", "mav0/cam1/data.csv",
                                            "mav0/cam1/data/10.png", "mav0/imu0/sensor.yaml",
                                            "mav0/state_groundtruth_estimate0/data.csv"}
    reused = acquire_archive(destination, 0, spec, local_factory(outer))
    assert reused == (archive, integrity, 0)


def test_member_metadata_and_offset_mismatches_fail_before_writing(tmp_path):
    outer, payload, offset = synthetic_outer(tmp_path, "vicon_room1", "V1_02_medium")
    destination = tmp_path / "dataset"
    destination.mkdir()
    with pytest.raises(ValueError, match="metadata does not match"):
        acquire_archive(destination, 0, SequenceSpec("vicon_room1", "V1_02_medium", len(payload) + 1, crc32(payload)), local_factory(outer))
    with pytest.raises(ValueError, match="offset"):
        acquire_archive(destination, 0, SequenceSpec("vicon_room1", "V1_02_medium", len(payload), crc32(payload), offset + 1), local_factory(outer))
    with pytest.raises(KeyError):
        acquire_archive(destination, 0, SequenceSpec("vicon_room1", "V1_03_difficult", len(payload), crc32(payload)), local_factory(outer))
    assert list(destination.iterdir()) == []


def test_existing_partial_and_mismatched_archive_are_preserved(tmp_path):
    outer, payload, offset = synthetic_outer(tmp_path, "vicon_room1", "V1_02_medium")
    destination = tmp_path / "dataset"
    destination.mkdir()
    spec = SequenceSpec("vicon_room1", "V1_02_medium", len(payload), crc32(payload), offset)
    partial = destination / "V1_02_medium.zip.partial"
    partial.write_bytes(b"incomplete earlier attempt")
    with pytest.raises(FileExistsError):
        acquire_archive(destination, 0, spec, local_factory(outer))
    assert partial.read_bytes() == b"incomplete earlier attempt"
    partial.unlink()
    stale = destination / "V1_02_medium.zip"
    stale.write_bytes(b"not the pinned member")
    with pytest.raises(ValueError, match="differs from pinned member"):
        acquire_archive(destination, 0, spec, local_factory(outer))
    assert stale.read_bytes() == b"not the pinned member"


def test_budget_is_enforced_through_the_outer_bundle_reader(tmp_path):
    outer, payload, offset = synthetic_outer(tmp_path, "vicon_room1", "V1_02_medium")
    destination = tmp_path / "dataset"
    destination.mkdir()
    spec = SequenceSpec("vicon_room1", "V1_02_medium", len(payload), crc32(payload), offset)
    with pytest.raises(ValueError, match="cumulative"):
        acquire_archive(destination, TRANSFER_LIMIT - 1, spec, local_factory(outer))
    assert not (destination / "V1_02_medium.zip").exists()
