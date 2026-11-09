"""Retrieve the official, separately licensed KolektorSDD archive."""

import argparse
import hashlib
from pathlib import Path
from urllib.request import urlretrieve
from zipfile import ZipFile


URL = "https://data.vicos.si/datasets/KSDD/KolektorSDD.zip"
SHA256 = "65dc621693418585de9c4467d1340ea7958a6181816f0dc2883a1e8b61f9d4dc"


def extract_checked(archive, destination):
    root = destination.resolve()
    with ZipFile(archive) as bundle:
        for member in bundle.infolist():
            target = (root / member.filename).resolve()
            if not target.is_relative_to(root):
                raise ValueError("archive entry escapes dataset directory")
        bundle.extractall(root)


def main():
    parser = argparse.ArgumentParser(description="Download 102 MB CC BY-NC-SA 4.0 dataset; see DATASET.md")
    parser.add_argument("--destination", type=Path, default=Path("datasets"))
    destination = parser.parse_args().destination
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / "KolektorSDD.zip"
    if not archive.exists():
        urlretrieve(URL, archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != SHA256:
        raise ValueError(f"dataset checksum differs: {digest}; no extraction performed")
    extract_checked(archive, destination / "KolektorSDD")
    print(f"Verified SHA-256 {digest}; extracted official dataset")


if __name__ == "__main__":
    main()
