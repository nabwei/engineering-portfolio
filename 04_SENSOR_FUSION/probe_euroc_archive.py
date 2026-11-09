"""Probe official ZIP directory with bounded HTTP Range; dataset acquisition is unverified."""

import io
import json
from pathlib import Path
from urllib.request import Request, urlopen
from zipfile import ZipFile


URL = "https://www.research-collection.ethz.ch/server/api/core/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/content"
ARCHIVE_SIZE = 6042263426


class HttpRangeFile(io.RawIOBase):
    def __init__(self, url, size):
        self.url, self.size, self.position = url, size, 0
        self.transferred = 0

    def seekable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=0):
        self.position = offset + (self.position if whence == 1 else self.size if whence == 2 else 0)
        if not 0 <= self.position <= self.size:
            raise ValueError("range seek outside archive")
        return self.position

    def read(self, count=-1):
        count = self.size-self.position if count < 0 else min(count, self.size-self.position)
        if count == 0:
            return b""
        if count > 20_000_000 or self.transferred+count > 2_000_000_000:
            raise ValueError("range download budget exceeded")
        start, end = self.position, self.position+count-1
        request = Request(self.url, headers={"Range": f"bytes={start}-{end}", "Accept-Encoding": "identity"})
        with urlopen(request, timeout=30) as response:
            if response.status != 206 or response.headers.get("Content-Range") != f"bytes {start}-{end}/{self.size}":
                raise ValueError("server did not honor exact byte range; full download refused")
            payload = response.read(count+1)
        if len(payload) != count:
            raise ValueError("incomplete or excessive byte range response")
        self.position += count
        self.transferred += count
        return payload


def main():
    archive = HttpRangeFile(URL, ARCHIVE_SIZE)
    with ZipFile(archive) as bundle:
        listing = [{"name": entry.filename, "size": entry.file_size, "compressed_size": entry.compress_size,
                    "offset": entry.header_offset, "compression": entry.compress_type} for entry in bundle.infolist()]
    destination = Path("../08_TEST_RESULTS/sensor_fusion/euroc_archive_directory.json")
    destination.write_text(json.dumps({"url": URL, "bytes_transferred": archive.transferred, "entries": listing}, indent=2), encoding="utf-8")
    print(json.dumps({"bytes_transferred": archive.transferred, "entries": listing[:30]}, indent=2))


if __name__ == "__main__":
    main()
