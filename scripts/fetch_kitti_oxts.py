#!/usr/bin/env python3
"""fetch_kitti_oxts.py — download just the OXTS (GPS/IMU) stream of a
KITTI raw sequence, without pulling the multi-gigabyte camera/LiDAR data.

A KITTI raw `_sync.zip` is ~6 GB, but the OXTS text files inside it total
only a couple of MB and are stored contiguously. This fetches the zip's
central directory with HTTP range requests, then pulls only the byte range
covering the OXTS entries and inflates them locally — typically ~2 MB over
the wire instead of 6 GB.

Usage:
    python3 scripts/fetch_kitti_oxts.py [drive] [out_dir]

    drive    e.g. 2011_09_30_drive_0033   (default)
    out_dir  destination directory        (default ./kitti_<drive>)

Then run the EKF on it:
    ./build/ekf_localization ./kitti_2011_09_30_drive_0033 kitti_ekf.csv

KITTI raw data is © its authors, released for non-commercial research
(see https://www.cvlibs.net/datasets/kitti/). This script only downloads;
respect that license.
"""

import io
import os
import struct
import sys
import urllib.request
import zipfile

BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"


class HttpRangeFile(io.RawIOBase):
    """A seekable read-only file backed by HTTP range requests."""
    def __init__(self, url):
        self.url = url
        self.pos = 0
        self.fetched = 0
        req = urllib.request.Request(url, method="HEAD")
        with urllib.request.urlopen(req, timeout=30) as r:
            self.size = int(r.headers["Content-Length"])

    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos

    def seek(self, off, whence=0):
        self.pos = (off if whence == 0 else
                    self.pos + off if whence == 1 else
                    self.size + off)
        return self.pos

    def read(self, n=-1):
        if n is None or n < 0:
            n = self.size - self.pos
        if n == 0:
            return b""
        end = min(self.pos + n, self.size) - 1
        req = urllib.request.Request(self.url, headers={"Range": f"bytes={self.pos}-{end}"})
        with urllib.request.urlopen(req, timeout=120) as r:
            data = r.read()
        self.pos += len(data)
        self.fetched += len(data)
        return data


def main():
    drive = sys.argv[1] if len(sys.argv) > 1 else "2011_09_30_drive_0033"
    out = sys.argv[2] if len(sys.argv) > 2 else f"kitti_{drive}"
    url = f"{BASE}/{drive}/{drive}_sync.zip"

    print(f"opening {url}")
    f = HttpRangeFile(url)
    zf = zipfile.ZipFile(f)                       # reads EOCD + central dir
    infos = [zi for zi in zf.infolist()
             if "/oxts/" in zi.filename and zi.filename.endswith(".txt")]
    if not infos:
        sys.exit("error: no oxts/*.txt entries found in zip")

    base = min(zi.header_offset for zi in infos)
    end = max(zi.header_offset + 30 + len(zi.filename.encode()) + 256 + zi.compress_size
              for zi in infos)
    span = end - base
    print(f"{len(infos)} oxts files, contiguous span {span/1024/1024:.2f} MB")
    if span > 80 * 1024 * 1024:
        sys.exit("error: oxts entries not contiguous; aborting")

    f.seek(base)
    blob = f.read(end - base)                     # one big range request
    count = 0
    for zi in infos:
        p = zi.header_offset - base
        nlen, elen = struct.unpack("<HH", blob[p + 26:p + 30])
        ds = p + 30 + nlen + elen
        comp = blob[ds:ds + zi.compress_size]
        import zlib
        raw = comp if zi.compress_type == 0 else zlib.decompress(comp, -15)
        rel = zi.filename.split("/oxts/", 1)[1]
        dest = os.path.join(out, "oxts", rel)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with open(dest, "wb") as o:
            o.write(raw)
        count += 1

    print(f"extracted {count} files to {out}/oxts "
          f"({f.fetched/1024/1024:.2f} MB downloaded)")


if __name__ == "__main__":
    main()
