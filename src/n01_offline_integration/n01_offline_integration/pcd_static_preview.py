"""Read-only, bounded PCD XYZ sampling for an RViz background preview.

No coordinate transform, map registration, planning input, or robot control.
A PCD header alone cannot prove it shares the recorded Bag's 'map' frame.
Supports standard uncompressed ASCII and binary PCD with FLOAT XYZ fields.
"""
import math
from pathlib import Path
import struct

MAX_PCD_POINTS = 50_000_000
MAX_PREVIEW_POINTS = 250_000
MAX_HEADER_BYTES = 65536


def _header(file):
    headers = {}
    size = 0
    while True:
        raw = file.readline(2048)
        size += len(raw)
        if not raw or len(raw) >= 2048 or size > MAX_HEADER_BYTES:
            raise ValueError("PCD header missing, oversized or malformed")
        try:
            line = raw.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise ValueError("PCD header is not ASCII") from exc
        if not line or line.startswith("#"):
            continue
        key, _, value = line.partition(" ")
        key = key.upper()
        if not value or key in headers:
            raise ValueError("PCD header missing value or duplicate key")
        headers[key] = value.split()
        if key == "DATA":
            break
    needed = {"FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA"}
    if not needed.issubset(headers):
        raise ValueError("PCD missing mandatory header fields")
    fmt = headers["DATA"][0].lower()
    if fmt not in ("ascii", "binary"):
        raise ValueError("PCD DATA=%s not supported: convert a COPY to binary or ascii" % fmt)
    names = headers["FIELDS"]
    try:
        sizes = list(map(int, headers["SIZE"]))
        types = headers["TYPE"]
        counts = list(map(int, headers.get("COUNT", ["1"] * len(names))))
        count = int(headers["POINTS"][0])
        width = int(headers["WIDTH"][0])
        height = int(headers["HEIGHT"][0])
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError("PCD header dimensions invalid") from exc
    if (not 1 <= count <= MAX_PCD_POINTS or width * height != count or
            len(names) != len(sizes) or len(names) != len(types) or
            len(names) != len(counts) or
            any(c <= 0 or s not in (1, 2, 4, 8) for c, s in zip(counts, sizes))):
        raise ValueError("PCD fields or point dimensions inconsistent")
    offsets = {}
    text_indexes = {}
    offset = text_index = 0
    for name, ptype, psize, field_count in zip(names, types, sizes, counts):
        if name in offsets:
            raise ValueError("duplicate PCD field")
        offsets[name] = (offset, ptype, psize, field_count)
        text_indexes[name] = text_index
        offset += psize * field_count
        text_index += field_count
    for axis in ("x", "y", "z"):
        if axis not in offsets:
            raise ValueError("PCD XYZ fields missing")
        _, ptype, psize, field_count = offsets[axis]
        if ptype != "F" or psize not in (4, 8) or field_count != 1:
            raise ValueError("PCD XYZ must be 32/64-bit floats")
    return dict(format=fmt, count=count, stride=offset, offsets=offsets,
                text_indexes=text_indexes, text_columns=text_index,
                data_offset=file.tell())


def inspect_pcd(path):
    """Read header only; never loads or modifies the original PCD."""
    with Path(path).open("rb") as file:
        return _header(file)


def sample_pcd(path, max_points=MAX_PREVIEW_POINTS):
    if type(max_points) is not int or not 1000 <= max_points <= MAX_PREVIEW_POINTS:
        raise ValueError("invalid preview point limit")
    path = Path(path)
    with path.open("rb") as file:
        meta = _header(file)
        stride_n = max(1, math.ceil(meta["count"] / max_points))
        xyz = []
        if meta["format"] == "binary":
            expected = meta["data_offset"] + meta["count"] * meta["stride"]
            if path.stat().st_size < expected:
                raise ValueError("PCD binary point data truncated")
            axes = []
            for axis in ("x", "y", "z"):
                offset, _, size, _ = meta["offsets"][axis]
                axes.append((offset, "<f" if size == 4 else "<d"))
            for i in range(0, meta["count"], stride_n):
                file.seek(meta["data_offset"] + i * meta["stride"])
                record = file.read(meta["stride"])
                if len(record) != meta["stride"]:
                    raise ValueError("PCD binary point data truncated")
                point = tuple(struct.unpack_from(fmt, record, offset)[0]
                              for offset, fmt in axes)
                if all(math.isfinite(v) for v in point):
                    xyz.append(point)
        else:
            indexes = [meta["text_indexes"][k] for k in ("x", "y", "z")]
            for i in range(meta["count"]):
                raw = file.readline()
                if not raw:
                    raise ValueError("PCD ASCII point data truncated")
                if i % stride_n != 0:
                    continue
                fields = raw.split()
                if len(fields) != meta["text_columns"]:
                    raise ValueError("PCD ASCII point data malformed")
                try:
                    point = tuple(float(fields[idx]) for idx in indexes)
                except ValueError as exc:
                    raise ValueError("PCD ASCII coordinate invalid") from exc
                if all(math.isfinite(v) for v in point):
                    xyz.append(point)
        if not xyz:
            raise ValueError("PCD has no finite sample coordinates")
        meta["sample_stride"] = stride_n
        meta["sampled_points"] = len(xyz)
        meta["bounds_xyz_m"] = [
            [min(p[dim] for p in xyz), max(p[dim] for p in xyz)]
            for dim in range(3)
        ]
        return xyz, meta
