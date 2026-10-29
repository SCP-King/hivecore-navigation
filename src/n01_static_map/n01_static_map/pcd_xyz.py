# 只读 PCD 解析(纯 numpy,无 ROS 依赖)。供 offline 验证脚本与 static_map_node 共用。
import numpy as np


def parse_pcd_xyz(path):
    """只读纯 xyz binary PCD -> (xs, ys, zs) numpy float32 arrays。
    PCD header 必须含:POINTS/WIDTH, FIELDS x y z, SIZE 4 4 4, TYPE F F F, COUNT 1 1 1。
    """
    with open(path, "rb") as f:
        raw = f.read()
    data_marker = raw.find(b"DATA")
    if data_marker < 0:
        raise ValueError("PCD missing DATA line")
    head = raw[:data_marker].decode("latin-1")
    lines = head.splitlines()
    def kv(key):
        for ln in lines:
            if ln.startswith(key):
                return ln.split()[1:]
        return None
    size = [int(x) for x in (kv("SIZE") or ["4", "4", "4"])]
    typ = kv("TYPE") or ["F", "F", "F"]
    cnt = [int(x) for x in (kv("COUNT") or ["1", "1", "1"])]
    fields = kv("FIELDS") or []
    points = int((kv("POINTS") or kv("WIDTH") or ["0"])[0])
    if size != [4, 4, 4]:
        raise ValueError(f"unsupported PCD SIZE {size} (expected [4,4,4])")
    if typ != ["F", "F", "F"]:
        raise ValueError(f"unsupported PCD TYPE {typ} (expected FFF)")
    if cnt != [1, 1, 1]:
        raise ValueError(f"unsupported PCD COUNT {cnt} (expected 1 1 1)")
    if fields != ["x", "y", "z"]:
        raise ValueError(f"unsupported PCD fields {fields} (need exactly x y z)")
    bin_start = raw.find(b"\n", data_marker) + 1
    expected = points * 3 * 4
    payload = raw[bin_start:bin_start + expected]
    if len(payload) != expected:
        raise ValueError(f"PCD binary payload {len(payload)} != expected {expected} "
                         f"(points={points})")
    a = np.frombuffer(payload, dtype="<f4")
    if a.size % 3:
        raise ValueError("PCD payload not multiple of 3 floats")
    xyz = a[:points * 3].reshape(points, 3)
    xs, ys, zs = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    good = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    return xs[good], ys[good], zs[good]