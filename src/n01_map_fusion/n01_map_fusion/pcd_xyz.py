"""只读纯 xyz PCD 解析(与 n01_static_map.pcd_xyz 同源,无 ROS 依赖)。

只用于离线脚本与单元测试;在线融合节点仅消费主题,不读 PCD 文件。
"""
import numpy as np


def parse_pcd_xyz(path):
    """解析只有 x y z 三个 float32 字段的 binary PCD。

    返回 (xs, ys, zs) float32 数组(已丢弃非有限值);解析失败抛 ValueError。
    """
    with open(path, 'rb') as f:
        raw = f.read()
    head_end = raw.find(b'\nDATA')
    if head_end < 0:
        raise ValueError('PCD 无 DATA 字段')
    header = raw[:head_end].decode('ascii', errors='replace')
    lines = header.splitlines()
    fields = np.array([])
    size = np.array([], dtype=int)
    typ = []
    cnt = np.array([], dtype=int)
    for ln in lines:
        if ln.startswith('FIELDS') or ln.startswith('SIZE') or \
           ln.startswith('TYPE') or ln.startswith('COUNT'):
            rest = ln.split()[1:]
            if ln.startswith('FIELDS'):
                fields = np.asarray(rest)
            elif ln.startswith('SIZE'):
                size = np.asarray(rest, dtype=int)
            elif ln.startswith('TYPE'):
                typ = rest
            elif ln.startswith('COUNT'):
                cnt = np.asarray(rest, dtype=int)
    width = None
    height = None
    for ln in lines:
        if ln.startswith('WIDTH'):
            width = int(ln.split()[1])
        if ln.startswith('HEIGHT'):
            height = int(ln.split()[1])
    if width is None or height is None:
        raise ValueError('PCD 缺 WIDTH/HEIGHT')
    if list(fields) != ['x', 'y', 'z'] or list(size) != [4, 4, 4] or \
       list(typ) != ['F', 'F', 'F'] or list(cnt) != [1, 1, 1]:
        raise ValueError(f'PCD 字段需为 FIELDS x y z 升,实际 {list(fields)}')
    total = width * height
    # 数据体起点:跳过 "DATA .." 头行(\nDATA 定位 -> 该行结束的换行之后)
    body0 = raw[head_end + 1:]             # 从 'DATA ...' 行首开始
    nl = body0.find(b'\n')                 # DATA 行结束
    if nl < 0:
        raise ValueError('PCD DATA 行未闭合')
    data_line = body0[:nl]
    if b'binary' not in data_line:
        raise ValueError(f'PCD 须为 binary,实际 {data_line!r}')
    body = body0[nl + 1:]                  # 跳过整行 -> 二进制体
    arr = np.frombuffer(body[:total * 12], dtype=np.float32).reshape(-1, 3)
    if arr.shape[0] < total:
        raise ValueError('PCD 数据不足')
    xyz = arr[np.all(np.isfinite(arr), axis=1)]
    return xyz[:, 0], xyz[:, 1], xyz[:, 2]