"""sensor_msgs/PointCloud2 -> (N,3) float32 世界系点(映射字段后转 numpy)。"""
import numpy as np
from sensor_msgs.msg import PointCloud2


def pc2_to_xyz(msg):
    """转 xyz 数组(不要求字段顺序;未知字段按二进制跳过)。"""
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.zeros((0, 3), dtype=np.float32)
    fmt = '<' if msg.is_bigendian == 0 else '>'
    names = list(msg.fields)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    npts = data.size // msg.point_step
    out = np.full((npts, 3), np.nan, dtype=np.float32)
    for fname, axis in (('x', 0), ('y', 1), ('z', 2)):
        f = next((ff for ff in names if ff.name == fname), None)
        if f is None:
            continue
        # 只支持常见小类型:1=INT8/UINT8... 用 uint8 view 精确处理
        dt = np.dtype(_dtype(f, fmt))
        raw = data[:npts * msg.point_step].reshape(npts, msg.point_step)
        col = raw[:, f.offset:f.offset + f.count * 4].copy() if f.datatype == 7 else \
              raw[:, f.offset:f.offset + f.count * dt.itemsize].copy()
        val = col.view(dt).reshape(npts, f.count)
        if f.count == 1:
            out[:, axis] = val[:, 0]
        else:
            out[:, axis] = val.mean(axis=1)  # 多维字段取均值
    out = out[np.all(np.isfinite(out), axis=1)]
    return out.astype(np.float32)


def _dtype(f, endian):
    # sensor_msgs::PointField 常量:1=INT8 2=UINT8 3=INT16 4=UINT16 5=INT32
    #                                 6=UINT32 7=FLOAT32 8=FLOAT64
    return {
        1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8',
    }.get(f.datatype, 'f4')