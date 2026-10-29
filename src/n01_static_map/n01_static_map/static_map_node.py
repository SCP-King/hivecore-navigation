# Copyright 2026 乙方 N-01. 只读静态地图供给节点。
#
# 职责:把甲方 SLAM 保存的静态 PCD(纯 xyz,**binary** 格式)加载为 map 帧的
# 静态占用 PointCloud2,恒定发布到 /grid_map/static_occupancy。
#
# 铁律对齐(开发运行准则):
#  - 只读:源 PCD 以外部绝对路径参数传入,绝不写入/移动/覆盖斜率数据,
#    中间全部在内存,不落盘。
#  - 数据源:坐标 frame=map 由甲方 SLAM 保存语义决定(保存时为建图坐标系),
#    本节点不做任何 TF 假设,也不发布 TF。
#  - 只发布静态层;动态障碍仍由 /local_pcd 实时链路/n01_scan 维护,
#    本节点不替代实时险情感知。
#
# PCD 解析:仅支持 header 中 FIELDS=SIZE=4 TYPE=F COUNT=1 的纯 xyz binary。
# 占用格 = 把点归入以 resolution 为边长的体素列(忽略 z),每列只发一个
# 随机 z(在该列点云 z 范围内)的代表点。这样既保留 2D 占用语义(N-01 滑窗
# 栅格 3D 是 2D 膨胀),又把地面/墙面/顶部压缩成稀疏代表点。
import math
import random
import struct

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

PUBLISH_TOPIC = "/grid_map/static_occupancy"


def parse_pcd_xyz(path):
    """只读纯 xyz binary PCD -> (xs, ys, zs) numpy arrays (float32).
    PCD header 必须含:POINTS/WIDTH, FIELDS x y z, SIZE 4 4 4, TYPE F F F, COUNT 1 1 1。
    """
    with open(path, "rb") as f:
        raw = f.read()
    data_marker = raw.find(b"DATA")
    if data_marker < 0:
        raise ValueError("PCD missing DATA line")
    head = raw[:data_marker].decode("latin-1")
    lines = head.splitlines()
    size = [int(x) for x in (list(filter(lambda x: x.startswith("SIZE"),
                                         lines)) or [""]).pop().split()[1:]]
    typ = (list(filter(lambda x: x.startswith("TYPE"), lines)) or [""]).pop().split()[1:]
    cnt = [int(x) for x in (list(filter(lambda x: x.startswith("COUNT"), lines)) or [""]).pop().split()[1:]]
    hdr = {}
    for ln in lines:
        sp = ln.split()
        if len(sp) >= 2:
            hdr[sp[0]] = sp[1:]
    points = int(hdr.get("POINTS", hdr.get("WIDTH", ["0"]))[0])
    if size != [4, 4, 4]:
        raise ValueError(f"unsupported PCD SIZE {size} (expected [4,4,4])")
    if typ != ["F", "F", "F"]:
        raise ValueError(f"unsupported PCD TYPE {typ} (expected FFF)")
    if cnt != [1, 1, 1]:
        raise ValueError(f"unsupported PCD COUNT {cnt} (expected 1 1 1)")
    # CAUTION: 'FIELDS' 顺序可能不同,但 xyz 必须连续 4 字节。
    fields = hdr.get("FIELDS", [])
    if fields != ["x", "y", "z"]:
        raise ValueError(f"unsupported PCD fields {fields} (need exactly x y z)")
    bin_start = raw.find(b"\n", data_marker) + 1
    expected = points * 3 * 4
    payload = raw[bin_start:bin_start + expected]
    if len(payload) != expected:
        raise ValueError(
            f"PCD binary payload {len(payload)} != expected {expected} (points={points})")
    a = np.frombuffer(payload, dtype="<f4")
    if a.size % 3:  # 防御:必须以 triples 对齐
        raise ValueError("PCD payload not multiple of 3 floats")
    xyz = a[:points * 3].reshape(points, 3)
    xs = xyz[:, 0]; ys = xyz[:, 1]; zs = xyz[:, 2]
    # 过滤非有限点(不抛出;记数供日志)
    good = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    if not good.all():
        print(f"[n01_static_map] dropped {int((~good).sum())} non-finite points")
    return xs[good], ys[good], zs[good]


class StaticMapNode(Node):
    def __init__(self, **kwargs):
        super().__init__("n01_static_map", **kwargs)
        # ---- parameters ----
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("resolution", 0.05)      # m/体素
        self.declare_parameter("publish_rate", 2.0)     # Hz
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("min_z", -1.5)           # 参与代表的 z 下限(世界系)
        self.declare_parameter("max_z", 3.0)            # 上限
        self.declare_parameter("seed", 0)               # 随机取样种子(确定性可复现)

        self._pcd_path = ""
        self._apply_params()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.pub = self.create_publisher(PointCloud2, PUBLISH_TOPIC, 10)

        if not self._pcd_path:
            self.get_logger().warning(
                "pcd_path 未设置,节点将不发布任何内容。"
                "仅用于检查静态地图供给链。请配置 pcd_path=<绝对路径>。")
            return
        self.load_and_publish()
        rng = random.Random(self._seed)
        self._timer = self.create_timer(1.0 / self._publish_rate,
                                        lambda: self._publish(rng))

    def _apply_params(self):
        path = self.get_parameter("pcd_path").value
        res = self.get_parameter("resolution").value
        if res <= 0:
            raise ValueError("resolution must be > 0")
        if self.get_parameter("min_z").value > self.get_parameter("max_z").value:
            raise ValueError("min_z must be <= max_z")
        self._pcd_path = str(path)
        self._resolution = float(res)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._min_z = float(self.get_parameter("min_z").value)
        self._max_z = float(self.get_parameter("max_z").value)
        self._seed = int(self.get_parameter("seed").value)

    def _on_param_change(self, params):
        for p in params:
            if p.name in ("pcd_path", "resolution", "publish_rate",
                          "frame_id", "min_z", "max_z", "seed"):
                try:
                    self._apply_params()
                    self.get_logger().info(f"parameter {p.name} updated")
                except Exception as e:
                    self.get_logger().error(f"parameter {p.name} invalid: {e}")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def load_and_publish(self):
        t0 = self.get_clock().now()
        try:
            xs, ys, zs = parse_pcd_xyz(self._pcd_path)
        except Exception as e:
            self.get_logger().fatal(f"PCD 加载失败: {e}")
            raise
        self.get_logger().info(
            f"PCD 加载成功: {self._pcd_path}  pts={xs.size} "
            f"bounds x[{xs.min():.2f},{xs.max():.2f}] y[{ys.min():.2f},{ys.max():.2f}] "
            f"z[{zs.min():.2f},{zs.max():.2f}]")
        # ---- 体素化:保留全部 3D 体素中心(不抽取代表点,保持完整形状) ----
        # 按 0.05m 立方体格(整数键)分组,每个格输出格中心一点。
        # 这样地面/墙面/顶部各自落在各自 z 层,避免"整列为一点"把墙折叠到地面。
        ix = np.floor((xs - xs.min()) / self._resolution).astype(np.int64)
        iy = np.floor((ys - ys.min()) / self._resolution).astype(np.int64)
        iz = np.floor((zs - self._min_z) / self._resolution).astype(np.int64)
        key = (ix * 1048576 + iy) * 2048 + iz  # 37 位内唯一(取余),int64 安全
        # 遍历分组:对每个体素格任意取一点(用 unique return_index)
        _, first_idx = np.unique(key, return_index=True)
        ix_o = ix[first_idx]; iy_o = iy[first_idx]; iz_o = iz[first_idx]
        # z 窗口已由 iz 编码保证在 [min_z, max_z);转世界坐标
        cx = xs.min(); cy = ys.min()
        self._out_x = (cx + (ix_o + 0.5) * self._resolution).astype(np.float32)
        self._out_y = (cy + (iy_o + 0.5) * self._resolution).astype(np.float32)
        self._out_z = (self._min_z + (iz_o + 0.5) * self._resolution).astype(np.float32)
        n = self._out_x.size
        self._payload = np.empty((n, 3), dtype="<f4")
        self._payload[:, 0] = self._out_x
        self._payload[:, 1] = self._out_y
        self._payload[:, 2] = self._out_z
        dt = (self.get_clock().now() - t0).nanoseconds / 1e9
        self.get_logger().info(
            f"体素化完成: 独立2D占位列={n}  @res={self._resolution}  "
            f"耗时{dt:.1f}s 发布到 {PUBLISH_TOPIC} (frame={self._frame_id}, "
            f"z窗口[{self._min_z},{self._max_z}])")

    def _publish(self, rng):
        if self._payload is None:
            return
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = 1
        msg.width = self._payload.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = self._payload.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StaticMapNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except Exception as e:
        import sys
        print(f"[n01_static_map] fatal: {e}", file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()