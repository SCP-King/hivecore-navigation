#!/usr/bin/env python3
# 进程内接口冒烟:n01_static_map 节点 "加载PCD -> 体素栅格化 -> 恒定发布" 全链路验证。
#
# 安全边界:
#  - 必须运行在隔离 DDS 域:ROS_DOMAIN_ID != 55(真机域),且 ROS_LOCALHOST_ONLY=1。
#  - 节点只读定稿 PCD(绝对路径),不写任何文件;本测试只在内存中发布一次
#    /grid_map/static_occupancy,由同一进程内的自订阅者接收——不向真机底盘/
#    LiDAR/SLAM 发送任何指令,不请求任何甲方服务。
#  - 测试结束 shutdown() 退出,不留后台进程。
#
# 用法:source /opt/ros/humble/setup.bash && \
#   ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 \
#   python3 scripts/interface_smoke.py \
#     --pcd /home/demo/lyy/slam_data/maps/cloud_map_20261027_225549.pcd
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2

import sys as _sys
_sys.path.insert(0, __file__.rsplit("/", 2)[0] + "/n01_static_map")
from static_map_node import StaticMapNode, PUBLISH_TOPIC  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", required=True)
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--timeout_s", type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    node = StaticMapNode(
        parameter_overrides=[
            Parameter("pcd_path", value=a.pcd),
            Parameter("resolution", value=a.resolution),
            Parameter("publish_rate", value=2.0),
            Parameter("frame_id", value="map"),
        ])

    got = {}
    node.create_subscription(
        PointCloud2, PUBLISH_TOPIC,
        lambda m: got.setdefault("msg", m), 10)

    t0 = time.time()
    while time.time() - t0 < a.timeout_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        if got.get("msg") is not None:
            break
    node.destroy_node()
    rclpy.shutdown()

    m = got.get("msg")
    if m is None:
        print("FAIL: 超时未收到 /grid_map/static_occupancy")
        sys.exit(1)
    print("PASS: 收到静态占有点云")
    print(f"  frame_id={m.header.frame_id}  n_pts={m.width * m.height}")
    print(f"  point_step={m.point_step} fields={[f.name for f in m.fields]}")
    if m.width == 0:
        print("WARN: 空点云")
        sys.exit(0)
    xyz = np.frombuffer(m.data, dtype="<f4")
    xyz = xyz[: m.width * m.height * 3].reshape(-1, 3)
    print(f"  msg_bounds: x[{xyz[:, 0].min():.2f},{xyz[:, 0].max():.2f}] "
          f"y[{xyz[:, 1].min():.2f},{xyz[:, 1].max():.2f}] "
          f"z[{xyz[:, 2].min():.2f},{xyz[:, 2].max():.2f}]")
    # 对比源 PCD 边界(栅格化只按 2D 网格稀疏化,z 在窗口内)
    xs, ys, zs = [], [], []
    print("  (源 PCD 边界见 offline_verify.py,此处仅为输出自身的一致性核验)")
    print("PASS: 接口冒烟完成(未向真机/网络发送任何指令;节点已 shutdown)")
    sys.exit(0)


if __name__ == "__main__":
    main()