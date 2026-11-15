"""Require a real, stable input window before establishing a new map frame."""
from collections import deque
import time
import numpy as np


def stable_window(imu_samples, cloud_stamps, now_ns):
    if len(imu_samples)<100 or len(cloud_stamps)<3:
        return False
    if (now_ns-imu_samples[-1][0])/1e9>.15 or (now_ns-cloud_stamps[-1])/1e9>.35:
        return False
    if not -.2 <= (now_ns-imu_samples[-1][0])/1e9 or not -.2 <= (now_ns-cloud_stamps[-1])/1e9:
        return False
    if (imu_samples[-1][0]-imu_samples[0][0])/1e9<.4:
        return False
    gravity=np.asarray([sample[1] for sample in imu_samples],dtype=float)
    norms=np.linalg.norm(gravity,axis=1)
    if not np.isfinite(gravity).all() or np.any(norms<.1):
        return False
    directions=gravity/norms[:,None]
    return float(np.sqrt(np.var(directions,axis=0).sum()))<.014


def wait_for_mapping_inputs(timeout_s=30.):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,PointCloud2
    context=Context();rclpy.init(args=[],context=context)
    node=Node('n01_mapping_input_warmup',context=context,enable_rosout=False,start_parameter_services=False)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    imu=deque(maxlen=100);cloud=deque(maxlen=5)
    def stamp(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec
    def on_imu(message):
        s=stamp(message);a=message.linear_acceleration
        if s>0 and (not imu or s>imu[-1][0]):imu.append((s,(a.x,a.y,a.z)))
    def on_cloud(message):
        s=stamp(message)
        if s>0 and message.width*message.height>0 and (not cloud or s>cloud[-1]):cloud.append(s)
    subs=[node.create_subscription(Imu,'/imu/data',on_imu,qos_profile_sensor_data),
          node.create_subscription(PointCloud2,'/livox/lidar/geo_filtered',on_cloud,qos_profile_sensor_data)]
    started=time.monotonic();last_log=0.
    try:
        while time.monotonic()-started<timeout_s:
            executor.spin_once(timeout_sec=.05)
            if stable_window(imu,cloud,node.get_clock().now().nanoseconds):
                print('[N-01] Mapping input warmup complete: fresh scans and stable IMU window.',flush=True)
                return
            if time.monotonic()-last_log>3.:
                print(f'[N-01] Waiting for mapping inputs: IMU {len(imu)}/100, scans {len(cloud)}/3; no map frame initialized yet.',flush=True)
                last_log=time.monotonic()
        raise RuntimeError('建图输入未就绪：30秒内未获得新鲜扫描和稳定IMU；请先恢复雷达/IMU后重试')
    finally:
        executor.remove_node(node);node.destroy_node();executor.shutdown();context.shutdown()
