"""Read-only check: raw planning occupancy vs the displayed live_grid."""
import json, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(off):
        return np.empty((0, 3))
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([
        a[:, off['x']:off['x'] + 4].copy().view('<f4').ravel(),
        a[:, off['y']:off['y'] + 4].copy().view('<f4').ravel(),
        a[:, off['z']:off['z'] + 4].copy().view('<f4').ravel(),
    ], axis=1)


class PerceptionConsistency(Node):
    def __init__(self):
        super().__init__('n01_perception_consistency')
        self.radius = float(self.declare_parameter('radius_m', 1.2).value)
        self.cloud = None; self.cloud_t = 0.
        self.live = None; self.live_t = 0.
        self.ref = None; self.ref_t = 0.
        self.pose = None; self.pose_t = 0.
        self.pub = self.create_publisher(String, '/n01/perception/consistency', 10)
        self.create_subscription(PointCloud2, '/grid_map/occupancy',
                                 lambda m: (setattr(self, 'cloud', m), setattr(self, 'cloud_t', time.monotonic())),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/grid_map/live_grid',
                                 lambda m: (setattr(self, 'live', m), setattr(self, 'live_t', time.monotonic())), 10)
        self.create_subscription(OccupancyGrid, '/grid_map/reference_grid',
                                 lambda m: (setattr(self, 'ref', m), setattr(self, 'ref_t', time.monotonic())), 10)
        self.create_subscription(Odometry, '/body_pose',
                                 lambda m: (setattr(self, 'pose', m), setattr(self, 'pose_t', time.monotonic())),
                                 qos_profile_sensor_data)
        self.create_timer(0.5, self.report)

    def report(self):
        now = time.monotonic()
        d = {'schema': 'n01.perception_consistency.v1', 'ok': False, 'reason': 'waiting'}
        if not self.pose or not self.cloud or not self.live:
            d['reason'] = 'missing_input'
        elif now - min(self.cloud_t, self.live_t, self.pose_t) > 1.5:
            d['reason'] = 'stale_input'
        else:
            p = self.pose.pose.pose.position
            c = xyz(self.cloud)
            c = c[np.isfinite(c).all(axis=1)]
            r = np.hypot(c[:, 0] - p.x, c[:, 1] - p.y)
            zband = (c[:, 2] > p.z - .90) & (c[:, 2] < p.z + .60)
            near = c[(r < self.radius) & zband]
            layers = []
            for m, name in ((self.live, 'live'), (self.ref, 'reference')):
                if m is not None:
                    layers.append((m, name))
            matched = 0
            for x, y, _ in near:
                hit = False
                for m, _name in layers:
                    res = m.info.resolution
                    ox, oy = m.info.origin.position.x, m.info.origin.position.y
                    data = np.asarray(m.data).reshape(m.info.height, m.info.width)
                    ix = int(math.floor((x - ox) / res)); iy = int(math.floor((y - oy) / res))
                    if 0 <= ix < m.info.width and 0 <= iy < m.info.height and data[iy, ix] >= 100:
                        hit = True; break
                if hit:
                    matched += 1
            n = int(len(near))
            d.update({'raw_near': n, 'raw_matched': matched,
                      'raw_unmatched': n - matched,
                      'layers': [name for _, name in layers],
                      'ok': n == 0 or matched == n,
                      'reason': 'consistent' if (n == 0 or matched == n) else 'raw_occupancy_not_in_live_or_reference'})
        d['stamp_s'] = now
        self.pub.publish(String(data=json.dumps(d, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args); n = PerceptionConsistency()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__': main()
