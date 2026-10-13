#!/usr/bin/env python3
"""base_link variant with realistic timestamps: RSP static at t0, bridge dynamic live."""
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from rclpy.time import Time as RTime
from rclpy.duration import Duration as RDuration

def mk_tf(parent, child, x, y, z, stamp_sec):
    t = TransformStamped()
    t.header.stamp.sec = stamp_sec
    t.header.stamp.nanosec = 0
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation = Vector3(x=x, y=y, z=z)
    t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return t

def lookup(buf, target, src):
    try:
        t = buf.lookup_transform(target, src, RTime())
        return "RESOLVED x=%.3f y=%.3f z=%.3f" % (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
    except Exception as e:
        return "FAIL (%s)" % (str(e).split("\n")[0][:60])

def run_case(name, dynamic_only_after):
    buf = tf2_ros.Buffer()
    rsp_bfp2bl = mk_tf("base_footprint", "base_link", 0.0, 0.0, 0.900, 1000)   # RSP static
    rsp_bl2lid = mk_tf("base_link", "lidar_link", 0.295, 0.0, 0.0, 1000)       # RSP static
    br_odom2bl = mk_tf("odom", "base_link", 1.0, 0.5, 0.0, 1001)               # bridge live, later
    print("=== %s ===" % name)
    buf.set_transform_static(rsp_bfp2bl, "rsp")
    buf.set_transform_static(rsp_bl2lid, "rsp")
    buf.set_transform(br_odom2bl, "bridge")          # arrives later (realistic)
    print("  after RSP then bridge:")
    print("    odom->base_link :", lookup(buf, "base_link", "odom"))
    print("    odom->lidar_link:", lookup(buf, "lidar_link", "odom"))
    print("    base_footprint->base_link:", lookup(buf, "base_link", "base_footprint"))
    print("    tree:", buf.all_frames_as_string().replace("\n", " ; "))

run_case("base_link variant, RSP static first then live bridge", dynamic_only_after=True)
