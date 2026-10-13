#!/usr/bin/env python3
"""Reproduce the M1 TF-tree scenario for the dynacbf audit."""
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

def mk_tf(parent, child, x, y, z):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation = Vector3(x=x, y=y, z=z)
    t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return t

def lookup(buf, target, src):
    try:
        t = buf.lookup_transform(target, src, tf2_ros.Time())
        return "RESOLVED x=%.3f" % (t.transform.translation.x)
    except Exception as e:
        return "FAIL (%s)" % (str(e).split("\n")[0][:70])

def run_case(name, order):
    buf = tf2_ros.Buffer()
    basefp2bl = mk_tf("base_footprint", "base_link", 0.0, 0.0, 0.30)
    bl2lidar  = mk_tf("base_link", "lidar_link", 0.295, 0.0, 0.0)
    odom2bl   = mk_tf("odom", "base_link", 1.0, 0.0, 0.0)
    odom2bfp  = mk_tf("odom", "base_footprint", 1.0, 0.0, 0.0)
    items = {"rsp_bfp2bl": (basefp2bl, True),  "rsp_bl2lidar": (bl2lidar, True),
             "bridge_odom2bl": (odom2bl, False), "bridge_odom2bfp": (odom2bfp, False)}
    print("=== %s (order: %s) ===" % (name, order))
    for key in order.split("|"):
        tr, st = items[key]
        (buf.set_transform_static if st else buf.set_transform)(tr, "test")
    print("   odom->base_link   :", lookup(buf, "base_link", "odom"))
    print("   odom->lidar_link  :", lookup(buf, "lidar_link", "odom"))
    print("   base_footprint->base_link:", lookup(buf, "base_link", "base_footprint"))
    print("   tree:", buf.all_frames_as_string().replace("\n", " ; "))

# M1 base_link variant (historical negative case; do not use for startup)
# m1_preflight_and_run.sh and run_m1_odom.sh both use pose_child_frame:=base_footprint.
run_case("M1 base_link, RSP first(bridge later)", "rsp_bfp2bl|rsp_bl2lidar|bridge_odom2bl")
run_case("M1 base_link, bridge first(RSP later)", "bridge_odom2bl|rsp_bfp2bl|rsp_bl2lidar")
# M1 base_footprint variant (run_m1_odom.sh: pose_child_frame:=base_footprint)
run_case("M1 base_footprint, RSP first", "rsp_bfp2bl|rsp_bl2lidar|bridge_odom2bfp")
run_case("M1 base_footprint, bridge first", "bridge_odom2bfp|rsp_bfp2bl|rsp_bl2lidar")
