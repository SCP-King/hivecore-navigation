#!/usr/bin/env python3
"""Read-only persistent ROS graph probe; bounded by monotonic wall time."""
import argparse,os,time
from collections import Counter
import rclpy
from rclpy.node import Node


def missing_nodes(graph, required):
    counts=Counter((namespace.rstrip('/')+'/'+name)for name,namespace in graph)
    return {name:counts[name]for name in required if counts[name]!=1}


def main():
    p=argparse.ArgumentParser();p.add_argument('--launch-pid',type=int,required=True)
    p.add_argument('--timeout',type=float,default=35.);p.add_argument('nodes',nargs='+');args=p.parse_args()
    rclpy.init(args=[]);node=Node('n01_navigation_start_probe_'+str(os.getpid()))
    deadline=time.monotonic()+args.timeout;missing={};began=time.monotonic()
    try:
        while time.monotonic()<deadline:
            try:os.kill(args.launch_pid,0)
            except ProcessLookupError:
                print('ERROR: navigation launch exited during graph check',flush=True);return 5
            rclpy.spin_once(node,timeout_sec=.1)
            missing=missing_nodes(node.get_node_names_and_namespaces(),args.nodes)
            if not missing:
                print(f'PASS: navigation nodes discovered in {time.monotonic()-began:.1f}s',flush=True);return 0
        print('ERROR: navigation nodes missing or duplicated: '+str(missing),flush=True);return 5
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':raise SystemExit(main())
