"""hive2_viz.launch.py —— 乙方独立 Hive2 STL 显示链（第一版）。

目标：把 docs/hive2.zip 的真实 STL 模型接入 RViz 独立显示，不带入/不上传/不
冲击甲方 TF、URDF、SLAM、运动栈。所有数据源均为「乙方自产或甲方既有」的只读
消费本链负责把可视化模型放到独立命名空间。

链式数据流
==========
  hive2_viz.urdf.xacro ─(<xacro>渲染)→ robot_description 参数
        │
        ▼
  robot_state_publisher（乙方 RSP，remap /tf→/tf_viz, /tf_static→/tf_viz_static）
        │  发布标准 frame：base_footprint/base_link/fl_hip...×4腿/轮
        ▼
  viz_tf_prefixer（dynacbf_gate 节点） 订阅 /tf_viz(+_static)
        │  每个 frame_id/child 加前缀 "viz_"
        ▼
  公共 /tf + /tf_static  ←– RViz（RobotModel TF Prefix=viz_）
        │
        ▼
  RViz 显示完整 Hive2 模型（base_footprint=贴地根，z 维持原始 0.90）

关键设计
========
* 模型根 = base_footprint（hive2_viz.urdf.xacro → hive2_body → root_mode=base_footprint），
  不写死贴地校正、不改链长（femur=0.3725/tibia=0.4325/below=0.0485 原样）。
* 乙方 RSP 的 TF 走独立话题 /tf_viz，避免与甲方公共 /tf 上同名 frame 竞争。
* RViz RobotModel 的 TF Prefix 必须与 viz_tf_prefixer 的 prefix 一致（=viz_），
  否则模型不会跟随 viz_* TF。
* 静态 TF 走 TRANSIENT_LOCAL，保证 RViz 启动后能收到启动前缓存的静态链。

使用
====
  ros2 launch dynacbf_bringup hive2_viz.launch.py rviz:=true

只启动显示链节点；不启动 /cmd_vel、不启动 A*、MPC、controller、gate、SLAM。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # hive2_description 包：meshes + urdf（原始链长）
    h2_share = get_package_share_directory("hive2_description")
    xacro_path = os.path.join(h2_share, "urdf", "hive2_viz.urdf.xacro")

    rviz = LaunchConfiguration("rviz")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_desc_ns = LaunchConfiguration("robot_desc_topic")
    tf_in = LaunchConfiguration("tf_in")
    tf_static_in = LaunchConfiguration("tf_static_in")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_path]),
            value_type=str,
        )
    }

    # RViz 配置：scan_map（固定帧 odom + RobotModel TF Prefix 由 launch 下发）
    bringup_share = get_package_share_directory("dynacbf_bringup")
    rviz_config = os.path.join(bringup_share, "rviz", "scan_map.rviz")

    declare_args = [
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="是否启动 RViz2 显示（scan_map 配置）。"),
        DeclareLaunchArgument(
            "prefix", default_value="viz_",
            description="显示 TF 前缀（转发器给所有 frame 加此前缀，"
                        "RViz RobotModel TF Prefix 需一致）。"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="真机显示一律 false。"),
        DeclareLaunchArgument(
            "robot_desc_topic", default_value="/robot_description_viz",
            description="乙方独立 URDF 描述话题（不占用甲方 /robot_description）。"),
        DeclareLaunchArgument(
            "tf_in", default_value="/tf_viz",
            description="乙方 RSP 重定向的 TF 输入话题（独立于甲方 /tf）。"),
        DeclareLaunchArgument(
            "tf_static_in", default_value="/tf_viz_static",
            description="乙方 RSP 重定向的 /tf_static 输入话题。"),
    ]

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="hive2_viz_rsp",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time,
                                        "use_tf_static": True}],
        remappings=[
            ("/tf", tf_in),
            ("/tf_static", tf_static_in),
            ("robot_description", robot_desc_ns),
        ],
    )

    prefixer = Node(
        package="dynacbf_gate",
        executable="viz_tf_prefixer",
        name="viz_tf_prefixer",
        output="screen",
        parameters=[{"prefix": prefix,
                     "in_tf": tf_in,
                     "in_tf_static": tf_static_in}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declare_args + [rsp, prefixer, rviz_node])