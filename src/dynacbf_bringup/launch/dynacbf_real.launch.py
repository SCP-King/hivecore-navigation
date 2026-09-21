#!/usr/bin/env python3
# ============================================================================
# dynacbf_real.launch.py —— hivecore-navigation 真机（HiveCore）总装入口
# ============================================================================
# 【本仓库是真机版：不包含仿真】
#   本 launch 只起乙方导航层（感知/跟踪/规划/MPC-D-CBF/定位门控），把它"夹"在
#   甲方系统中间：
#     * 上游：甲方 SLAM 定位（map TF + /robot_pose + /localization_status）
#              + 甲方 Livox 点云
#     * 下游：甲方底盘（/cmd_vel → Hive2 全向轮，/odom 反馈）
#   本文件**不启动** Gazebo、turtlebot3、pseudo_odom、move_test 等仿真组件。
#
# 【数据闭环（真机）】
#   /livox/lidar(geo_filtered) → local_map → /for_obs_track → obs_kf
#     → /obs_predict_pub → planner(sensor_pose=/robot_pose) → /planning/bspline
#     → bspline_path_sampler → /global_path → local_planner_node → /local_plan
#     → controller_node → /cmd_vel_nav → gate_node(定位就绪门控) → /cmd_vel → 甲方底盘
#   定位：甲方 SLAM 发布 map TF，controller/local_map/planner 统一读 map frame。
#
# 【定位就绪门控（dynacbf_gate）】
#   controller_node 不再直连底盘：其输出改发 /cmd_vel_nav，由 gate_node 在甲方
#   LocalizationStatus 确认稳定（跟踪模式 + 位姿/TF 已发布 + 拟合误差≤门限）时才
#   透传到底盘 /cmd_vel；否则发布零速，避免定位未就绪时机器人误动。
#
# 【真机对接点（都在本文件显式声明）】
#   1) 感知 local_map   ：lidar_topic=/livox/lidar(geo_filtered)，
#                          base_link/base_scan=真机 frame，map_frame=map（dynacbf_real.yaml）
#   2) 规划 planner_node：sensor_pose/body_pose = 甲方定位 /robot_pose（或传的 pose_topic）；
#                          cloud 仍 remap 到 local_map 的 local_pcd（map 系点云）。
#   3) controller_node  ：frame_id/base_frame 参数化（读 dynacbf_real.yaml 的 /** 块）；
#                         限速 + 命令超时按需求收紧；输出重映射到 /cmd_vel_nav。
#   4) bspline_path_sampler：odom_topic=/robot_pose，frame_id=map（dynacbf_real.yaml）。
#   5) gate_node        ：cmd_vel_in=/cmd_vel_nav，cmd_vel_out=/cmd_vel（底盘）。
#
# 【运行前提（真机侧，由现场 SOP 先保证）】
#   * 甲方 SLAM 已加载离线 .pcd 并定位成功：map→base_link TF 连续不跳变；
#     /robot_pose（或 /localization_status）有数据。现场无 /robot_pose 时，
#     传 pose_topic:=/odom（相对起点，先验证 M1 链路）。
#   * 甲方 Livox 点云已起：/livox/lidar 有数据；若起预处理则有 /livox/lidar/geo_filtered。
#   * 甲方底盘已起：/cmd_vel 被底盘吸收，/odom ~50Hz。
#   * 速度对齐：本 launch 与甲方底盘 max_linear_speed 都收敛到 ≤ 1.0 m/s。
#   * 甲方消息 overlay 已 source（否则 gate 降级为 TF 新鲜度门控并告警）。
#
# 【用法】
#   ros2 launch dynacbf_bringup dynacbf_real.launch.py \
#       navi_mode:=1 pose_topic:=/robot_pose rviz:=true
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    perception_share = get_package_share_directory("dynacbf_perception")
    planner_share = get_package_share_directory("dynacbf_planner")
    bringup_share = get_package_share_directory("dynacbf_bringup")

    # ---------------- 参数文件 ----------------
    # 各算法节点继续读各自包的 config（单一来源）；bringup 自带的 real.yaml
    # 只覆盖「真机对接项」——把它放在每包 config 之后，值以 real.yaml 为准。
    perception_yaml = os.path.join(perception_share, "config", "config.yaml")
    planner_yaml = os.path.join(planner_share, "config", "planner.yaml")
    keypoints_default = os.path.join(planner_share, "config", "keypoints.demo.yaml")
    dcbf_yaml = os.path.join(
        get_package_share_directory("dynacbf_dcbf"), "config", "dcbf_params.yaml"
    )
    real_yaml = os.path.join(bringup_share, "config", "dynacbf_real.yaml")

    # RViz 用本包自带的真机配置（Fixed Frame=map）
    rviz_config = os.path.join(bringup_share, "rviz", "dynacbf_real.rviz")

    # ---------------- launch 参数 ----------------
    rviz = LaunchConfiguration("rviz")
    navi_mode = LaunchConfiguration("navi_mode")
    keypoints_file = LaunchConfiguration("keypoints_file")
    pose_topic = LaunchConfiguration("pose_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="是否启动 RViz2（用 2D Goal Pose 下发目标，navi_mode=1）。",
        ),
        DeclareLaunchArgument(
            "navi_mode",
            default_value="1",
            description="规划器导航模式：1=等待 move_base_simple/goal（默认，RViz 下发）；"
            "2=预设航点；3=reference_path（外部全局路径）。",
        ),
        DeclareLaunchArgument(
            "keypoints_file",
            default_value=keypoints_default,
            description="navi_mode:=2 使用的航点参数 YAML（顶层键 dynacbf_planner_node）。",
        ),
        DeclareLaunchArgument(
            "pose_topic",
            default_value="/robot_pose",
            description="定位位姿话题（nav_msgs/Odometry，map 系）。甲方 SLAM 提供 /robot_pose；"
            "M1 阶段可用 /odom 先验证链路。",
        ),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/livox/lidar/geo_filtered",
            description="感知输入点云话题。无预处理时改 /livox/lidar。",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="真机一律 false（真实时钟）。",
        ),
    ]

    # ---------------- 1. 感知：local_map ----------------
    # 订阅甲方点云（默认过滤点云），用 TF(map→base_link, map→base_scan) 转到 map 系，
    # 发布 gridmap / local_pcd(map系) / for_obs_track。
    # 只读各包 config + real.yaml（覆盖 lidar_topic/base_link/map_frame）。
    local_map = Node(
        package="dynacbf_perception",
        executable="local_map",
        name="local_map",
        output="screen",
        parameters=[perception_yaml, real_yaml,
                    {"lidar_topic": lidar_topic}],
    )

    # ---------------- 2. 跟踪：obs_kf ----------------
    # /for_obs_track → /obs_predict_pub（每条障碍 kalman_N=25 步、步距 0.25s）。
    # 无声明参数，N/T 硬编码，与 dcbf_params.yaml 对齐。复用仿真节点，无需改动。
    obs_kf = Node(
        package="dynacbf_tracking",
        executable="obs_kf",
        name="obs_kf",
        output="screen",
    )

    # ---------------- 3. 规划层：dynacbf_planner_node ----------------
    # cloud ← local_pcd（map 系点云）
    # sensor_pose / body_pose ← 甲方定位 odometry（默认 /robot_pose）
    # 参数：planner.yaml + keypoints + real.yaml（grid_map.frame_id=map）+ navi_mode 覆盖
    planner_node = Node(
        package="dynacbf_planner",
        executable="dynacbf_planner_node",
        name="dynacbf_planner_node",
        output="screen",
        remappings=[
            ("cloud", "local_pcd"),
            ("sensor_pose", pose_topic),
            ("body_pose", pose_topic),
        ],
        parameters=[
            planner_yaml,
            keypoints_file,
            real_yaml,
            {"fsm.navi_mode": ParameterValue(navi_mode, value_type=int)},
        ],
    )

    # ---------------- 4. MPC-D-CBF 三节点 ----------------
    # 直接声明（不 include 各自 launch，规避 params_file 作用域缺陷，见仿真版注释）。
    # 参数：各包 config + real.yaml（frame_id=map、odom_topic=/robot_pose、
    #        command_timeout/max_linear_speed）。
    local_planner_node = Node(
        package="dynacbf_dcbf",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[dcbf_yaml, real_yaml],
    )
    controller_node = Node(
        package="dynacbf_dcbf",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[dcbf_yaml, real_yaml],
        # 输出改发 /cmd_vel_nav，由 gate_node 门控后再转发到底盘 /cmd_vel
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
    )
    bspline_path_sampler = Node(
        package="dynacbf_dcbf",
        executable="bspline_path_sampler",
        name="bspline_path_sampler",
        output="screen",
        parameters=[dcbf_yaml, real_yaml],
    )

    # ---------------- 5. 定位就绪门控：gate_node ----------------
    # controller 输出 /cmd_vel_nav → gate_node → 底盘 /cmd_vel。
    # 甲方 LocalizationStatus 确认稳定（且新鲜、TF 连续）后才放行，否则零速。
    gate_node = Node(
        package="dynacbf_gate",
        executable="gate_node",
        name="gate_node",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("dynacbf_gate"),
                "config", "gate_params.yaml"),
            {"cmd_vel_in": "/cmd_vel_nav",
             "cmd_vel_out": "/cmd_vel",
             "localization_topic": "/localization_status"},
        ],
        remappings=[("/cmd_vel_nav", "/cmd_vel_nav")],
    )

    # ---------------- 6. RViz（可选） ----------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        declare_args
        + [
            LogInfo(
                msg=["[dynacbf_bringup] 真机导航：感知 → 跟踪 → 规划 → MPC-D-CBF "
                     "→ 定位门控 → /cmd_vel。",
                     " 定位=", pose_topic, " 点云=", lidar_topic,
                     " navi_mode=", navi_mode]
            ),
            local_map,
            obs_kf,
            planner_node,
            local_planner_node,
            controller_node,
            bspline_path_sampler,
            gate_node,
            rviz_node,
        ]
    )
