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
# 【定位 TF 补桥（dynacbf_gate/pose_tf_bridge）】
#   现场实测（2026-10-12）：甲方 slam3d_node 的 /localization_status 报
#   registration_output_tf_published=true，但**新起的 tf2 订阅者**取不到
#   map→base_footprint / map→base_link —— TF 树裂成两棵互不连通：
#       {map → sliding_map}（乙方 planner 发）
#       {base_footprint → base_link → 腿/雷达/IMU}（甲方 robot_state_publisher 发）
#   tf2_echo 直接报 "Tf has two or more unconnected trees"。于是
#   controller_node（要 map→base_link 出 /curr_state）、local_map 的 updateTF()、
#   gate_node 的降级判据全部拿不到 map 系位姿。
#   补救：由 pose_tf_bridge 订阅 /robot_pose（map→base_footprint）补出这条 TF 边，
#   TF 树即连通；后续静态 base_footprint→base_link 自动组成 map→base_link。
#   甲方侧修好后把 pose_tf_bridge 的 enable 置 false 或从本 launch 移除即可。
#
# 【真机对接点（都在本文件显式声明）】
#   1) 感知 local_map   ：lidar_topic=/livox/lidar(geo_filtered)，
#                          base_link/lidar_frame=真机 frame，map_frame=map（dynacbf_real.yaml）
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
#   * 甲方点云已起：默认 /livox/lidar/geo_filtered（甲方 SLAM preprocessor 产物，
#     frame=lidar_link）；显式换回 /livox/lidar（frame=livox_frame）需
#     preflight 确认其 X 静态 TF 可解析。
#   * 甲方底盘已起：/cmd_vel 被底盘吸收，/odom ~50Hz。
#   * 速度对齐：本 launch 与甲方底盘 max_linear_speed 都收敛到 ≤ 1.0 m/s。
#   * 甲方消息 overlay 已 source（否则 gate 降级为 TF 新鲜度门控并告警）。
#
# 【用法】
#   真机（甲方 SLAM 定位）:
#     ros2 launch dynacbf_bringup dynacbf_real.launch.py \
#         navi_mode:=1 pose_topic:=/robot_pose rviz:=true
#
#   M1 链路（无 / 不用甲方 SLAM，切到底盘里程计系 odom）:
#     ros2 launch dynacbf_bringup dynacbf_real.launch.py \
#         navi_mode:=1 pose_topic:=/odom nav_frame:=odom \
#         pose_child_frame:=base_footprint gate_require_slam:=false rviz:=true
#   M1 说明：整链在 odom 系（相对起点）跑通，点云/栅格/规划/控制同系；
#   不加载离线 .pcd（planner 用 cloud_is_world 的实时点云），可用于验证
#   感知→规划→MPC→门控→底盘 全链路。目标点请用 RViz 在 odom 系下发。
#   gate_require_slam:=false 时门控不再看甲方定位状态 —— 一旦下发目标
#   机器人就会动，测试前务必确认场地安全（或把车架起来）。
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # ---------------- launch 参数 ----------------
    rviz = LaunchConfiguration("rviz")
    navi_mode = LaunchConfiguration("navi_mode")
    keypoints_file = LaunchConfiguration("keypoints_file")
    pose_topic = LaunchConfiguration("pose_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_frame = LaunchConfiguration("nav_frame")
    pose_child_frame = LaunchConfiguration("pose_child_frame")
    gate_require_slam = LaunchConfiguration("gate_require_slam")
    gate_cmd_out = LaunchConfiguration("gate_cmd_out")
    speed_limit = LaunchConfiguration("speed_limit")
    omega_limit = LaunchConfiguration("omega_limit")

    # RViz 配置按导航 frame 自动选择：
    #   nav_frame:=odom（M1 / 无 SLAM / 底盘里程计系）→ dynacbf_m1.rviz（Fixed Frame=odom，
    #   odom 系有效话题集）
    #   其它 / 默认 map（真机甲方 SLAM）→ dynacbf_real.rviz（Fixed Frame=map）
    # 该条件拼接会随 nav_frame 的动态值求值，因此必须放在 nav_frame 定义之后。
    # 路径用 realpath(__file__) 定位 src 下的 rviz：本工作区是 symlink-install，
    # launch 文件经 install 指向 src；这样新增的 dynacbf_m1.rviz 在未重新
    # colcon build 前也能被直接找到（否则只 build 过一次 install 里没有它）。
    _src_bringup = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    rviz_config = PythonExpression([
        "'", os.path.join(_src_bringup, "rviz", "dynacbf_m1.rviz"),
        "' if '", nav_frame, "' == 'odom' else '",
        os.path.join(_src_bringup, "rviz", "dynacbf_real.rviz"), "'",
    ])

    # 【M1 / 无 SLAM 模式】全局 frame 与位姿子帧的统一覆盖。
    # 默认 nav_frame=map：真机走甲方 SLAM（pose_topic=/robot_pose，
    #   /robot_pose 的 child_frame_id=base_footprint）。
    # 传 nav_frame:=odom 时整链切到底盘里程计系（pose_topic=/odom，
    #   /odom 的 child_frame_id=base_link），不依赖甲方 SLAM —— 即 SOP 的 M1 链路。
    # 这些键被不同节点读取（perception 读 map_frame、planner 读 grid_map.frame_id、
    # controller 读 global_frame、bspline 读 frame_id）；未声明的键会被忽略。
    frame_override = {
        "frame_id": nav_frame,
        "map_frame": nav_frame,
        "global_frame": nav_frame,
        # M1 bridge 挂在甲方已有的 base_footprint；所有消费者必须查询同一挂点。
        # local_map 的参数名是 base_link，controller 的参数名是 base_frame；
        # 二者在 M1 下都统一覆盖为 pose_child_frame，避免再查不存在的 odom->base_link。
        "base_link": pose_child_frame,
        "base_frame": pose_child_frame,
        "grid_map.frame_id": nav_frame,
        # bspline_path_sampler 的位姿来源必须与 pose_topic 同源：
        # 真机 /robot_pose；M1 /odom。不覆盖时它仍读 dynacbf_real.yaml 的
        # /robot_pose —— M1 下拿不到位姿，会退化成"从 s=0 采样名义 B 样条"，
        # 于是 /global_path 参考里程错乱，MPC 会追着错误参考跑（实测表现为原地打转）。
        "odom_topic": pose_topic,
    }

    # 【限速】按 launch 参数统一收敛速度上限（真机默认 1.0 m/s，首次真机试跑可传 0.3）：
    #   planner   : manager.max_vel / optimization.max_vel（B 样条参考速度上限）
    #   MPC       : v_max（合速度）、omega_max（角速度）
    #   controller: max_linear_speed / max_angular_speed（发 /cmd_vel_nav 前再夹一层）
    speed_override = {
        "manager.max_vel": ParameterValue(speed_limit, value_type=float),
        "optimization.max_vel": ParameterValue(speed_limit, value_type=float),
        "v_max": ParameterValue(speed_limit, value_type=float),
        "max_linear_speed": ParameterValue(speed_limit, value_type=float),
        "omega_max": ParameterValue(omega_limit, value_type=float),
        "max_angular_speed": ParameterValue(omega_limit, value_type=float),
    }

    # pose_tf_bridge 启动检测窗口。真机(SLAM 已发 map→base_footprint)保持 3.0s
    # 以便自动识别"别人在发"而停用(避免双发布);M1(甲方从不发 odom→base_link)
    # 传 0.0 立即广播,消除启动时 odom→base_link 缺席的瞬态报错。
    bridge_detect_sec = LaunchConfiguration("bridge_detect_sec")

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
            description="感知输入点云话题。默认甲方 SLAM 栈 preprocessor 的过滤点云"
            "（frame=lidar_link，乙方 local_map 的 updateTF 查 base_link→lidar_link "
            "静态边，实测可解析；2026-10-14 已实机读入成功）。"
            "显式切回 /livox/lidar（甲方 Livox 驱动原始点云，frame=livox_frame）须"
            "先经 preflight 确认 base_link→livox_frame TF 可用，不静默回退。",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="真机一律 false（真实时钟）。",
        ),
        DeclareLaunchArgument(
            "nav_frame",
            default_value="map",
            description="导航全局 frame。真机=map（甲方 SLAM）。无 SLAM 时传 odom 走 M1 "
            "链路（配合 pose_topic:=/odom pose_child_frame:=base_footprint）。",
        ),
        DeclareLaunchArgument(
            "pose_child_frame",
            default_value="base_footprint",
            description="位姿话题的 child_frame_id，用于 pose_tf_bridge 补 TF。"
            "/robot_pose=base_footprint；/odom 的 child_frame 是 base_link，但乙方桥"
            "必须挂甲方地面系 base_footprint（与 RSP base_footprint→base_link 一致），"
            "否则 base_link 双 parent 分裂 TF 树。",
        ),
        DeclareLaunchArgument(
            "gate_require_slam",
            default_value="true",
            description="gate_node 是否必须甲方 LocalizationStatus 判就绪。"
            "真机=true；M1（用 /odom 跑、无 SLAM）传 false，否则门控会一直拦住。",
        ),
        DeclareLaunchArgument(
            "gate_cmd_out",
            default_value="/cmd_vel",
            description="gate_node 的速度出口话题。默认 /cmd_vel（到底盘）；"
            "干跑验证传 /cmd_vel_dryrun —— 整链照常出解但机器人不动。",
        ),
        DeclareLaunchArgument(
            "speed_limit",
            default_value="1.0",
            description="线速度上限 [m/s]。真机=1.0（需求 ≤1.0）；首次真机试跑建议 0.3。",
        ),
        DeclareLaunchArgument(
            "omega_limit",
            default_value="1.25",
            description="角速度上限 [rad/s]。限速试跑时一起压小（如 0.4）。",
        ),
    DeclareLaunchArgument(
            "bridge_detect_sec",
            default_value="3.0",
            description="pose_tf_bridge 静默检测窗口(s)。真机 3.0(识别甲方是否已在发,避免双发布);"
            "M1/无 SLAM 传 0.0 —— 甲方从不发 odom→base_link,立即广播可消除启动瞬态报错。",
        ),
    ]

    # ---------------- 1. 感知：local_map ----------------
    # 订阅甲方点云（默认过滤点云），用 TF(map→base_link, map→lidar_link) 转到 map 系，
    # 发布 gridmap / local_pcd(map系) / for_obs_track。
    # 只读各包 config + real.yaml（覆盖 lidar_topic/base_link/map_frame）。
    local_map = Node(
        package="dynacbf_perception",
        executable="local_map",
        name="local_map",
        output="screen",
        parameters=[perception_yaml, real_yaml,
                    {"lidar_topic": lidar_topic},
                    frame_override],
    )

    # ---------------- 2. 跟踪：obs_kf ----------------
    # /for_obs_track → /obs_predict_pub（每条障碍 kalman_N=25 步、步距 0.25s）。
    # 无声明参数，N/T 硬编码，与 dcbf_params.yaml 对齐。复用仿真节点，无需改动。
    # 预测 Marker 的 frame_id 用 frame_override 统一（真机=map；M1=odom）。
    obs_kf = Node(
        package="dynacbf_tracking",
        executable="obs_kf",
        name="obs_kf",
        output="screen",
        parameters=[{"frame_id": nav_frame}],
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
            frame_override,
            speed_override,
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
        parameters=[dcbf_yaml, real_yaml, frame_override, speed_override],
    )
    controller_node = Node(
        package="dynacbf_dcbf",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[dcbf_yaml, real_yaml, frame_override, speed_override],
        # 输出改发 /cmd_vel_nav，由 gate_node 门控后再转发到底盘 /cmd_vel
        remappings=[("/cmd_vel", "/cmd_vel_nav")],
    )
    bspline_path_sampler = Node(
        package="dynacbf_dcbf",
        executable="bspline_path_sampler",
        name="bspline_path_sampler",
        output="screen",
        parameters=[dcbf_yaml, real_yaml, frame_override, speed_override],
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
             "cmd_vel_out": gate_cmd_out,
             "localization_topic": "/localization_status",
             "pose_topic": pose_topic,
             "map_frame": nav_frame,
             "base_frame": pose_child_frame,
             "require_slam_ready": ParameterValue(gate_require_slam,
                                                  value_type=bool)},
        ],
        remappings=[("/cmd_vel_nav", "/cmd_vel_nav")],
    )

    # ---------------- 6. 定位 TF 补桥：pose_tf_bridge ----------------
    # 订阅 /robot_pose，补出 map→base_footprint 的 TF，修复 TF 树裂成两棵的问题。
    # 启动后先静默检测 detect_sec 秒；发现别人已在发同一变换则自动不启用。
    # 现场甲方 SLAM 的位姿 TF 恢复后，可将 enable 置 false 或删掉本节点。
    # 【M1 数据流重构（2026-10-13）】M1（nav_frame=odom）下 controller/local_map/
    #   gate/planner 全部直接消费甲方 /odom 或 /tf_static，不再查询任何乙方补发
    #   的 odom→base_footprint TF → 桥在北模式下完全冗余，直接禁用（不再发多余 TF）。
    #   真机（nav_frame=map）保留桥补 map→base_footprint，兼容 SLAM 全局系链路。
    pose_tf_bridge = Node(
        package="dynacbf_gate",
        executable="pose_tf_bridge",
        name="pose_tf_bridge",
        output="screen",
        parameters=[
            {"enable": True,
             "pose_topic": pose_topic,
             "parent_frame": nav_frame,
             "child_frame": pose_child_frame,
             "detect_sec": ParameterValue(bridge_detect_sec, value_type=float),
             # 甲方 RSP 的静态链由乙方 tf2 TransformListener 直接订阅 /tf_static。
             # 不重发静态边：重发会把静态边改成当前时间戳，造成点云 TF 外推错误。
             "republish_static": False},
        ],
        # 只随 nav_frame 决定是否启动，不依赖 enable 参数：
        #   M1（nav_frame=odom）→ 完全不起桥（不消耗任何乙方合成 TF）
        #   真机（nav_frame=map）→ 保留桥补 map→base_footprint
        condition=IfCondition(PythonExpression([
            "'true' if '", nav_frame, "' == 'map' else 'false'",
        ])),
    )

    # ---------------- 7. RViz（可选） ----------------
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
                     " navi_mode=", navi_mode, " frame=", nav_frame,
                     " gate_require_slam=", gate_require_slam,
                     " 限速=", speed_limit, "m/s/", omega_limit, "rad/s",
                     " TF 补桥=pose_tf_bridge(", nav_frame, "→", pose_child_frame, ")"]
            ),
            local_map,
            obs_kf,
            planner_node,
            local_planner_node,
            controller_node,
            bspline_path_sampler,
            gate_node,
            pose_tf_bridge,
            rviz_node,
        ]
    )
