#!/usr/bin/env bash
# diagnose_dup_nodes.sh — 只读诊断“为什么出现重名节点 / 孤儿残留”
#
#   bash diagnose_dup_nodes.sh
#
# 只读：不启动、不停止、不 source 任何 SLAM 树、不发信号。
#
# 注意：本脚本用 rclpy 图 API 普查（tools/node_census_once.py），
# 不用 `ros2 node list` —— 后者依赖 daemon，终端环境里混进其他环境时
# 会在 import 阶段抛异常，统计结果会变成 traceback 里的 "File"/"return"。
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
T=12

say() { printf '\n========== %s ==========\n' "$*"; }

say "1. 节点普查（rclpy 图 API，只读）"
if ! command -v python3 >/dev/null; then
  echo "  找不到 python3"
else
  # rclpy 来自系统 ROS，source 官方 base 不会混入任何 SLAM 树
  ( source /opt/ros/humble/setup.bash 2>/dev/null
    timeout "$T" python3 tools/node_census_once.py ) \
    || echo "  [普查失败/超时，退出码 $?；不重试]"
fi

say "2. desk 链进程（pid / ppid / pgid / 启动时长 / 父进程）"
PAT='complementary_filter_node|lidar_cloud_preprocess_node|static_tf_publisher_node|mount_tf_publisher_node|robot_state_publisher|localization_lite_node|mapping_lite_node'
ps -eo pid=,ppid=,pgid=,etime=,cmd= | grep -E "$PAT" | grep -v grep | \
while read -r pid ppid pgid etime cmd; do
  printf '  pid %-7s ppid %-7s pgid %-7s up %-10s %s\n' \
    "$pid" "$ppid" "$pgid" "$etime" "${cmd:0:140}"
  pcmd=$(ps -o cmd= -p "$ppid" 2>/dev/null)
  printf '        父进程(%s): %s\n' "$ppid" "${pcmd:0:140}"
done

say "3. 每台进程属于哪棵树"
ps -eo pid=,cmd= | grep -E "$PAT" | grep -v grep | \
  awk '{
    if ($0 ~ /\/opt\/slam3d_release/)       tag="SLAM3D Lite 综合包";
    else if ($0 ~ /\/opt\/hivecore/)           tag="甲方机器人驱动栈";
    else                                       tag="其他";
    printf "  %-8s %s\n", $1, tag
  }'

say "4. 若第 1 节失败：ros2 CLI 的真实报错（只用来判断终端环境）"
err=$(timeout "$T" ros2 node list 2>&1 >/dev/null | head -6)
if [[ -n "$err" ]]; then
  echo "  ros2 node list 报错："
  sed 's/^/    /' <<<"$err"
  echo "  → 说明该终端的 ros2 CLI 不可用，不代表节点不在；请以第 1 节为准。"
else
  echo "  （ros2 node list 正常，无报错）"
fi

say "5. 判断"
echo "  · 第 1 节显示重名 >1 组 → 先看第 3 节 ppid 是否为 1。"
echo "    ppid=1 = 父 launch 已死的孤儿，直接停止即可，不会被 respawn。"
echo "  · 第 3b 节列出的是白名单外的重名（例如 N01 导航栈）："
echo "    同一 pgid = 同一次 launch 起了两份；不同 pgid = 两次独立启动。"
echo "    GUI 只负责 SLAM/GUI 进程族，不停止导航栈；请在启动它的终端里停掉较多余的那次。"
echo "  · 若两个 lite 节点同时存在 → 定位与建图 desk 同时在跑，停掉一个。"
echo "  · 若某项来自 /opt/hivecore → 甲方栈自带，不要动；改让 desk 少起同项"
echo "    （例如 launch_robot_state_publisher:=false）。"
say "完成（全程只读）"
