cmake_minimum_required(VERSION 3.8)
project(dynacbf_bringup)

# ---------------------------------------------------------------------------
# dynacbf_bringup 是纯编排包：没有 C++ 代码，不产生任何可执行文件或库。
# 本文件只负责把 launch/config/rviz 安装到 share/dynacbf_bringup 下。
# 每个节点的算法参数仍由拥有该节点的包提供（单一来源）；
# 本包只携带「真机覆盖参数」dynacbf_real.yaml 与真机 RViz 配置。
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()