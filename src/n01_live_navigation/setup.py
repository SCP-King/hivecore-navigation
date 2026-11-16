from setuptools import setup, Extension

package_name = "n01_live_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    ext_modules=[Extension(
        package_name + '._hive2_native',
        sources=[package_name + '/hive2_native.cpp'],
        include_dirs=['/opt/hivecore/hive2-motion/1.0.0/install/include'],
        library_dirs=['/opt/hivecore/hive2-motion/1.0.0/install/lib'],
        runtime_library_dirs=['/opt/hivecore/hive2-motion/1.0.0/install/lib'],
        libraries=['motion_controller'], language='c++', extra_compile_args=['-std=c++17'],
    )],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/live_scan.launch.py",
                                               "launch/n01_localization_stack.launch.py"]),
        ("share/" + package_name + "/config", ["../../config/navigation.yaml", "config/live_scan.yaml", "config/slam3d_lite_adapter.yaml", "config/slam3d_lite_mapping_adapter.yaml", "config/mola_localization_overrides.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/n01_live_navigation.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="N-01 contributors",
    maintainer_email="noreply@example.invalid",
    description="Read-only live localization and cloud bridge into SCAN",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "n01_robot_footprint = n01_live_navigation.robot_footprint_node:main",
            "n01_mapping_scan_sampler = n01_live_navigation.mapping_scan_sampler:main",
            "n01_environment_display = n01_live_navigation.environment_display:main",
            "n01_realtime_reference = n01_live_navigation.realtime_reference:main",
            "n01_wheel_joint_state = n01_live_navigation.wheel_joint_state:main",
            "n01_mapless_motion = n01_live_navigation.mapless_motion:main",
            "n01_ground_reference = n01_live_navigation.ground_reference_node:main",
            "n01_localization_adapter = n01_live_navigation.localization_adapter:main",
            "n01_live_scan_bridge = n01_live_navigation.live_scan_bridge:main",
            "n01_live_goal_gate = n01_live_navigation.goal_gate:main",
            "n01_live_runtime_supervisor = n01_live_navigation.runtime_supervisor:main",
            "n01_live_dynamic_obstacle_tracker = n01_live_navigation.dynamic_obstacle_tracker:main",
            "n01_hive2_trajectory_monitor = n01_live_navigation.trajectory_monitor:main",
            "n01_trajectory_display = n01_live_navigation.trajectory_display:main",
            "n01_live_safety_supervisor = n01_live_navigation.safety_supervisor:main",
            "n01_navigation_acceptance = n01_live_navigation.acceptance:main",
        ],
    },
)
