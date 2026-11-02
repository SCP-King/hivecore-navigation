from setuptools import setup

package_name = "n01_live_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/live_scan.launch.py"]),
        ("share/" + package_name + "/config", ["config/live_scan.yaml"]),
        ("share/" + package_name + "/rviz", ["rviz/n01_live_navigation.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="N-01 contributors",
    maintainer_email="noreply@example.invalid",
    description="Read-only live localization and cloud bridge into SCAN",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "n01_live_scan_bridge = n01_live_navigation.live_scan_bridge:main",
            "n01_live_goal_gate = n01_live_navigation.goal_gate:main",
            "n01_live_runtime_supervisor = n01_live_navigation.runtime_supervisor:main",
            "n01_live_dynamic_obstacle_tracker = n01_live_navigation.dynamic_obstacle_tracker:main",
            "n01_hive2_trajectory_monitor = n01_live_navigation.trajectory_monitor:main",
            "n01_live_safety_supervisor = n01_live_navigation.safety_supervisor:main",
            "n01_navigation_acceptance = n01_live_navigation.acceptance:main",
        ],
    },
)
