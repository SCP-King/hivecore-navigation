from glob import glob
from setuptools import setup

package_name = "n01_fast_lio"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="N-01",
    maintainer_email="n01@example.com",
    description="Isolated FAST-LIO2 backend for Hive2 MID-360.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "n01_fast_lio_odometry_adapter = n01_fast_lio.odometry_adapter:main",
        "n01_fast_lio_pose_adapter = n01_fast_lio.pose_adapter:main",
        "n01_fast_lio_relocalizer = n01_fast_lio.relocalizer:main",
    ]},
)
