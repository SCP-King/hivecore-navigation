import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dynacbf_gate"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NuoDaJia02",
    maintainer_email="NuoDaJia02@hivecore.cn",
    description="HiveCore 定位就绪门控节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gate_node = dynacbf_gate.gate_node:main",
            "pose_tf_bridge = dynacbf_gate.pose_tf_bridge:main",
            "viz_tf_prefixer = dynacbf_gate.viz_tf_prefixer:main",
        ],
    },
)
