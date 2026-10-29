from setuptools import find_packages, setup

_package_name = "n01_static_map"

setup(
    name=_package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + _package_name]),
        ("share/" + _package_name, ["package.xml"]),
        ("share/" + _package_name + "/launch", ["launch/static_map.launch.py"]),
        ("share/" + _package_name + "/config", ["config/static_map.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="乙方 N-01",
    maintainer_email="n01@local",
    description="只读静态地图供给:加载甲方 SLAM 保存的 PCD(纯 xyz),发布为 map 帧静态占用 PointCloud2。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "static_map_node = n01_static_map.static_map_node:main",
        ],
    },
)