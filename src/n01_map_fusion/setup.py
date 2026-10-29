from setuptools import setup
from glob import glob
import os

package_name = 'n01_map_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='demo',
    maintainer_email='lyy@local',
    description='N-01 静态地图与实时障碍物融合 + 最小 A* 路径规划(离线验证,只读)',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'fusion_node = n01_map_fusion.fusion_node:main',
            'planner_node = n01_map_fusion.planner_node:main',
            'mock_obstacle = n01_map_fusion.mock_obstacle:main',
            'sim_motion = n01_map_fusion.sim_motion:main',
            'loop_collector = n01_map_fusion.loop_collector:main',
        ],
    },
)