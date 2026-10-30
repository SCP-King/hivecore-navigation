from glob import glob
from setuptools import setup
import os

name = 'n01_offline_integration'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        (os.path.join('share', name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True, license='Apache-2.0',
    maintainer='N-01 contributors', maintainer_email='noreply@example.invalid',
    description='Offline-only TF/readiness/point cloud adapter for SCAN',
    entry_points={'console_scripts': [
        'n01_offline_bridge = n01_offline_integration.bridge_node:main',
        'n01_offline_fixture = n01_offline_integration.fixture_node:main',
        'n01_offline_observer = n01_offline_integration.observer_node:main',
        'n01_dynamic_port = n01_offline_integration.dynamic_port_node:main',
        'n01_historical_bag = n01_offline_integration.historical_bag_node:main',
        'n01_dynamic_tracker = n01_offline_integration.dynamic_tracker_node:main',
        'n01_path_risk = n01_offline_integration.path_risk_node:main',
        'n01_g3_optimizer = n01_offline_integration.g3_solver_node:main',
        'n01_demo_markers = n01_offline_integration.demo_markers_node:main',
        'n01_static_pcd = n01_offline_integration.pcd_static_node:main',
        'n01_g4_reference = n01_offline_integration.g4_reference_node:main',
    ]},
)
