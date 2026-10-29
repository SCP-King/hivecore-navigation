from setuptools import setup
from glob import glob
import os

package_name = 'n01_scan_dynacbf_bridge'
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
    maintainer='N-01 contributors',
    maintainer_email='noreply@example.invalid',
    description='Test-only SCAN B-spline to DynaCBF reference adapter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'scan_bspline_bridge = n01_scan_dynacbf_bridge.sampler_node:main',
        'dcbf_test_guard = n01_scan_dynacbf_bridge.command_guard_node:main',
        'bridge_smoke_feed = n01_scan_dynacbf_bridge.smoke_feed:main',
    ]},
)
