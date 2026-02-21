from setuptools import setup, find_packages

package_name = 'carla_diff_drive_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/diff_drive_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
    ],
    install_requires=['setuptools', 'carla'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bridge_node = carla_diff_drive_bridge.bridge_node:main',
            'spawn_robot = carla_diff_drive_bridge.spawn_robot:main',
        ],
    },
)
