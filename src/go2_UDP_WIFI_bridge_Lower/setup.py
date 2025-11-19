from setuptools import setup

package_name = 'go2_UDP_WIFI_bridge_Lower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/lower_bridge.example.yaml']),
        ('share/' + package_name + '/launch', ['launch/lower_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=False,
    maintainer='Unitree Bridge',
    maintainer_email='robot@example.com',
    description='Lower-side UDP/Wi-Fi bridge for Unitree Go2 ROS 2 topics',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lower_bridge_node = go2_UDP_WIFI_bridge_Lower.bridge_node:main',
        ],
    },
)
