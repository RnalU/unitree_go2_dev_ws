import os
from glob import glob
from setuptools import setup

package_name = 'nav_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装启动文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # 安装参数配置文件
        (os.path.join('share', package_name, 'params'), 
            glob('params/*.yaml')),
        # 安装地图文件
        (os.path.join('share', package_name, 'maps'), 
            glob('maps/*')),
        # 安装RVIZ配置文件
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ymc',
    maintainer_email='Ymc1415481736@outlook.com',
    description='ROS2 Navigation launcher for Unitree robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_launcher = nav_launcher.nav_launcher:main',
            'set_initial_pose = nav_launcher.set_initial_pose:main'
        ],
    },
)
