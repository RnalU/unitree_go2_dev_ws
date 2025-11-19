from setuptools import setup
import os
from glob import glob

package_name = 'unitree_topic_relay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YMC',
    maintainer_email='your_email@example.com',
    description='Topic relay node for Unitree Go2 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_relay_node = unitree_topic_relay.topic_relay_node:main',
        ],
    },
)
