from setuptools import setup
import glob

package_name = 'go2_driver_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob.glob("launch/*.launch.py")),
        ('share/' + package_name + "/params", glob.glob("params/*.yaml")),
        ('share/' + package_name + "/rviz", glob.glob("rviz/*.rviz")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zut-robot',
    maintainer_email='zut-robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = go2_driver_py.driver:main'
        ],
    },
)
