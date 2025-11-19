from setuptools import setup

package_name = 'go2_gmapping_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ymc',
    maintainer_email='Ymc1415481736@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_gmapping_launcher = go2_gmapping_launcher.go2_gmapping_launcher:main'
        ],
    },
)
