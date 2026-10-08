from setuptools import setup

package_name = 'adafruit_rplidar_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert Lowe',
    maintainer_email='you@utm.edu',
    description='RPLidar LaserScan publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rplidar_node = adafruit_rplidar_ros2.rplidar_node:main',
        ],
    },
)
