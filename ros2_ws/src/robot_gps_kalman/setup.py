from setuptools import setup
from glob import glob

package_name = 'robot_gps_kalman'

setup(
    name=package_name,
    version='0.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Robot GPS Filter',
    maintainer_email='robot@example.com',
    description='Online GPS Kalman and robot status classifier for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_filter = robot_gps_kalman.gps_filter_node:main',
        ],
    },
)
