from glob import glob

from setuptools import find_packages, setup

package_name = 'tram_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hackathon team',
    maintainer_email='team@example.com',
    description='Tram backup odometry ROS 2 nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = tram_odometry.odometry_node:main',
        ],
    },
)
