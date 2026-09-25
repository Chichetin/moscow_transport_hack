from setuptools import find_packages, setup

package_name = 'tram_odometry_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hackathon team',
    maintainer_email='team@example.com',
    description='Tram backup odometry core (no ROS dependencies).',
    license='MIT',
    tests_require=['pytest'],
)
