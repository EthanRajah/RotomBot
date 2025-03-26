from setuptools import setup

package_name = 'api_waypoint'

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
    maintainer='jetson',
    maintainer_email='ethan.rajah@mail.utoronto.ca',
    description='A ROS 2 package that processes waypoints from an API and updates their status',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'api_waypoint_node = api_waypoint.api_waypoint_node:main',
        ],
    },
)