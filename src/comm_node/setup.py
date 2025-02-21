from setuptools import setup

package_name = 'comm_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='ROS 2 communication node for drone commands',
    license='ROTOMBOT URMUM',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm = comm_node.comm:main',
        ],
    },
)
