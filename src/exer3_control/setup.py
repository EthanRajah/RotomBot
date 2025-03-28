from setuptools import setup

package_name = 'exer3_control'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = exer3_control.flight_controller:main',
            'grabber = exer3_control.grabber:main',  # Added entry point for grabber
            'plotter = exer3_control.plotter:main',
            'obs_grabber = exer3_control.obs_grabber:main',
        ],
    },
)
