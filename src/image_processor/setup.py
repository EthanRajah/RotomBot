from setuptools import setup

package_name = 'image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.darknet_dir'],
    # package_data = {'image_processor.darknet_dir': ['libdarknet.so'],},
    # include_package_data = True,
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
            "state_machine = image_processor.state_machine:main"
        ],
    },
)
