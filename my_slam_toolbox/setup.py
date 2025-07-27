from setuptools import find_packages, setup

package_name = 'my_slam_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/slam_toolbox_launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moksh',
    maintainer_email='moksh@todo.todo',
    description='SLAM Toolbox integration with RPLIDAR A1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
