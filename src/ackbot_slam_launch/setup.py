from setuptools import find_packages, setup

package_name = 'ackbot_slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch file
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
        # Install the RViz configuration file
        #('share/' + package_name + '/rviz', ['rviz/ackbot.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beto',
    maintainer_email='beto@todo.todo',
    description='Launch file for SLAM using SLAM Toolbox and RViz',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
