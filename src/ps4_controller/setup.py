from setuptools import find_packages, setup

package_name = 'ps4_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/controller_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beto',
    maintainer_email='beto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ps4_joy = ps4_controller.ps4_controller_node:main'
        ],
    },
)
