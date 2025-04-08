from setuptools import find_packages, setup

package_name = 'arap_robot_system_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aix',
    maintainer_email='akooku12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_diff_drive_controller = arap_robot_system_tests.square_diff_drive_controller:main',
            'robot_nodes = arap_robot_system_tests.robot_nodes:main'
        ],
    },
)
