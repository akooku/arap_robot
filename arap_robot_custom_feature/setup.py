from setuptools import find_packages, setup

package_name = 'arap_robot_custom_feature'

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
    description='Multi-map manager for campus navigationn',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_map_manager = arap_robot_custom_feature.multi_map_manager_node:main',
        ],
    },
)
