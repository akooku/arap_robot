from setuptools import find_packages
from setuptools import setup

setup(
    name='arap_robot_navigation',
    version='0.0.0',
    packages=find_packages(
        include=('arap_robot_navigation', 'arap_robot_navigation.*')),
)
