from setuptools import find_packages
from setuptools import setup

setup(
    name='insta360_ros_driver',
    version='1.0.0',
    packages=find_packages(
        include=('insta360_ros_driver', 'insta360_ros_driver.*')),
)
