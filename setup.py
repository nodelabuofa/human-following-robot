#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    packages=['human_following_robot'],
    package_dir={'': 'src'}
)

setup(**d)
