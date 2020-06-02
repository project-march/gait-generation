#!/usr/bin/env python
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    packages=['march_rqt_gait_generator', 'march_rqt_gait_generator.model'],
    package_dir={'': 'src'},
    scripts=['scripts/march_rqt_gait_generator'],
)

setup(**d)
