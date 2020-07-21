#!/usr/bin/env python
import os
from glob import glob
from setuptools import setup, find_packages

path = os.path.join(os.path.dirname(__file__), 'src/rosdiscover/version.py')
with open(path, 'r') as f:
    exec(f.read())

setup(
    version=__version__,
    include_package_data=True
)
