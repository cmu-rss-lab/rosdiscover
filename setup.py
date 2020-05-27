#!/usr/bin/env python
import os
from glob import glob
from setuptools import setup, find_packages

path = os.path.join(os.path.dirname(__file__), 'src/rosdiscover/version.py')
with open(path, 'r') as f:
    exec(f.read())


setup(
    version=__version__,
    install_requires=[
        'rooibos>=0.4.1',
        'roswire @ git+https://github.com/ChrisTimperley/roswire.git@e87dafdd811b8909f3c49ec0a969529c39330155#egg=roswire'
        'typing>=3.6.6',
        'attrs>=18.2.0',
        'dockerblade~=0.4.0',
        'pyyaml'
    ],
    include_package_data=True,
    entry_points = {
        'console_scripts': [
            'rosdiscover = rosdiscover.cli:main',
        ]
    }
)
