#!/usr/bin/env python
import os
from glob import glob
from setuptools import setup, find_packages

path = os.path.join(os.path.dirname(__file__), 'src/rosdiscover/version.py')
with open(path, 'r') as f:
    exec(f.read())


setup(
    name='rosdiscover',
    version=__version__,
    description='TODO',
    author='Chris Timperley',
    author_email='ctimperley@cmu.edu',
    url='https://github.com/ChrisTimperley/rosdiscover',
    license='mit',
    python_requires='>=2.7',
    install_requires=[
        'rooibos>=0.4.1',
        'roswire~=1.1.0',
        'typing>=3.6.6',
        'attrs>=18.2.0',
        'dockerblade~=0.4.0',
        'pyyaml'
    ],
    include_package_data=True,
    packages=find_packages('src'),
    package_dir={'': 'src'},
    py_modules=[splitext(basename(path))[0] for path in glob('src/*.py')],
    classifiers=[
        'Natural Language :: English',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6'
    ],
    entry_points = {
        'console_scripts': [
            'rosdiscover = rosdiscover.cli:main',
        ]
    }
)
