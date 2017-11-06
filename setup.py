# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license_file = f.read()

setup(
    name='state_space_vehicle_trailer',
    version='0.0.1',
    description='Python package which implements state space model of vehicle and trailer',
    long_description=readme,
    author='Kazumi Malhan',
    author_email='kmalhan7@gmail.com',
    install_requires=['numpy'],
    url='https://github.com/kmalhan/state_space_vehicle_trailer',
    licese=license_file,
    packages=find_packages(exclude='tests')
)
