from setuptools import find_packages
from setuptools import setup

setup(
    name='rc_receiver_if',
    version='0.0.0',
    packages=find_packages(
        include=('rc_receiver_if', 'rc_receiver_if.*')),
)
