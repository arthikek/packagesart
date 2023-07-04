from setuptools import find_packages
from setuptools import setup

setup(
    name='raspimouse_msgs',
    version='1.1.1',
    packages=find_packages(
        include=('raspimouse_msgs', 'raspimouse_msgs.*')),
)
