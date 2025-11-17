from setuptools import find_packages
from setuptools import setup

setup(
    name='rqt_example',
    version='0.6.0',
    packages=find_packages(
        include=('rqt_example', 'rqt_example.*')),
)
