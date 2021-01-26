from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=find_packages(),
    package_dir={'': ''})

setup(**setup_args)