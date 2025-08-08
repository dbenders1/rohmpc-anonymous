#!/usr/bin/env python3

# from distutils.core import setup

# from catkin_pkg.python_setup import generate_distutils_setup

# d = generate_distutils_setup(
#     packages=["mpc_model_id_mismatch"], package_dir={"": "src"}
# )

# setup(**d)

from setuptools import setup, find_packages

setup(
    name="mpc_model_id_mismatch",
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
