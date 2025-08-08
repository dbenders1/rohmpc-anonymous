from setuptools import setup, find_packages

setup(
    name="rohmpc_data_analysis",
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
