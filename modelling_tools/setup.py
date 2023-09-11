from setuptools import setup, find_packages

setup(
    name='modelling_tools',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'rdflib == 6.3.2',
        'jinja2',
    ],
    author='Vamsi Kalagaturu',
)