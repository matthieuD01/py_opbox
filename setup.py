from setuptools import setup, find_packages

with open("requirements.txt") as f:
    requirements = f.read().splitlines()

setup(
    name='py_opbox',
    version='1.0.0',
    author='Matthieu Dominici',
    author_email='matthieu.dominici@columbia.edu',
    description='Python library to operate the OpBox locally and save contents to database.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/matthieuD01/py_opbox',
    packages=find_packages(),
    install_requires=requirements,
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.11",
)
