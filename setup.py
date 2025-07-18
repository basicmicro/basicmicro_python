from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="basicmicro",
    version="2.0.9",  # Updated to match __init__.py version
    author="Nathan Scherdin",
    author_email="support@basicmicro.com",
    description="A modernized Python 3 library for controlling Basicmicro motor controllers",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/basicmicro/basicmicro_python",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Programming Language :: Python :: 3.13",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator",
    ],
    python_requires=">=3.6",
    install_requires=[
        "pyserial>=3.4",
    ],
    keywords="robotics, motor control, basicmicro, roboclaw, serial",
)