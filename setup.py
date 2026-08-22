#!/usr/bin/env python3
"""Install Heron Simulator's reusable pure-Python plant models through catkin."""

import os
import sys

if __name__ == "__main__" and os.environ.get("SETUPTOOLS_USE_DISTUTILS") != "stdlib":
    environment = os.environ.copy()
    environment["SETUPTOOLS_USE_DISTUTILS"] = "stdlib"
    os.execve(
        sys.executable,
        [sys.executable, os.path.abspath(__file__), *sys.argv[1:]],
        environment,
    )

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["models"],
    package_dir={"": "scripts"},
)
setup(**setup_args)
