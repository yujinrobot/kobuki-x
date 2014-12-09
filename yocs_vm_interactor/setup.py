#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=['yocs_vm_interactor'],
                             package_dir={'': 'src'},
                             requires=['rospy'])

setup(**d)
