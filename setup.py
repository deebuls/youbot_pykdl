#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
#d['packages'] = ['youbot_pykdl', 'youbot_kdl', 'urdf_parser_py']
d['packages'] = ['youbot_pykdl', 'youbot_kdl']
d['package_dir'] = {'': 'src'}

setup(**d)
