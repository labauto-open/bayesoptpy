#! THIS setup.py IS MADE FROM THE TEMPLATE IN THE LINK BELOW
#  duplicating information is fetched from the package.xml
#  http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['optris_ros'],
    package_dir={'': 'src'}
)

setup(**d)
