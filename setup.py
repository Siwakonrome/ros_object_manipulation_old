from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_object_manipulation'],
    package_dir={'': 'src'}
)

setup(**d)