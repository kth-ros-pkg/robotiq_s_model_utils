## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
d = generate_distutils_setup(
    packages=['robotiq_s_model_control_client'],
    package_dir={'': 'src'},
    scripts=['scripts/robotiq_control_client_example_python']
)

setup(**d)
