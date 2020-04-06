from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['table_logger','table_setter_node', 'table_configuration_manager'],
    package_dir={'' : ''}
)

setup(**d)
