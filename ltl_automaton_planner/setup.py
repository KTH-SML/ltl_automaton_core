from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

print "CALLING SETUP"

d = generate_distutils_setup(
    packages=['ltl_automaton_planner'],
    package_dir={'': 'src'}
)

setup(**d)
