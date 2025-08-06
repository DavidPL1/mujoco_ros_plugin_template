# ! DO NO MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

packages = find_packages("python/src")

d = generate_distutils_setup(
    packages=packages,
    package_dir={"": "python/src"},
    # Registering the entry point lets mujoco_ros know about this plugin
    # change "template" to the name of your plugin
    # and "mujoco_ros_template" to the name of your package
    entry_points={
        "mujoco_ros.plugins": [
            "template = mujoco_ros_template"
        ]
    }
)

setup(**d)
