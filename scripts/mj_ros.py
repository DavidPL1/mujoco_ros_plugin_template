#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import mujoco_ros
from pathlib import Path
import argparse
from IPython import embed

try:
    from _mujoco_ros_template_python import MujocoRosTemplatePlugin
except ImportError as e:
    print("Could not find mujoco_ros_template plugin python bindings!")
    raise e

parser = argparse.ArgumentParser(description="Mujoco ROS Plugin Template", )
parser.add_argument('--viewer', action='store_true', help="Attach viewer")
parser.add_argument('--interactive', action='store_true', help="Embed IPython after initialization")

args, _ = parser.parse_known_args()

def main():
    mj_env = mujoco_ros.MujocoEnv(start_ros_core=False, unpause=False)

    if args.viewer:
        mj_env.attach_viewer(active=True)

    assert len(mj_env.plugins) > 0

    for plugin in mj_env.plugins:
        if isinstance(plugin, MujocoRosTemplatePlugin):
            break

    print(f"Found plugin: {plugin}")

    # Example of setting plugin properties
    plugin.example_bool = True
    plugin.example_int = 42

    print(f"Updated plugin: {plugin}")

    # Example of using the plugin's methods
    # plugin.

    plugin.extended_functionality()

    if args.interactive:
        # Embed in IPython for interactive commands
        print("Entering IPython shell for interactive commands...")
        embed()
    

if __name__ == "__main__":
    main()
    print("Exiting...")
    exit(0)