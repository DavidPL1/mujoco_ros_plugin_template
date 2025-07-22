import mujoco_ros

try:
    from _mujoco_ros_template_python import MujocoRosTemplatePlugin
except ImportError as e:
    print("Could not find mujoco_ros_template plugin python bindings!")
    raise e