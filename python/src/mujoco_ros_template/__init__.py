# Replace pymujoco_ros_template with the actual plugin bindings library name 
try:
    from pymujoco_ros_template import MujocoRosTemplatePlugin
except ImportError as e:
    print("Could not find mujoco_ros_template plugin python bindings!")
    raise e