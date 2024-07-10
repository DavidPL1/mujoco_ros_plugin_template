# MuJoCo ROS Plugin Template

This repository serves as a template for creating plugins for MuJoCo ROS. It provides a starting point for developing custom plugins that integrate MuJoCo physics simulation with ROS.

## Customizing the Plugin

To customize the plugin for your specific needs, make the following changes:

- Modify the `src/main.cpp` file:
  - Implement your custom plugin logic here.
  - Optionally rename the file.

- Modify the `config/template_example_config.yaml` file:
  - Modify the example config to showcase configuration options.
  - Rename the file.

- Modify the `CMakeLists.txt` file:
  - Modify the package name and add dependencies as needed.
  - Add any additional compilation flags or options.
  - Add any additional source files and libraries to be compiled.
  - Modify includes and linking as needed.

- Modify the `plugin_template.xml` file:
  - Correct the library name
  - Correct the class name and type
  - Correct the plugin_namespace
  - Correct the description

- Modify the `package.xml` file:
  - Correct the package name and dependencies as needed.
  - Correct maintainers and authors
  - Correct the `plugin_template.xml` filename

- Modify the `launch/mujoco_ros_plugin_template.launch` file:
  - Modify the example launch file with the appropriate parameters and configurations.

## Contributing

Contributions to this template repository are welcome. If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This repository is licensed under the [BSD License](LICENSE).
