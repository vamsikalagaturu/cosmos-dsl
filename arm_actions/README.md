### This is a C++ package that contains required libraries and robot models to run the auto-generated code from the DSL.

## Dependencies
- Orocos KDL
- KDL Parser
- GNU Plot
- GLFW3
- MuJoCo

## Libraries
- `gnu_plotter` - A library to plot data using GNU Plot.
- `kinova_mediator` - A library to communicate with the Kinova Gen3 robot.
- `logger` - A library to log data to the terminal and/or a file.
- `math_utils` - A library to perform mathematical operations on KDL variables.
- `monitors` - A library implementing monitors for pre and post conditions of the motion specification.
- `mujoco_env` - A library to simulate the Kinova Gen3 robot in MuJoCo.
- `pid_controller` - A library implementing a PID controller.
- `solver_utils` - A library to interact with the Popov-Vereshchagin Hybrid Dynamics Solver.
- `tf_utils` - A library to perform transformations on KDL variables.
- `utils` - A library containing utility functions.

## urdf
This folder contains the Kinova Gen3 robot model and meshes.

## src
The auto-generated code from the DSL is placed in this folder.