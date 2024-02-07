# Robot Arm Controller

## Overview
The `robot_arm` package enables control of a robotic arm via external HTTP calls. It includes a simulation environment using SDF for a virtual representation of the robot arm.

## Features
- **HTTP Control**: Control the robot arm through HTTP requests.
- **Simulation**: Simulate the robot arm in a virtual environment using SDF.

## Usage
### Build
In the container, at the path `/home/user/robot_arm_ws`, run the following command:
```
colcon build --packages-select robot_arm
``````

- To control the real robot arm:
- To launch the simulation:
In the container, at the path `/home/user/robot_arm_ws`, run the following command:
```
colcon build --packages-select robot_arm
```

For simulation, run the following command:
```
ros2 launch robot_arm robot_arm_sim.launch.py
```

## Simulation Details
The simulation uses SDF files for the virtual world and robot model. The simulation can be configured and launched using the provided launch files.

## Configuration
Configuration files can be found under the `config` directory. Modify these files to change the robot's parameters and behavior.

