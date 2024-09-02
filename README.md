## franka utils

Some utils functions for franka emika panda robot.

![image](./520.gif)

## lib Directory Functions

The `lib` directory contains functions that implement algorithms to solve computational problems related to the class material. These include:

- **calcAngDiff.py**: This file contains functions to calculate the angular difference between two orientations.
- **calcJacobian.py**: This file contains functions to calculate the Jacobian matrix for the robot.
- **calcManipulability.py**: This file contains functions to calculate the manipulability of the robot.
- **calculateFK.py**: This file contains functions to calculate the forward kinematics of the Franka Emika Panda robot.
- **FK_velocity.py**: This file contains functions to calculate the forward kinematics velocity of the robot.
- **IK_position_null.py**: This file contains functions to calculate the inverse kinematics with null space optimization for position control.
- **IK_velocity_null.py**: This file contains functions to calculate the inverse kinematics with null space optimization for velocity control.
- **IK_velocity.py**: This file contains functions to calculate the inverse kinematics for velocity control.

## ros Directory Explanation

The `ros` directory contains code necessary to launch the simulator. This includes:

- **Launch Files**: Scripts to start the simulation environment, including the robot model and the Gazebo simulator.
- **Configuration Files**: Settings and parameters for the simulation, such as robot descriptions and environment configurations.
- **ROS Nodes**: Programs that perform specific tasks within the ROS ecosystem, such as sensor data processing, robot control, and communication with other nodes.

You won't need to work in this directory, as it is primarily used to set up and run the simulation environment.
