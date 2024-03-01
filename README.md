# Yaw-Pitch Robotic Arm Simulation

Welcome to the repository for the yaw-pitch robotic arm simulation in C. This project demonstrates the application of forward and inverse kinematics, control dynamics, trajectory planning, and dynamic modeling for the YPP robotic arm. Below is a detailed guide on how to utilize and understand the components of this project.

## Project Structure Overview

### Folders and Key Files

- **CSV Files**: For plotting simulation outputs.
  - `Dynamics.csv`: Stores force and torque dynamics.
  - `Trajectory.csv`: Records the trajectory planning outcomes.
- **Control**: Implements control logic.
  - `Control.c`: PD controller for the robotic arm.
  - `ControlDynamics.c`: Applies control dynamics based on the PD controller output.
- **Kinematics**:
  - `ForwardKinematics.c`: Calculates the end-effector position based on given joint angles.
  - `InverseKinematics.c`: Computes the joint angles required to reach a specified end-effector position.
- **Dynamics.c**: Models the robotic arm's dynamics.
- **Trajectory.c**: Plans the movement trajectory.
- **Makefile**: Compiles the project, generating executables and object files.

### Compilation and Execution

For compiling the project :
```sh
make all
```
This generates executables in the `executable` folder.

For executing the files :
```sh
cd executable
./<executable_name>
```
Run these executables to simulate the robotic arm's behavior. The simulations' results are output to `CSV Files` for dynamics and trajectory.

For deleting the executable and object files in the project :
```sh
make clean
```

## Detailed Guide

### Forward and Inverse Kinematics

- **Forward Kinematics (`ForwardKinematics.c`)**: 
  - Input: Joint angles (yaw, pitch1, pitch2).
  - Output: Position of the end-effector (x, y, z).
  - Usage: Modify the joint angles in the `main` function to simulate different end-effector positions.

- **Inverse Kinematics (`InverseKinematics.c`)**:
  - Input: Desired end-effector position (x, y, z).
  - Output: Required joint angles.
  - Usage: Set the target position in the `main` function to compute the necessary joint angles.

### Dynamics (`Dynamics.c`)

This file models the robotic arm's dynamics, including the calculation of forces and torques required to achieve desired movements.

- Input: Desired states (positions and velocities).
- Output: Forces and torques.
- User-Specified Information: Modify the desired states to simulate different dynamic behaviors.

### Trajectory Planning (`Trajectory.c`)

Generates a trajectory for the robotic arm from a starting to an ending point over a specified time.

- Input: Start and end points, duration.
- Output: Trajectory data including position, velocity, acceleration, and jerk.
- User-Specified Information: Adjust start and end points, and duration as needed for different trajectories.

### Control Logic (`Control.c` and `ControlDynamics.c`)

Implements a Proportional-Derivative (PD) control strategy to achieve desired arm positions and velocities.

- **`Control.c`**: Defines the PD controller gains and desired states.
- **`ControlDynamics.c`**: Applies the control inputs to simulate the arm's motion under PD control.
- User-Specified Information: Update PD gains and desired states based on specific control requirements.

### DH Parameters

While not explicitly included as a separate file, Denavit-Hartenberg (DH) parameters are fundamental to understanding the kinematics of robotic arms. The Forward and Inverse Kinematics implementations are based on these parameters. To adapt the code for different robotic arm configurations, modify the kinematics calculations according to the arm's specific DH parameters.
