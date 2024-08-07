# 3-DOF Robotic Arm Simulation

### Complete Mathematical modelling of 3DOF (yaw-pitch-pitch) Robotic arm in `C` with no use of external libraries.

This is the repository for the yaw-pitch robotic arm simulation in C. This project demonstrates the application of forward and inverse kinematics, control dynamics, trajectory planning, and dynamic modeling for the YPP robotic arm. Below is a detailed guide on how to utilize and understand the components of this project.

## Project Structure Overview

### Folders and Key Files

- **CSV Files**: For plotting simulation outputs.
  - `Dynamics.csv`: Stores force and torque dynamics.
  - `Trajectory.csv`: Records the trajectory planning outcomes.
- **Control**: Implements control logic.
  - `Control.c`: PD controller for the robotic arm.
  - `ControlDynamics.c`: Applies control dynamics based on the PD controller output.
  - `GravityCompensated.c`: Implements gravity compensation alongside PD control for the robotic arm.
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

To execute and run these programs in `Linux`, edit the Makefile or in the terminal execute each program with the following command:

```
gcc fileName.c -o executableFileName -lm
./executableFileName
```

## Detailed Guide

### Forward and Inverse Kinematics

- **Forward Kinematics (`ForwardKinematics.c`)**:

  - Input: Joint angles (yaw, pitch1, pitch2).
    Example Input for yaw = 30deg, pitch1 = 45deg, pitch2 = 45deg and all link lengths (L1, L2, L3) = 10units :

  ```
  Enter yaw: 30
  Enter pitch1: 45
  Enter pitch2 : 30
  Enter Link length L1: 10
  Enter Link length L2: 10
  Enter Link length L3: 10
  ```

  - Output: Position of the end-effector (x, y, z).
  - Usage: Enter the joint angles after executing the script to simulate different end-effector positions.

- **Inverse Kinematics (`InverseKinematics.c`)**:
  - Input: Desired end-effector position (x, y, z).
    Example Input for target coordinates (12,14,16) with all link lengths(L1, L2, L3) = 10units
  ```
  Enter link lengths (L1 L2 L3):
  10 10 10
  Enter Target coordinates (x y z):
  12 14 16
  ```
  - Output: Required joint angles.
  - Usage: Enter the target position after executing the script to compute the necessary joint angles.

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

### Control Logic (`Control.c`, `GravityCompensated.c` and `ControlDynamics.c`)

Implements a Proportional-Derivative (PD) control strategy to achieve desired arm positions and velocities.

- **`Control.c`**: Defines the PD controller gains and desired states.
- **`ControlDynamics.c`**: Applies the control inputs to simulate the arm's motion under PD control.
- **`GravityCompensated.c`**: Start at a point and apply the torque that cancels out gravity terms only. Then the robot holds all its angles in simulation.
- User-Specified Information: Update PD gains and desired states based on specific control requirements.

### DH Parameters

While not explicitly included as a separate file, Denavit-Hartenberg (DH) parameters are fundamental to understanding the kinematics of robotic arms. The Forward and Inverse Kinematics implementations are based on these parameters. To adapt the code for different robotic arm configurations, modify the kinematics calculations according to the arm's specific DH parameters.
