#include "ControlDynamics.c" // Make sure this path is correct
#include <math.h>
#include <stdio.h>

// PD Controller Gains
double Kp[3] = {1200, 1271, 1344}; // Proportional gains
double Kd[3] = {70, 72, 74};       // Derivative gains

// Desired state (positions and velocities)
double th0dds[3] = {1.0, 1.0, 1.0}; // Example desired positions
double th1dds[3] = {0.0, 0.0, 0.0}; // Example desired velocities

void calculateControlTorques(const RobotState *state, double *tau) {
  for (int i = 0; i < 3; i++) {
    // PD Control Law: tau = Kp * (desired_pos - current_pos) + Kd *
    // (desired_vel - current_vel)
    tau[i] = Kp[i] * (th0dds[i] - state->q[i]) +
             Kd[i] * (th1dds[i] - state->qdot[i]);

    // Print the control torque equation for each joint
    printf("%f*th0dds%d + %f*th1dds%d + th2dds%d - %f*x%d - %f*x%d\n", Kp[i],
           i + 1, Kd[i], i + 1, i + 1, Kp[i], i + 1, Kd[i], i + 4);
  }
}

int main() {
  // Initialize current state
  RobotState currentState = {{0, -0.387, 0.45}, {0, 0, 0}};
  // Control torques for each joint
  double tau[3];

  // Calculate control torques based on the current state and desired state
  calculateControlTorques(&currentState, tau);

  return 0;
}
