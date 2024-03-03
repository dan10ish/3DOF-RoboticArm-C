#include "ControlDynamics.c" // Ensure this includes the RobotState definition
#include <math.h>
#include <stdio.h>

// PD Controller Gains
double Kp[3] = {1200, 1271, 1344}; // Proportional gains
double Kd[3] = {70, 72, 74};       // Derivative gains

// Desired state (positions and velocities)
double th0dds[3] = {1.0, 1.0, 1.0}; // Example desired positions
double th1dds[3] = {0.0, 0.0, 0.0}; // Example desired velocities

void calculateControlTorques(const RobotState *state, double *tau) {
  double M[3][3];         // Mass matrix
  double Minv[3][3];      // Inverse of the mass matrix
  double err[3], errd[3]; // Error and error derivative

  // Compute the mass matrix for the current state
  calculateMassMatrix(state, M);
  if (invertMatrix(M, Minv) != 0) {
    fprintf(stderr, "Failed to invert mass matrix.\n");
    return;
  }

  // Variables for Coriolis and gravitational forces
  double C[3] = {0}; // Initialize Coriolis forces
  double g[3];       // Gravity vector

  double q1 = state->q[0], q2 = state->q[1], q3 = state->q[2];
  double qdot1 = state->qdot[0], qdot2 = state->qdot[1], qdot3 = state->qdot[2];

  // Coriolis forces calculations
  double C112 = -((1.0 / 3.0) * m2 * pow(a2, 2) * sin(2 * q2) +
                  m3 * pow(a2, 2) * sin(q2) +
                  (1.0 / 3.0) * m3 * pow(a3, 2) * sin(2 * (q2 + q3)) +
                  m3 * a2 * a3 * sin(2 * q2 + q3));
  double C113 = -((1.0 / 2.0) * m3 * a2 * a3 * sin(q3) +
                  (1.0 / 3.0) * m3 * pow(a3, 2) * sin(2 * (q2 + q3)) +
                  (1.0 / 2.0) * m3 * a2 * a3 * sin(2 * q2 + q3));
  double C211 =
      ((1.0 / 6.0) * m2 * pow(a2, 2) + (1.0 / 2.0) * m3 * pow(a3, 2)) *
          sin(2 * q2) +
      (1.0 / 6.0) * m3 * pow(a3, 2) * sin(2 * (q2 + q3)) +
      (1.0 / 2.0) * m3 * a2 * a3 * sin(2 * q2 + q3);
  double C223 = -(m3 * a2 * a3 * sin(q3));
  double C233 = -(1.0 / 2.0) * m2 * a2 * a3 * sin(q3);
  double C311 = (1.0 / 6.0) * m3 * pow(a3, 2) * sin(2 * (q2 + q3)) +
                (1.0 / 4.0) * m3 * a2 * a3 * sin(q3) +
                (1.0 / 4.0) * m3 * a2 * a3 * sin(2 * q2 + q3);
  double C322 = (1.0 / 2.0) * m3 * a2 * a3 * sin(q3);

  // Gravity vector calculations
  g[0] = 0;
  g[1] = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3) + a2 * cos(q2)) -
         (1.0 / 2.0) * g0 * m2 * a2 * cos(q2);
  g[2] = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3));

  double C1 = qdot1 * (C112 * qdot2 + C113 * qdot3);
  double C2 = qdot2 * (C211 * qdot1 + C223 * qdot3);
  double C3 = qdot3 * (C311 * qdot1 + C322 * qdot2);

  // Compute total Coriolis force for each joint
  C[0] = C1;
  C[1] = C2;
  C[2] = C3;

  // PD Control with gravity compensation and dynamic consideration
  for (int i = 0; i < 3; i++) {
    err[i] = th0dds[i] - state->q[i];     // Position error
    errd[i] = th1dds[i] - state->qdot[i]; // Velocity error
    // Desired acceleration based on PD control
    double qdd = Kp[i] * err[i] + Kd[i] * errd[i];
    // Compute control torques, including dynamics
    tau[i] = M[i][0] * qdd + M[i][1] * qdd + M[i][2] * qdd + C[i] + g[i];
  }
}

// Main function remains largely unchanged
int main() {
  // Initialize current state
  RobotState currentState = {{0, -0.387, 0.45}, {0, 0, 0}};
  double tau[3];

  // Calculate control torques based on the current state and desired state
  calculateControlTorques(&currentState, tau);

  // Print calculated torques for demonstration
  printf("Calculated control torques: %f, %f, %f\n", tau[0], tau[1], tau[2]);

  return 0;
}
