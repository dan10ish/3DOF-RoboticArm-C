#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Robot parameters (global variables)
double g0 = 9.81, m1 = 0.25, m2 = 0.25, m3 = 0.2, d1 = 0.2, a2 = 0.5, a3 = 0.4;

// Sign function
double sign(double num)
{
  if (num > 0)
  {
    return 1.0;
  }
  else if (num < 0)
  {
    return -1.0;
  }
  else
  {
    return 0.0;
  }
}

// Robot State Structure
typedef struct
{
  double q[3];    // Displacements: q1, q2, q3
  double qdot[3]; // Velocities: qdot1, qdot2, qdot3
} RobotState;

// Mass Matrix Calculation
void calculateMassMatrix(const RobotState *state, double M[3][3])
{
  double q2 = state->q[1], q3 = state->q[2];

  M[0][0] = (1.0 / 6.0) * m2 * (a2 * a2) + (1.0 / 3.0) * m3 * (a2 * a2) +
            (1.0 / 6.0) * m3 * (a3 * a3) +
            (1.0 / 6.0) * m3 * (a3 * a3) * cos((2 * q2) + (2 * q3)) +
            ((1.0 / 6.0) * m2 + (1.0 / 3.0) * m3) * (a2 * a2) * cos(2 * q2) +
            (1 / 2) * m3 * a2 * a3 * (cos(2 * q2 + 2 * q3)) + cos(q3);
  M[0][1] = 0.0;
  M[0][2] = 0.0;

  M[1][0] = 0.0;
  M[1][1] = (1.0 / 3.0) * m2 * (a2 * a2) + m3 * (a2 * a2) +
            (1.0 / 3.0) * m3 * (a3 * a3) + m3 * a2 * a3 * cos(q3);
  M[1][2] = (1.0 / 3.0) * m3 * (a3 * a3) + (1.0 / 2.0) * m3 * a2 * a3 * cos(q3);

  M[2][0] = 0.0;
  M[2][1] = (1.0 / 3.0) * m3 * (a3 * a3) + (1.0 / 2.0) * m3 * a2 * a3 * cos(q3);
  M[2][2] = (1.0 / 3.0) * m3 * (a3 * a3);
}

double determinant(double M[3][3])
{
  double det = M[0][0] * (M[1][1] * M[2][2] - M[2][1] * M[1][2]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  return det;
}

int invertMatrix(double M[3][3], double Minv[3][3])
{
  double det = determinant(M);

  if (fabs(det) < 1e-6)
  {            // Check for non-invertible matrix
    return -1; // Matrix is singular or nearly singular
  }

  double invDet = 1.0 / det;

  Minv[0][0] = invDet * (M[1][1] * M[2][2] - M[2][1] * M[1][2]);
  Minv[0][1] = invDet * (M[0][2] * M[2][1] - M[0][1] * M[2][2]);
  Minv[0][2] = invDet * (M[0][1] * M[1][2] - M[0][2] * M[1][1]);

  Minv[1][0] = invDet * (M[1][2] * M[2][0] - M[1][0] * M[2][2]);
  Minv[1][1] = invDet * (M[0][0] * M[2][2] - M[0][2] * M[2][0]);
  Minv[1][2] = invDet * (M[1][0] * M[0][2] - M[0][0] * M[1][2]);

  Minv[2][0] = invDet * (M[1][0] * M[2][1] - M[2][0] * M[1][1]);
  Minv[2][1] = invDet * (M[2][0] * M[0][1] - M[0][0] * M[2][1]);
  Minv[2][2] = invDet * (M[0][0] * M[1][1] - M[1][0] * M[0][1]);

  return 0; // Success
}

void multiplyMatrixVector(double M[3][3], double V[3], double R[3])
{
  for (int i = 0; i < 3; i++)
  {
    R[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      R[i] += M[i][j] * V[j];
    }
  }
}

// Dynamics Function
void DynIImkcndzero(const RobotState *state, double *xdot, const double *tau)
{
  // Extract state variables for readability
  double q1 = state->q[0], q2 = state->q[1], q3 = state->q[2];
  double qdot1 = state->qdot[0], qdot2 = state->qdot[1], qdot3 = state->qdot[2];

  // Initializing damping coefficients and apsilon values
  double b11 = 0.01, b12 = 0.02, b13 = 0.01;
  double b21 = 0.01, b22 = 0.02, b23 = 0.01;
  double b31 = 0.01, b32 = 0.02, b33 = 0.01;
  double apsilon1 = 0.05, apsilon2 = 0.05, apsilon3 = 0.05;

  // Compute Coriolis forces and gravitational forces
  double C112 =
      -((1.0 / 3.0) * (m2 + m3) * pow(a2, 2) * sin(2 * q2) +
        (1.0 / 3.0) * m3 *
            (pow(a3, 2) * sin(2 * q2 + 2 * q3) + a2 * a3 * sin(2 * q2 + q3)));
  double C113 =
      -((1.0 / 3.0) * m3 * pow(a3, 2) * sin(2 * q2 + 2 * q3) +
        (1.0 / 2.0) * m3 * a2 * a3 * (sin(2 * q2 + q3) + sin(2 * q2)));
  double C211 =
      ((1.0 / 6.0) * m2 + (1.0 / 2.0) * m3) * pow(a2, 2) * sin(2 * q2) +
      (1.0 / 6.0) * m3 * a3 * sin(2 * q2 + 2 * q3) +
      (1.0 / 2.0) * m3 * a2 * a3 * sin(2 * q2 + q3);
  double C233 = -(1.0 / 2.0) * m3 * a2 * a3 * sin(q3);
  double C232 = -m3 * a2 * a3 * sin(q3);
  double C311 = ((1.0 / 4.0) * m3 * a2 * a3 * sin(q3) +
                 (1.0 / 6.0) * m3 * a3 * sin(2 * q2 + 2 * q3) +
                 (1.0 / 4.0) * m3 * a2 * a3 * sin(2 * q2 + q3));
  double C322 = (1.0 / 2.0) * m3 * a2 * a3 * sin(q3);
  double h2 = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3) + a2 * cos(q2)) -
              (1.0 / 2.0) * g0 * m2 * a2 * cos(q2);
  double h3 = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3));

  // Mass matrix and its inverse
  double M[3][3], Minv[3][3];
  calculateMassMatrix(state, M);
  if (invertMatrix(M, Minv) != 0)
  {
    fprintf(stderr, "Matrix inversion failed.\n");
    return;
  }

  // Net forces acting on the system (considering damping, Coriolis, and
  // gravitational forces)
  double net_forces[3];
  net_forces[0] =
      tau[0] -
      (b11 * qdot1 +
       sign(qdot1) * (b12 - (b12 - b13) * exp(-fabs(qdot1) / apsilon1))) +
      0; // h1
  net_forces[1] =
      tau[1] -
      (b21 * qdot2 +
       sign(qdot2) * (b22 - (b22 - b23) * exp(-fabs(qdot2) / apsilon2))) +
      -(C211 * pow(qdot1, 2) + C233 * pow(qdot3, 2) + C232 * qdot3 * qdot2) -
      h2;
  net_forces[2] =
      tau[2] -
      (b31 * qdot3 +
       sign(qdot3) * (b32 - (b32 - b33) * exp(-fabs(qdot3) / apsilon3))) +
      -(C311 * pow(qdot1, 2) + C322 * pow(qdot2, 2)) - h3;

  // Solving for accelerations
  double accelerations[3];
  multiplyMatrixVector(Minv, net_forces, accelerations);

  // Update xdot with velocities and accelerations
  xdot[0] = qdot1;
  xdot[1] = qdot2;
  xdot[2] = qdot3;
  xdot[3] = accelerations[0];
  xdot[4] = accelerations[1];
  xdot[5] = accelerations[2];
}

void rk4(void (*model)(const RobotState *, double *, const double *),
         RobotState *state, double dt, const double *tau)
{
  double k1[6], k2[6], k3[6], k4[6];
  RobotState tempState = *state;

  // Calculate k1
  model(state, k1, tau);
  // Update tempState for k2 calculation
  for (int i = 0; i < 6; i++)
  {
    if (i < 3) // For q[i], update using velocities
      tempState.q[i] += 0.5 * dt * k1[i];
    else // For qdot[i-3], update using accelerations
      tempState.qdot[i - 3] += 0.5 * dt * k1[i];
  }
  model(&tempState, k2, tau);

  // Update tempState for k3 calculation
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
      tempState.q[i] = state->q[i] + 0.5 * dt * k2[i];
    else
      tempState.qdot[i - 3] = state->qdot[i - 3] + 0.5 * dt * k2[i];
  }
  model(&tempState, k3, tau);

  // Update tempState for k4 calculation
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
      tempState.q[i] = state->q[i] + dt * k3[i];
    else
      tempState.qdot[i - 3] = state->qdot[i - 3] + dt * k3[i];
  }
  model(&tempState, k4, tau);

  // Update state with the final weighted average
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
      state->q[i] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6;
    else
      state->qdot[i - 3] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6;
  }
}

int main()
{
  // Initial coordinates (0, -0.385, 0.4)
  // Initial joint Velocities (0,0,0)
  RobotState state = {{0, -0.387, 0.45}, {0, 0, 0}};

  // Time Step
  double dt = 0.1;
  double t = 0;
  double t_end = 50;

  double tau[3] = {0, 0, 0};

  // Open a file for writing simulation data
  FILE *file = fopen("CSV Files/Dynamics.csv", "w");
  if (file == NULL)
  {
    printf("Error opening file\n");
    return 1;
  }

  // Write header for the CSV file
  fprintf(file, "Time,q1,q2,q3,qdot1,qdot2,qdot3\n");

  // Main simulation loop
  while (t < t_end)
  {
    rk4(DynIImkcndzero, &state, dt, tau);
    t += dt;

    // Print state variables
    printf("Time: %.2f, Positions: q1 = %.3f, q2 = %.3f, q3 = %.3f, "
           "Velocities: qdot1 = %.3f, qdot2 = %.3f, qdot3 = %.3f\n",
           t, state.q[0], state.q[1], state.q[2], state.qdot[0], state.qdot[1],
           state.qdot[2]);

    // Write data to the CSV file
    fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", t, state.q[0], state.q[1],
            state.q[2], state.qdot[0], state.qdot[1], state.qdot[2]);
  }

  // Close the file
  fclose(file);

  return 0;
}
