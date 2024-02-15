#include "ControlDynamics.c"
#include <stdio.h>
#include <math.h> // Include Dynamics.c or necessary declarations

// PD Controller Gains
double Kp[3][3] = {{1200, 0, 0}, {0, 1271, 0}, {0, 0, 1344}};
double Kd[3][3] = {{70, 0, 0}, {0, 72, 0}, {0, 0, 74}};

// Desired States
double th0dds[3] = {1.0, 1.0, 1.0}; // Desired positions
double th1dds[3] = {0.0, 0.0, 0.0}; // Desired velocities

void PDController(const RobotState *state, double *tau)
{
    double error_pos[3], error_vel[3], control_effort[3];

    // Calculate position and velocity errors
    for (int i = 0; i < 3; i++)
    {
        error_pos[i] = th0dds[i] - state->q[i];
        error_vel[i] = th1dds[i] - state->qdot[i];
    }

    // Apply PD control law: tau = -Kp * error_pos - Kd * error_vel
    for (int i = 0; i < 3; i++)
    {
        tau[i] = 0; // Initialize control input
        for (int j = 0; j < 3; j++)
        {
            tau[i] -= Kp[i][j] * error_pos[j] + Kd[i][j] * error_vel[j];
        }
    }
}

int main()
{
    RobotState state = {{0, -0.387, 0.45}, {0, 0, 0}};
    // double tau[3] = {0, 0, 0}; // Control input

    double A[6] = {state.q[0], state.q[1], state.q[2], state.qdot[0], state.qdot[1], state.qdot[2]};
    double tau[3] = {0, 0, 0}; // Example values for tau

    // Assuming Bc matrix as a 6x3 matrix with specific structure
    double Bc[6][3] = {
        {0.0, 0.0, 0.0}, // Top half is zero
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}, // Bottom half is identity matrix
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}};

    // Simulation parameters
    double t = 0.0, dt = 0.01, t_end = 50.0;

    // Simulation loop
    while (t <= t_end)
    {
        // Update control input based on PD controller
        PDController(&state, tau);

        // Update state based on dynamics and control input
        rk4(DynIImkcndzero, &state, dt, tau);

        // Increment time
        t += dt;

        // Logging or output code can go here
    }

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            A[i] += Bc[i][j] * tau[j]; // Directly adding Bc * tau to A
        }
        if (isnan(A[i]))
        {
            printf("NaN detected in A[%d]\n", i);
        }
        else
        {
            printf("%.2f\n", A[i]);
        }
    }

    return 0;
}
