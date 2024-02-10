#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

// Define a structure for the robot state
typedef struct
{
    double q[3];    // Displacements: q1, q2, q3
    double qdot[3]; // Velocities: qdot1, qdot2, qdot3
} RobotState;

void DynIImkcndzero(const RobotState *state, double *xdot)
{

    // Extract state variables for readability
    double q1 = state->q[0], q2 = state->q[1], q3 = state->q[2];
    double qdot1 = state->qdot[0], qdot2 = state->qdot[1], qdot3 = state->qdot[2];

    // Initializing variables
    double b11 = 0.01;
    double b12 = 0.02;
    double b13 = 0.01;
    double b21 = 0.01;
    double b22 = 0.02;
    double b23 = 0.01;
    double b31 = 0.01;
    double b32 = 0.02;
    double b33 = 0.01;

    double apsilon1 = 0.05;
    double apsilon2 = 0.05;
    double apsilon3 = 0.05;

    // Compute Coriolis forces
    double C112 = -((1.0 / 3.0) * (m2 + m3) * pow(a2, 2) * sin(2 * q2) +
                    (1.0 / 3.0) * m3 * (pow(a3, 2) * sin(2 * q2 + 2 * q3) + a2 * a3 * sin(2 * q2 + q3)));
    double C113 = -((1.0 / 3.0) * m3 * pow(a3, 2) * sin(2 * q2 + 2 * q3) +
                    (1.0 / 2.0) * m3 * a2 * a3 * (sin(2 * q2 + q3) + sin(2 * q2)));
    double C211 = ((1.0 / 6.0) * m2 + (1.0 / 2.0) * m3) * pow(a2, 2) * sin(2 * q2) +
                  (1.0 / 6.0) * m3 * a3 * sin(2 * q2 + 2 * q3) +
                  (1.0 / 2.0) * m3 * a2 * a3 * sin(2 * q2 + q3);
    double C233 = -(1.0 / 2.0) * m3 * a2 * a3 * sin(q3);
    double C232 = -m3 * a2 * a3 * sin(q3);
    double C311 = ((1.0 / 4.0) * m3 * a2 * a3 * sin(q3) +
                   (1.0 / 6.0) * m3 * a3 * sin(2 * q2 + 2 * q3) +
                   (1.0 / 4.0) * m3 * a2 * a3 * sin(2 * q2 + q3));
    double C322 = (1.0 / 2.0) * m3 * a2 * a3 * sin(q3);
    double h1 = 0;
    double h2 = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3) + a2 * cos(q2)) -
                (1.0 / 2.0) * g0 * m2 * a2 * cos(q2);
    double h3 = -g0 * m3 * ((1.0 / 2.0) * a3 * cos(q2 + q3));

    // Update xdot based on dynamics
    xdot[0] = qdot1;
    xdot[1] = qdot2;
    xdot[2] = qdot3;
    xdot[3] = -(b11 * qdot1 + sign(qdot1) * (b12 - (b12 - b13) * exp(-fabs(qdot1) / apsilon1))) -
              (C112 * qdot1 * qdot2 + C113 * qdot1 * qdot3) - h1;
    xdot[4] = -(b21 * qdot2 + sign(qdot2) * (b22 - (b22 - b23) * exp(-fabs(qdot2) / apsilon2))) -
              (C211 * pow(qdot1, 2) + C233 * pow(qdot3, 2) + C232 * qdot3 * qdot2) - h2;
    xdot[5] = -(b31 * qdot3 + sign(qdot3) * (b32 - (b32 - b33) * exp(-fabs(qdot3) / apsilon3))) -
              (C311 * pow(qdot1, 2) + C322 * pow(qdot2, 2)) - h3;

    // // Update derivatives
    // xdot[0] = qdot1;
    // xdot[1] = qdot2;
    // xdot[2] = qdot3;
    // xdot[3] = -(b11 * qdot1 + sign(qdot1) * (b12 - (b12 - b13) * exp(-fabs(qdot1) / apsilon1))) -
    //           (C112 * qdot1 * qdot2 + C113 * qdot1 * qdot3) - h1;
    // xdot[4] = -(b21 * qdot2 + sign(qdot2) * (b22 - (b22 - b23) * exp(-fabs(qdot2) / apsilon2))) -
    //           (C211 * pow(qdot1, 2) + C223 * qdot3 * qdot2) - h2;
    // xdot[5] = -(b31 * qdot3 + sign(qdot3) * (b32 - (b32 - b33) * exp(-fabs(qdot3) / apsilon3))) -
    //           (C322 * pow(qdot2, 2) + C311 * pow(qdot1, 2)) - h3;
}

void rk4(void (*model)(const RobotState *, double *), RobotState *state, double dt)
{
    double k1[6], k2[6], k3[6], k4[6];
    RobotState tempState;

    // Compute k1 for both q and qdot
    model(state, k1);
    for (int i = 0; i < 3; i++)
    {
        tempState.q[i] = state->q[i] + 0.5 * dt * k1[i];
        tempState.qdot[i] = state->qdot[i] + 0.5 * dt * k1[i + 3];
    }

    // Compute k2 for both q and qdot
    model(&tempState, k2);
    for (int i = 0; i < 3; i++)
    {
        tempState.q[i] = state->q[i] + 0.5 * dt * k2[i];
        tempState.qdot[i] = state->qdot[i] + 0.5 * dt * k2[i + 3];
    }

    // Compute k3 for both q and qdot
    model(&tempState, k3);
    for (int i = 0; i < 3; i++)
    {
        tempState.q[i] = state->q[i] + dt * k3[i];
        tempState.qdot[i] = state->qdot[i] + dt * k3[i + 3];
    }

    // Compute k4 for both q and qdot
    model(&tempState, k4);

    // Update state with the final weighted average for both q and qdot
    for (int i = 0; i < 3; i++)
    {
        state->q[i] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        state->qdot[i] += dt * (k1[i + 3] + 2 * k2[i + 3] + 2 * k3[i + 3] + k4[i + 3]) / 6.0;
    }
}

int main()
{
    // Initial coordinates (0, -0.385, 0.4)
    // Initial joint Velocities (0,0,0)
    RobotState state = {{0, -0.387, 0.45}, {0, 0, 0}};

    // Time Step
    double dt = 0.01;
    double t = 0;
    double t_end = 200;

    // Plotting the Values
    // Open a file for writing
    FILE *file = fopen("simulation_data.csv", "w");
    if (file == NULL)
    {
        printf("Error opening file\n");
        return 1;
    }

    // Write header
    fprintf(file, "Time,q1,q2,q3,qdot1,qdot2,qdot3\n");

    while (t < t_end)
    {
        rk4(DynIImkcndzero, &state, dt);
        t += dt;
        // Optionally print state variables to observe the simulation
        printf("Time: %.2f, Positions: q1 = %.3f, q2 = %.3f, q3 = %.3f, Velocities: qdot1 = %.3f, qdot2 = %.3f, qdot3 = %.3f\n",
               t, state.q[0], state.q[1], state.q[2], state.qdot[0], state.qdot[1], state.qdot[2]);

        fprintf(file, "%f,%f,%f,%f,%f,%f,%f\n", t, state.q[0], state.q[1], state.q[2], state.qdot[0], state.qdot[1], state.qdot[2]);
    }

    // Close the file
    fclose(file);

    return 0;
}
