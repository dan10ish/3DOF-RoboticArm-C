#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Adjusting the define values to avoid naming conflicts
#define NSTEPPERSEC 5000
#define DELT (1.0 / NSTEPPERSEC)
#define LINK_D1 0.2
#define LINK_A2 0.5
#define LINK_A3 0.4
#define START_TIME 2
#define DURATION_T 2
#define DURATION_S 0.5
#define TIME_T1 3
#define TIME_T2 5
#define START_X1 0.23
#define START_Y1 0.3
#define START_Z1 0.15
#define END_X2 0.73
#define END_Y2 0
#define END_Z2 0.2
#define ARRAY_SIZE 4 * NSTEPPERSEC + 1 // Added extra space for spline calculation

#define PI 3.14159265358979323846

// Declaration for the provided inverse kinematics and Jacobian functions
void ikypprob1(double x, double y, double z, double *th1, double *th2, double *th3);
void yppjack(double th1, double th2, double th3, double jac[3][3]);

// acdsnjj function declaration
void acdsnjj(double s1, double S, double t1, double T, double Sa, double Saa, double t, double *jerk, double *jounce, double *Ta, double *ac, double *Tac, double *acs);

void writeCSV(char *filename, double *data, int rows, int columns);

void writeCSV2(const char *filename, double data[][6], int length);

// Calculate spline acceleration and jerk
void calculate_spline_acceleration_and_jerk(double Bth[][6], int length);

// Matrix Calculations

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

    // Add a small epsilon value to the determinant check
    if (fabs(det) < 1e-12)
    {
        fprintf(stderr, "Matrix is singular or nearly singular.");
        return -1;
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

int main()
{
    double x1 = START_X1, y1 = START_Y1, z1 = START_Z1;
    double x2 = END_X2, y2 = END_Y2, z2 = END_Z2;
    double to = START_TIME;
    double delt = DELT;
    int nsteppersec = NSTEPPERSEC;

    double th10, th20, th30;
    ikypprob1(x1, y1, z1, &th10, &th20, &th30);

    double my = (y2 - y1) / (x2 - x1);
    double Cy = y1 - my * x1;
    double mz = (z2 - z1) / (x2 - x1);
    double Cz = z1 - mz * x1;

    double Bx[4 * nsteppersec + 1][3], By[4 * nsteppersec + 1][3], Bz[4 * nsteppersec + 1][3];
    double Bth1[4 * nsteppersec + 1][6], Bth2[4 * nsteppersec + 1][6], Bth3[4 * nsteppersec + 1][6];
    double jerk, jounce, Ta, ac, Tac, acs;
    double a = 0, v = 0, s = 0.23;
    double sx, sy, sz, vx, vy, vz;
    double sth1, sth2, sth3, vth1, vth2, vth3;
    double jac[3][3];

    for (int i = 0; i <= 4 * nsteppersec; i++)
    {
        double t = to + i * DELT;
        acdsnjj(0.23, 0.5, TIME_T1, DURATION_T, 0.025, 0.005, t, &jerk, &jounce, &Ta, &ac, &Tac, &acs);
        a += jerk * DELT;
        v += a * DELT;
        s += v * DELT;
        sx = s;
        sy = my * sx + Cy;
        sz = mz * sx + Cz;

        ikypprob1(sx, sy, sz, &sth1, &sth2, &sth3);
        yppjack(sth1, sth2, sth3, jac);

        // Assuming dth calculation is handled correctly within ikypprob1 or similar

        Bx[i][0] = t;
        Bx[i][1] = sx;
        Bx[i][2] = vx;

        By[i][0] = t;
        By[i][1] = sy;
        By[i][2] = vy;

        Bz[i][0] = t;
        Bz[i][1] = sz;
        Bz[i][2] = vz;

        Bth1[i][0] = t;
        Bth1[i][1] = sth1;
        Bth2[i][0] = t;
        Bth2[i][1] = sth2;
        Bth3[i][0] = t;
        Bth3[i][1] = sth3;
    }

    calculate_spline_acceleration_and_jerk(Bth1, ARRAY_SIZE);
    calculate_spline_acceleration_and_jerk(Bth2, ARRAY_SIZE);
    calculate_spline_acceleration_and_jerk(Bth3, ARRAY_SIZE);

    writeCSV2("Bth1.csv", Bth1, ARRAY_SIZE);
    writeCSV2("Bth2.csv", Bth2, ARRAY_SIZE);
    writeCSV2("Bth3.csv", Bth3, ARRAY_SIZE);

    writeCSV("Bx.csv", &Bx[0][0], 4 * NSTEPPERSEC + 1, 3);
    writeCSV("By.csv", &By[0][0], 4 * NSTEPPERSEC + 1, 3);
    writeCSV("Bz.csv", &Bz[0][0], 4 * NSTEPPERSEC + 1, 3);
    // writeCSV("Bth1.csv", &Bth1[0][0], 4 * NSTEPPERSEC + 1, 6);
    // writeCSV("Bth2.csv", &Bth2[0][0], 4 * NSTEPPERSEC + 1, 6);
    // writeCSV("Bth3.csv", &Bth3[0][0], 4 * NSTEPPERSEC + 1, 6);

    return 0;
}

void ikypprob1(double x, double y, double z, double *th1, double *th2, double *th3)
{
    double a1 = LINK_D1;
    double a2 = LINK_A2;
    double a3 = LINK_A3;

    double dop1 = sqrt((x * x) + (y * y) + ((z - a1) * (z - a1)));

    // Calculate theta1
    *th1 = atan2(y, x);

    // Calculate theta2
    *th2 = atan2((-z + a1), sqrt(x * x + y * y)) - acos(((a3 * a3) - (a2 * a2) - (dop1 * dop1)) / (-2 * a2 * dop1));

    // Calculate theta3
    *th3 = PI - acos((dop1 * dop1 - a2 * a2 - a3 * a3) / (-2 * a2 * a3));
}

void yppjack(double th1, double th2, double th3, double jac[3][3])
{
    // First row of the Jacobian matrix
    jac[0][0] = -sin(th1) * (LINK_A3 * cos(th2 + th3) + LINK_A2 * cos(th2));
    jac[0][1] = -cos(th1) * (LINK_A3 * sin(th2 + th3) + LINK_A2 * sin(th2));
    jac[0][2] = -LINK_A3 * sin(th2 + th3) * cos(th1);

    // Second row of the Jacobian matrix
    jac[1][0] = cos(th1) * (LINK_A3 * cos(th2 + th3) + LINK_A2 * cos(th2));
    jac[1][1] = -sin(th1) * (LINK_A3 * sin(th2 + th3) + LINK_A2 * sin(th2));
    jac[1][2] = -LINK_A3 * sin(th2 + th3) * sin(th1);

    // Third row of the Jacobian matrix
    jac[2][0] = 0;
    jac[2][1] = -LINK_A3 * cos(th2 + th3) - LINK_A2 * cos(th2);
    jac[2][2] = -LINK_A3 * cos(th2 + th3);
}

void acdsnjj(double s1, double S, double t1, double T, double Sa, double Saa, double t,
             double *jerk, double *jounce, double *Ta, double *ac, double *Tac, double *acs)
{
    double t2 = TIME_T2;
    double Sc = S - 2 * Sa;                                // Continuous segment length
    *Ta = (Sa * T) / (2 * Sa + Sc);                        // Total time for acceleration phase
    double Tc = T - 2 * (*Ta);                             // Total time for cruising phase
    double Taa = (Saa * (*Ta)) / (2 * Saa + Sa - 2 * Saa); // Time for start and end acceleration of S-curve
    *Tac = *Ta - 2 * Taa;                                  // Time for constant acceleration in the S-curve
    *ac = 2 * Sa / (pow(*Ta, 2));                          // Acceleration
    *acs = *ac * (*Ta) / ((*Ta + *Tac) / 2);               // Equivalent constant acceleration for S-curve
    double adot = *acs / Taa;                              // Rate of change of acceleration
    double Taj = Taa / 5;                                  // Time for jounce phases
    double jouncemax = 1.25 * adot / Taj;                  // Maximum jounce value

    *jerk = 0;   // Initialize jerk
    *jounce = 0; // Initialize jounce

    // Calculate jerk and jounce based on the current time `t`
    if (t < t1)
    {
        *jerk = 0;
        *jounce = 0;
    }
    else if (t >= t1 && t < (t1 + Taj))
    {
        *jounce = jouncemax;
        *jerk = jouncemax * (t - t1);
    }
    else if (t >= (t1 + Taj) && t < (t1 + Taa - Taj))
    {
        *jounce = 0;
        *jerk = jouncemax * Taj;
    }
    else if (t >= (t1 + Taa - Taj) && t < t1 + Taa)
    {
        *jounce = -jouncemax;
        *jerk = jouncemax * Taj - jouncemax * (t - (t1 + Taa - Taj));
    }
    else if (t >= t1 + Taa && t < (t1 + Taa + *Tac))
    {
        *jounce = 0;
        *jerk = 0;
    }
    else if (t >= (t1 + Taa + *Tac) && t < (t1 + Taa + *Tac + Taj))
    {
        *jounce = -jouncemax;
        *jerk = -jouncemax * (t - (t1 + Taa + *Tac));
    }
    else if (t >= (t1 + Taa + *Tac + Taj) && t < t2 - Taj)
    {
        *jounce = 0;
        *jerk = -jouncemax * Taj;
    }
    else if (t >= (t2 - Taj) && t < t2)
    {
        *jounce = jouncemax;
        *jerk = -jouncemax * Taj + jouncemax * (t - (t2 - Taj));
    }
    else
    {
        *jounce = 0;
        *jerk = 0;
    }
}

void calculate_spline_acceleration_and_jerk(double Bth[][6], int length)
{
    double S[3][3] = {{1, 0, 0}, {1, 0.5, 0.25}, {1, 1, 1}};
    double Si[3][3]; // Inverted S matrix

    if (invertMatrix(S, Si) != 0)
    {
        fprintf(stderr, "Matrix inversion failed.\n");
        return;
    }

    // Perform the spline calculations
    for (int k = 0; k < length; k++)
    {
        double V[3] = {Bth[k][2], Bth[k + 1][2], Bth[k + 2][2]}; // Velocity vector
        double rabc[3];                                          // Resultant vector for acceleration and jerk
        multiplyMatrixVector(Si, V, rabc);

        Bth[k][4] = rabc[1] / (2 * DELT);        // Acceleration
        Bth[k][5] = rabc[2] / (2 * DELT * DELT); // Jerk
    }
}

void writeCSV(char *filename, double *data, int rows, int columns)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL)
    {
        printf("Error opening file %s\n", filename);
        return;
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            // Calculate the index using row-major order and print the value
            fprintf(file, "%f", *(data + i * columns + j));
            if (j < columns - 1)
                fprintf(file, ",");
        }
        fprintf(file, "\n");
    }
    fclose(file);
}

// void writeCSV2(const char *filename, double data[][6], int length)
// {
//     FILE *fp = fopen(filename, "w");
//     if (fp == NULL)
//     {
//         perror("Unable to open file for writing");
//         return;
//     }

//     for (int i = 0; i < length; ++i)
//     {
//         for (int j = 0; j < 6; ++j)
//         {
//             fprintf(fp, "%f", data[i][j]);
//             if (j < 5)
//             {
//                 fprintf(fp, ",");
//             }
//         }
//         fprintf(fp, "\n");
//     }

//     fclose(fp);
// }

void writeCSV2(const char *filename, double data[][6], int length)
{
    FILE *fp = fopen(filename, "w");
    if (fp == NULL)
    {
        perror("Unable to open file for writing");
        return;
    }

    for (int i = 0; i < length; ++i)
    {
        for (int j = 0; j < 20; ++j)
        {
            fprintf(fp, "%f", data[i][j]);
            if (j < 19)
            {
                fprintf(fp, ",");
            }
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}