#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NSTEPPERSEC 5000
#define DELT (1.0 / NSTEPPERSEC)
#define LINK_D1 0.2
#define LINK_A2 0.5
#define LINK_A3 0.4
#define START_TIME 2
#define START_X1 0.23
#define START_Y1 0.3
#define START_Z1 0.15
#define END_X2 0.73
#define END_Y2 0
#define END_Z2 0.2
#define TIME_T1 3
#define TIME_T2 5
#define DURATION_T 2
#define DURATION_S 0.5
#define PI 3.14159265358979323846
#define ARRAY_SIZE (4 * NSTEPPERSEC + 1)

// Function prototypes
void ikypprob1(double x, double y, double z, double *th1, double *th2,
               double *th3);
void yppjack(double th1, double th2, double th3, double jac[3][3]);
void acdsnjj(double s1, double S, double t1, double T, double Sa, double Saa,
             double t, double *jerk, double *jounce, double *Ta, double *ac,
             double *Tac, double *acs);
void calculate_spline_acceleration_and_jerk(double Bth[][6], int length);

void writeCSV(const char *filename, double data[][3], int length);

// Matrix Calculations

double determinant(double M[3][3]) {
  double det = M[0][0] * (M[1][1] * M[2][2] - M[2][1] * M[1][2]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  return det;
}

int invertMatrix(double M[3][3], double Minv[3][3]) {
  double det = determinant(M);

  // Add a small epsilon value to the determinant check
  if (fabs(det) < 1e-12) {
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

void multiplyMatrixVector(double M[3][3], double V[3], double R[3]) {
  for (int i = 0; i < 3; i++) {
    R[i] = 0;
    for (int j = 0; j < 3; j++) {
      R[i] += M[i][j] * V[j];
    }
  }
}

int main() {
  double x1 = START_X1, y1 = START_Y1, z1 = START_Z1;
  double x2 = END_X2, y2 = END_Y2, z2 = END_Z2;
  double my = (y2 - y1) / (x2 - x1);
  double Cy = y1 - my * x1;
  double mz = (z2 - z1) / (x2 - x1);
  double Cz = z1 - mz * x1;

  double th1, th2, th3;
  double sx, sy, sz, vx = 0, vy = 0, vz = 0;
  double jerk, jounce, Ta, ac, Tac, acs;
  double a = 0, v = 0, s = START_X1;

  double trajectory[ARRAY_SIZE][3]; // To store the end effector positions

  for (int i = 0; i < ARRAY_SIZE; i++) {
    double t = START_TIME + i * DELT;
    acdsnjj(0.23, 0.5, TIME_T1, DURATION_T, 0.025, 0.005, t, &jerk, &jounce,
            &Ta, &ac, &Tac, &acs);
    a += jerk * DELT;
    v += a * DELT;
    s += v * DELT;

    sx = s;
    sy = my * sx + Cy;
    sz = mz * sx + Cz;

    ikypprob1(sx, sy, sz, &th1, &th2, &th3);

    trajectory[i][0] = sx;
    trajectory[i][1] = sy;
    trajectory[i][2] = sz;
  }

  double Bth1[ARRAY_SIZE][6], Bth2[ARRAY_SIZE][6], Bth3[ARRAY_SIZE][6];

  calculate_spline_acceleration_and_jerk(Bth1, ARRAY_SIZE);
  calculate_spline_acceleration_and_jerk(Bth2, ARRAY_SIZE);
  calculate_spline_acceleration_and_jerk(Bth3, ARRAY_SIZE);

  writeCSV("CSV Files/Trajectory.csv", trajectory, ARRAY_SIZE);

  return 0;
}

void writeCSV(const char *filename, double data[][3], int length) {
  FILE *fp = fopen(filename, "w");
  if (fp == NULL) {
    perror("Unable to open file for writing");
    return;
  }

  for (int i = 0; i < length; ++i) {
    fprintf(fp, "%f,%f,%f\n", data[i][0], data[i][1], data[i][2]);
  }

  fclose(fp);
}

void ikypprob1(double x, double y, double z, double *th1, double *th2,
               double *th3) {
  double a1 = LINK_D1;
  double a2 = LINK_A2;
  double a3 = LINK_A3;

  double dop1 = sqrt((x * x) + (y * y) + ((z - a1) * (z - a1)));

  // Calculate theta1
  *th1 = atan2(y, x);

  // Calculate theta2
  *th2 = atan2((-z + a1), sqrt(x * x + y * y)) -
         acos(((a3 * a3) - (a2 * a2) - (dop1 * dop1)) / (-2 * a2 * dop1));

  // Calculate theta3
  *th3 = PI - acos((dop1 * dop1 - a2 * a2 - a3 * a3) / (-2 * a2 * a3));
}

void yppjack(double th1, double th2, double th3, double jac[3][3]) {
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

void acdsnjj(double s1, double S, double t1, double T, double Sa, double Saa,
             double t, double *jerk, double *jounce, double *Ta, double *ac,
             double *Tac, double *acs) {
  double t2 = TIME_T2;
  double Sc = S - 2 * Sa;         // Continuous segment length
  *Ta = (Sa * T) / (2 * Sa + Sc); // Total time for acceleration phase
  double Tc = T - 2 * (*Ta);      // Total time for cruising phase
  double Taa = (Saa * (*Ta)) /
               (2 * Saa + Sa -
                2 * Saa); // Time for start and end acceleration of S-curve
  *Tac = *Ta - 2 * Taa;   // Time for constant acceleration in the S-curve
  *ac = 2 * Sa / (pow(*Ta, 2)); // Acceleration
  *acs = *ac * (*Ta) /
         ((*Ta + *Tac) / 2); // Equivalent constant acceleration for S-curve
  double adot = *acs / Taa;  // Rate of change of acceleration
  double Taj = Taa / 5;      // Time for jounce phases
  double jouncemax = 1.25 * adot / Taj; // Maximum jounce value

  *jerk = 0;   // Initialize jerk
  *jounce = 0; // Initialize jounce

  // Calculate jerk and jounce based on the current time `t`
  if (t < t1) {
    *jerk = 0;
    *jounce = 0;
  } else if (t >= t1 && t < (t1 + Taj)) {
    *jounce = jouncemax;
    *jerk = jouncemax * (t - t1);
  } else if (t >= (t1 + Taj) && t < (t1 + Taa - Taj)) {
    *jounce = 0;
    *jerk = jouncemax * Taj;
  } else if (t >= (t1 + Taa - Taj) && t < t1 + Taa) {
    *jounce = -jouncemax;
    *jerk = jouncemax * Taj - jouncemax * (t - (t1 + Taa - Taj));
  } else if (t >= t1 + Taa && t < (t1 + Taa + *Tac)) {
    *jounce = 0;
    *jerk = 0;
  } else if (t >= (t1 + Taa + *Tac) && t < (t1 + Taa + *Tac + Taj)) {
    *jounce = -jouncemax;
    *jerk = -jouncemax * (t - (t1 + Taa + *Tac));
  } else if (t >= (t1 + Taa + *Tac + Taj) && t < t2 - Taj) {
    *jounce = 0;
    *jerk = -jouncemax * Taj;
  } else if (t >= (t2 - Taj) && t < t2) {
    *jounce = jouncemax;
    *jerk = -jouncemax * Taj + jouncemax * (t - (t2 - Taj));
  } else {
    *jounce = 0;
    *jerk = 0;
  }
}

void calculate_spline_acceleration_and_jerk(double Bth[][6], int length) {
  double S[3][3] = {{1, 0, 0}, {1, 0.5, 0.25}, {1, 1, 1}};
  double Si[3][3]; // Inverted S matrix

  if (invertMatrix(S, Si) != 0) {
    fprintf(stderr, "Matrix inversion failed.\n");
    return;
  }

  // Adjusted to avoid accessing out-of-bounds elements
  for (int k = 0; k < length - 2; k++) { // Ensure k+2 is within bounds
    double V[3] = {Bth[k][2], Bth[k + 1][2], Bth[k + 2][2]}; // Velocity vector
    double rabc[3]; // Resultant vector for acceleration and jerk
    multiplyMatrixVector(Si, V, rabc);

    // Store acceleration and jerk in the array
    Bth[k][4] = rabc[1] / (2 * DELT);        // Acceleration
    Bth[k][5] = rabc[2] / (2 * DELT * DELT); // Jerk
  }
}
