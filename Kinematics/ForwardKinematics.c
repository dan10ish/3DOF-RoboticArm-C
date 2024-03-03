#include <math.h>
#include <stdio.h>

// Define a structure for 3D vectors
typedef struct {
  double x, y, z;
} Vector3D;

// Define a structure for 3x3 matrices
typedef struct {
  double m[3][3];
} Matrix3x3;

// Define a structure for 4x4 matrices
typedef struct {
  double m[4][4];
} Matrix4x4;

// Function to create a rotation matrix about the Z-axis (yaw)
Matrix4x4 createRotationZ(double angle) {
  Matrix4x4 R;
  double rad = angle * M_PI / 180.0; // Convert to radians
  R.m[0][0] = cos(rad);
  R.m[0][1] = -sin(rad);
  R.m[0][2] = 0;
  R.m[0][3] = 0;
  R.m[1][0] = sin(rad);
  R.m[1][1] = cos(rad);
  R.m[1][2] = 0;
  R.m[1][3] = 0;
  R.m[2][0] = 0;
  R.m[2][1] = 0;
  R.m[2][2] = 1;
  R.m[2][3] = 0;
  R.m[3][0] = 0;
  R.m[3][1] = 0;
  R.m[3][2] = 0;
  R.m[3][3] = 1;
  return R;
}

// Function to create a rotation matrix about the Y-axis (pitch)
Matrix4x4 createRotationY(double angle) {
  Matrix4x4 R;
  double rad = angle * M_PI / 180.0; // Convert to radians
  R.m[0][0] = cos(rad);
  R.m[0][1] = 0;
  R.m[0][2] = sin(rad);
  R.m[0][3] = 0;
  R.m[1][0] = 0;
  R.m[1][1] = 1;
  R.m[1][2] = 0;
  R.m[1][3] = 0;
  R.m[2][0] = -sin(rad);
  R.m[2][1] = 0;
  R.m[2][2] = cos(rad);
  R.m[2][3] = 0;
  R.m[3][0] = 0;
  R.m[3][1] = 0;
  R.m[3][2] = 0;
  R.m[3][3] = 1;
  return R;
}

// Function for 4x4 matrix multiplication
Matrix4x4 multiplyMatrix4x4(Matrix4x4 A, Matrix4x4 B) {
  Matrix4x4 C;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      C.m[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        C.m[i][j] += A.m[i][k] * B.m[k][j];
      }
    }
  }
  return C;
}

// Function to create a translation matrix
Matrix4x4 createTranslation(double dx, double dy, double dz) {
  Matrix4x4 T;
  T.m[0][0] = 1;
  T.m[0][1] = 0;
  T.m[0][2] = 0;
  T.m[0][3] = dx;
  T.m[1][0] = 0;
  T.m[1][1] = 1;
  T.m[1][2] = 0;
  T.m[1][3] = dy;
  T.m[2][0] = 0;
  T.m[2][1] = 0;
  T.m[2][2] = 1;
  T.m[2][3] = dz;
  T.m[3][0] = 0;
  T.m[3][1] = 0;
  T.m[3][2] = 0;
  T.m[3][3] = 1;
  return T;
}

// Forward kinematics function
Vector3D calculateForwardKinematics(double yaw, double pitch1, double pitch2,
                                    double L1, double L2, double L3) {
  Matrix4x4 T_base_to_1 = createRotationZ(yaw);
  Matrix4x4 T_1_to_2 =
      multiplyMatrix4x4(T_base_to_1, createTranslation(0, 0, L1));
  T_1_to_2 = multiplyMatrix4x4(T_1_to_2, createRotationY(pitch1));
  Matrix4x4 T_2_to_3 = multiplyMatrix4x4(T_1_to_2, createTranslation(0, 0, L2));
  T_2_to_3 = multiplyMatrix4x4(T_2_to_3, createRotationY(pitch2));
  Matrix4x4 T_3_to_end =
      multiplyMatrix4x4(T_2_to_3, createTranslation(0, 0, L3));

  Vector3D endEffectorPosition;
  endEffectorPosition.x = T_3_to_end.m[0][3];
  endEffectorPosition.y = T_3_to_end.m[1][3];
  endEffectorPosition.z = T_3_to_end.m[2][3];

  return endEffectorPosition;
}

int main() {
  double yaw = 30;                  // Base rotation in degrees
  double pitch1 = 45;               // First joint pitch in degrees
  double pitch2 = 30;               // Second joint pitch in degrees
  double L1 = 10, L2 = 10, L3 = 10; // Link lengths

  Vector3D endEffector =
      calculateForwardKinematics(yaw, pitch1, pitch2, L1, L2, L3);
  printf("End Effector Position: (x: %f, y: %f, z: %f)\n", endEffector.x,
         endEffector.y, endEffector.z);

  return 0;
}
