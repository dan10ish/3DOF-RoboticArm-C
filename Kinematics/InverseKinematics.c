#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

typedef struct
{
  double x, y, z;
} Position;

typedef struct
{
  double a1, a2, a3;             // Link lengths
  double theta1, theta2, theta3; // Joint angles
} RobotArm;

// Function to calculate the inverse kinematics
void calculateIK(RobotArm *arm, Position target)
{

  double dop1 = sqrt((target.x * target.x) + (target.y * target.y) +
                     ((target.z - arm->a1) * (target.z - arm->a1)));

  // Calculate theta1
  arm->theta1 = atan2(target.y, target.x);

  // Calculate theta2
  arm->theta2 =
      atan2((-target.z + arm->a1),
            sqrt(target.x * target.x + target.y * target.y)) -
      acos(((arm->a3 * arm->a3) - (arm->a2 * arm->a2) - (dop1 * dop1)) /
           (-2 * arm->a2 * dop1));

  // Calculate theta3
  arm->theta3 =
      PI - acos((dop1 * dop1 - arm->a2 * arm->a2 - arm->a3 * arm->a3) /
                (-2 * arm->a2 * arm->a3));
}

int main()
{

  RobotArm arm;
  Position pos;

  printf("Enter link lengths (L1 L2 L3): \n");
  scanf("%lf %lf %lf", &arm.a1, &arm.a2, &arm.a3);

  printf("Enter Target coordinates (x y z): \n");
  scanf("%lf %lf %lf", &pos.x, &pos.y, &pos.z);

  calculateIK(&arm, pos);

  printf("Joint Angles: \n");

  printf("Theta1: %f radians and %f degrees\n", arm.theta1,
         arm.theta1 * (180 / PI));
  printf("Theta2: %f radians and %f degrees\n", arm.theta2,
         arm.theta2 * (180 / PI));
  printf("Theta3: %f radians and %f degrees\n", arm.theta3,
         arm.theta3 * (180 / PI));

  return 0;
}
