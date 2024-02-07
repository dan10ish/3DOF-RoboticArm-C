#include <stdio.h>
#include <math.h>

// Define the function f(y, t)
double f(double y, double t) {
    // Example: dy/dt = y - t
    return y - t;
}

// Runge-Kutta method function
void rungeKutta(double y0, double t0, double tn, int steps) {
    double h = (tn - t0) / steps;
    double y = y0;
    double t = t0;
    double k1, k2, k3, k4;

    for (int i = 0; i < steps; i++) {
        k1 = h * f(y, t);
        k2 = h * f(y + 0.5 * k1, t + 0.5 * h);
        k3 = h * f(y + 0.5 * k2, t + 0.5 * h);
        k4 = h * f(y + k3, t + h);
        y = y + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
        t = t + h;
        printf("At t=%.2f, y=%.4f\n", t, y);
    }
}

int main() {
    // Initial conditions
    double y0 = 1.0; // Initial value of y
    double t0 = 0.0; // Initial time
    double tn = 2.0; // Final time
    int steps = 10;  // Number of steps

    rungeKutta(y0, t0, tn, steps);

    return 0;
}

