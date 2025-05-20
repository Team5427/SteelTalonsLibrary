// package team5427.lib.kinematics.shooter.projectiles;

// import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
// import org.apache.commons.math3.ode.FirstOrderIntegrator;
// import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;

// public class MagnusProjectileSimulation {

//     public static void main(String[] args) {
//         // Initial conditions: x, y, vx, vy
//         double[] y0 = {0, 0, 5, 3}; // Initial position and velocity

//         double t0 = 0.0;
//         double tFinal = 10.0;
//         double stepSize = 0.01;

//         FirstOrderIntegrator integrator = new ClassicalRungeKuttaIntegrator(stepSize);
//         FirstOrderDifferentialEquations ode = new ProjectileODE();

//         integrator.integrate(ode, t0, y0, tFinal, y0);

//         System.out.printf("Final position: (%.2f, %.2f)%n", y0[0], y0[1]);
//         System.out.printf("Final velocity: (%.2f, %.2f)%n", y0[2], y0[3]);
//     }

//     static class ProjectileODE implements FirstOrderDifferentialEquations {
//         // Physical constants
//         final double g = 9.81;
//         final double m = 0.145;              // mass (kg)
//         final double r = 0.1;             // radius (m)
//         final double A = Math.PI * r * r;    // cross-sectional area
//         final double rho = 1.225;            // air density (kg/m^3)
//         final double C_D = 0.47;             // drag coefficient (sphere)
//         final double C_L = 0.2;              // lift coefficient (spin-dependent)
//         final double omegaZ = -100;           // spin around z-axis (rad/s)

//         @Override
//         public int getDimension() {
//             return 4; // [x, y, vx, vy]
//         }

//         @Override
//         public void computeDerivatives(double t, double[] y, double[] dydt) {
//             double vx = y[2];
//             double vy = y[3];
//             double v = Math.sqrt(vx * vx + vy * vy);

//             // Drag force
//             double Fd = 0.5 * rho * C_D * A * v * v;
//             double Fdx = -Fd * (vx / v);
//             double Fdy = -Fd * (vy / v);

//             // Magnus force (lift perpendicular to velocity in 2D)
//             double Fm = 0.5 * rho * C_L * A * v * v;
//             double Fmx = Fm * (vy / v);  // Perpendicular: rotate velocity by +90°
//             double Fmy = -Fm * (vx / v);

//             // Accelerations
//             double ax = (Fdx + Fmx) / m;
//             double ay = (Fdy + Fmy - m * g) / m;

//             System.out.printf("Time: %.2f s | Pos: (%.2f, %.2f) | Vel: (%.2f, %.2f)\n", t, y[0],
// y[1], vx, vy);
//             System.out.printf("  Drag Force: (%.2f, %.2f) | Magnus Force: (%.2f, %.2f)\n", Fdx,
// Fdy, Fmx, Fmy);
//             System.out.printf("  Acceleration: (%.2f, %.2f)\n", ax, ay);

//             // Derivatives
//             dydt[0] = vx;
//             dydt[1] = vy;
//             dydt[2] = ax;
//             dydt[3] = ay;
//         }
//     }
// }
