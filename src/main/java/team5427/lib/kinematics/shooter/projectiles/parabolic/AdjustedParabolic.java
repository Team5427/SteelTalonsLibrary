package team5427.lib.kinematics.shooter.projectiles.parabolic;

import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mult;
import java.io.IOException;
import lombok.Getter;
import lombok.Setter;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.events.EventHandler;
import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import team5427.lib.detection.tuples.Tuple3Plus;

public class AdjustedParabolic {

  @Getter
  private Mult<DistanceUnit, DistanceUnit> kProjectileArea = Meters.of(0.0).times(Meters.of(0.0));

  @Getter private double ballisticCoefficient = 0.0;

  @Setter @Getter private AdjustedParabolicConfiguration configuration;

  @Setter @Getter private Translation3d projectilePositionToTarget;

  double t0 = 0.0;
  double tFinal = 2.0;
  double stepSize = 0.1;
  double vx = 0;
  double vy = 0;
  double w = 0;
  double[] y0 = {0, 0, 0, vx, vy, 0, w}; // Initial position and velocity

  public AdjustedParabolic(AdjustedParabolicConfiguration configuration) {
    this.configuration = configuration;
    kProjectileArea =
        configuration.kProjectileRadius.times(configuration.kProjectileRadius).times(Math.PI);
    // verify if the configuration.kAmbientPressure is converted a double correctly
    configuration.kAirDensity =
        (configuration.kAmbientPressure.toBaseUnits(1.0)
            / (287.058 * configuration.kAmbientTemperature.in(Kelvin)));
    ballisticCoefficient =
        configuration.kProjectileMass.in(Kilogram)
            / (configuration.kProjectileDragCoefficient
                * this.kProjectileArea.in(kProjectileArea.baseUnit()));
  }

  /**
   * @return rotation2d - pivot angle Linear Velocity - Top flywheel velocity Linear Velocity -
   *     Bottom flywheel velocity
   */
  public Tuple3Plus<Rotation2d, LinearVelocity, LinearVelocity> calculateSetpoints() {

    FirstOrderDifferentialEquations ode = this.new NetForceODE();
    double kTopFlywheelConversionFactor =
        configuration.kTopFlywheelRadius.in(Meters) * 2.0 * Math.PI;
    double kBottomFlywheelConversionFactor =
        configuration.kBottomFlywheelRadius.in(Meters) * 2.0 * Math.PI;
    double smallestDistance = Double.MAX_VALUE;
    double[] state = new double[3];
    double[] position = new double[3];
    double[] targetPositionArray =
        new double[] {
          projectilePositionToTarget.getX(),
          projectilePositionToTarget.getY(),
          projectilePositionToTarget.getZ()
        };
    Vector3D targetPositionVector =
        new Vector3D(
            projectilePositionToTarget.getX(),
            projectilePositionToTarget.getY(),
            projectilePositionToTarget.getZ());
    FirstOrderIntegrator integrator = new ClassicalRungeKuttaIntegrator(stepSize);
    integrator.addEventHandler(
        new EventHandler() {
          double previousDistance;
          ArrayRealVector targetPosition;
          RealVector currentState;

          @Override
          public void init(double t0, double[] y0, double t) {
            previousDistance = Double.MAX_VALUE;
            targetPosition = new ArrayRealVector(targetPositionArray);
            currentState = new ArrayRealVector(3);
          }

          @Override
          public double g(double t, double[] y) {
            currentState.setEntry(0, y[0]);
            currentState.setEntry(1, y[1]);
            currentState.setEntry(2, y[2]);
            double currentDistance = currentState.getDistance(targetPosition);

            if (previousDistance < currentDistance) {
              previousDistance = currentDistance;
              return -1;
            } else if (previousDistance == currentDistance) {
              previousDistance = currentDistance;
              return 0;
            } else {
              previousDistance = currentDistance;
              return 1;
            }
          }

          @Override
          public Action eventOccurred(double t, double[] y, boolean increasing) {
            return Action.STOP;
          }

          @Override
          public void resetState(double t, double[] y) {}
        },
        0.01,
        0.01,
        1000);

    double maxFlywheelSpeed = configuration.kFlywheelMaxSpeed.in(RotationsPerSecond);

    // theta in degrees
    for (int theta = 20; theta <= 80; theta += 1) {

      // in rotations per minute
      // Top Flywheel
      for (int omega = 1; omega < maxFlywheelSpeed; omega += 1) {
        double vt = omega * kTopFlywheelConversionFactor;
        // in rotations per minute
        // Bottom Flywheel
        for (int gamma = 1; gamma < maxFlywheelSpeed; gamma += 1) {
          double vb = gamma * kBottomFlywheelConversionFactor;
          double a = (vb + vt) / 2.0;
          double[] y1 = y0.clone();
          y0[3] = Math.cos(theta) * a;
          y0[4] = Math.sin(theta) * a;
          y0[6] = -(vt / kTopFlywheelConversionFactor) + (vb / kBottomFlywheelConversionFactor);
          integrator.integrate(ode, t0, y0, tFinal, y1);
          Vector3D positionVector = new Vector3D(y1[0], y1[1], y1[2]);
          double d = positionVector.distance(targetPositionVector);
          if (d < smallestDistance) {
            smallestDistance = d;
            state[0] = theta;
            state[1] = vt;
            state[2] = vb;
            position[0] = positionVector.getX();
            position[1] = positionVector.getY();
            position[2] = positionVector.getZ();
          }
        }
      }
    }
    System.out.printf("%.2f, %.2f, %.2f \n", position[0], position[1], position[2]);
    return new Tuple3Plus<Rotation2d, LinearVelocity, LinearVelocity>(
        Rotation2d.fromDegrees(state[0]),
        MetersPerSecond.of(state[1]),
        MetersPerSecond.of(state[2]));
  }

  public static void main(String[] args) {

    AdjustedParabolic parabolic = new AdjustedParabolic(new AdjustedParabolicConfiguration());
    Translation3d translation3d = new Translation3d(6, 2, 0);
    long prevTime = System.currentTimeMillis();
    parabolic.setProjectilePositionToTarget(translation3d);

    Object o = parabolic.calculateSetpoints();
    System.out.println((System.currentTimeMillis() - prevTime));
    System.out.println(o);
    System.out.println(translation3d);
    try {
      while (true) {
        int i = System.in.read();
        if (i != -1) {
          prevTime = System.currentTimeMillis();
          o = parabolic.calculateSetpoints();
          System.out.println((System.currentTimeMillis() - prevTime));
          System.out.println(o);
          System.out.println(translation3d);
          System.out.println("I love men :D");
        }
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  public class NetForceODE implements FirstOrderDifferentialEquations {

    @Override
    public int getDimension() {
      return 7; // [x, y, z, vx, vy, vz, w]
    }

    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) {
      double vx = y[3];
      double vy = y[4];
      double vz = y[5];

      double omega = y[6];

      Vector3D omegaVector = new Vector3D(0, 0, omega);
      Vector3D velocityVector = new Vector3D(vx, vy, vz);

      double v = velocityVector.getNorm();

      double s =
          (0.5)
              * (configuration.kAirDensity
                  * kProjectileArea.in(kProjectileArea.baseUnit())
                  * configuration.kProjectileLiftCoefficient
                  * v
                  * v);
      Vector3D magnusForceVector = (Vector3D.crossProduct(omegaVector, velocityVector));
      magnusForceVector.scalarMultiply(s);
      Vector3D gravityForceVector =
          new Vector3D(
              0,
              -configuration.g.in(MetersPerSecondPerSecond)
                  * configuration.kProjectileMass.in(Kilograms),
              0);
      Vector3D dragForceVector =
          new Vector3D(
              -v * configuration.kProjectileMass.in(Kilograms) / ballisticCoefficient,
              velocityVector);

      Vector3D netForceVector = gravityForceVector.add(dragForceVector).add(magnusForceVector);

      Vector3D netAccelerationVector =
          new Vector3D(1.0 / configuration.kProjectileMass.in(Kilograms), netForceVector);

      yDot[0] = velocityVector.getX();
      yDot[1] = velocityVector.getY();
      yDot[2] = velocityVector.getZ();
      yDot[3] = netAccelerationVector.getX();
      yDot[4] = netAccelerationVector.getY();
      yDot[5] = netAccelerationVector.getZ();
      yDot[6] = -Math.copySign(configuration.kProjectileSpinDecay, yDot[6]);
    }
  }
}
