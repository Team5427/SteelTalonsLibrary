package team5427.lib.kinematics.shooter.projectiles.parabolic;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.ForceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import lombok.Getter;
import lombok.Setter;
import team5427.lib.drivers.ComplexGearRatio;

public class AdjustedParabolicConfiguration {

  // Environmental Constants

  @Getter @Setter public double kAirDensity = 1.293;

  @Getter @Setter public LinearAcceleration g = MetersPerSecondPerSecond.of(9.80665);

  @Getter @Setter public Temperature kAmbientTemperature = Celsius.of(20.0);

  @Getter @Setter
  public PerUnit<PerUnit<ForceUnit, DistanceUnit>, DistanceUnit> kAmbientPressure =
      PerUnit.combine(PerUnit.combine(Newton, Meter), Meter);

  // Ball Properties

  @Getter @Setter public Mass kProjectileMass = Kilogram.of(0.4);

  @Getter @Setter public Distance kProjectileRadius = Inches.of(8.5 / 2.0);

  @Getter @Setter
  public MomentOfInertia kProjectileMomentOfInertia =
      KilogramSquareMeters.of(
          0.4 * kProjectileMass.in(Kilogram) * Math.pow(kProjectileRadius.in(Meter), 2));

  // Aerodynamic Coefficients

  @Getter @Setter public double kProjectileDragCoefficient = 0.47;

  @Getter @Setter public double kProjectileLiftCoefficient = 0.2;

  @Getter @Setter public double kProjectileSpinDecay = 10e-3;

  // Flywheel and Shooter Hardware

  @Getter @Setter public Distance kTopFlywheelRadius = Inches.of(2.0);

  @Getter @Setter public Distance kBottomFlywheelRadius = Inches.of(2.0);

  @Getter @Setter public MomentOfInertia kFlywheelMomentOfInertia = KilogramSquareMeters.of(0.001);

  @Getter @Setter public AngularVelocity kFlywheelMaxSpeed = RotationsPerSecond.of(4000.0 / 60.0);

  @Getter @Setter public ComplexGearRatio kGearRatio = new ComplexGearRatio(1.0);

  @Getter @Setter public Distance kHoodArcRadius = Meters.of(0.1);

  @Getter @Setter public double kFlywheelCompression = 0.01;

  @Getter @Setter
  public Per<ForceUnit, DistanceUnit> kEffectiveStiffness = Newton.of(0.1).per(Meters);

  // Friction, Slip and Contact

  @Getter @Setter public double kCoefficientOfKineticFriction = 0.4;

  @Getter @Setter public double kCoefficientOfStaticFriction = 0.6;

  @Getter @Setter public double kSlipThreshold = 0.1;

  @Getter @Setter public double kEnergyLossFactor = 0.1;

  // Solver knobs

  @Getter @Setter public Time kTimeStep = Milliseconds.of(0.1);

  @Getter @Setter public Time kMaxTime = Milliseconds.of(10.0);

  @Getter @Setter public Distance kPositionTolerance = Centimeter.of(0.5);

  @Getter @Setter public LinearVelocity kVelocityTolerance = MetersPerSecond.of(0.01);
}
