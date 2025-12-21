package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d pivotMotorRotation = Rotation2d.kZero;
    public AngularVelocity pivotMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration pivotMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);

    public LinearVelocity rollerMotorLinearVelocity = MetersPerSecond.of(0.0);
    public LinearAcceleration rollerMotorLinearAcceleration = MetersPerSecondPerSecond.of(0.0);
    public AngularVelocity rollerMotorAngularVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration rollerMotorAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);

    public Current pivotMotorCurrent = Amps.of(0.0);
    public Current rollerMotorCurrent = Amps.of(0.0);

    public Voltage pivotMotorVoltage = Volts.of(0.0);
    public Voltage rollerMotorVoltage = Volts.of(0.0);

    public Temperature pivotMotorTemperature = Celsius.of(0.0);
    public Temperature rollerMotorTemperature = Celsius.of(0.0);

    public boolean pivotMotorConnected = false;
    public boolean rollerMotorConnected = false;

    public boolean pivotMotorDisabled = false;
    public boolean rollerMotorDisabled = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotRotation(Rotation2d rotation) {}

  public default void setPivotRotation(Angle rotation) {}

  public default void setRollerSpeed(LinearVelocity velocity) {}

  public default void setRollerSpeed(AngularVelocity velocity) {}

  public default void disableRollerMotor(boolean shouldDisable) {}

  public default void disablePivotMotor(boolean shouldDisable) {}
}
