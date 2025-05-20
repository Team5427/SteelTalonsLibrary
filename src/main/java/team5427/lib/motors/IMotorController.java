package team5427.lib.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface IMotorController {

  public void apply(MotorConfiguration configuration);

  public MotorConfiguration getMotorConfiguration();

  public default double getSetpoint() {
    return 0.0;
  }

  public void setEncoderPosition(double position);

  public void setEncoderPosition(Rotation2d position);

  public default double getEncoderPosition() {
    return 0.0;
  }

  public default double getEncoderVelocity() {
    return 0.0;
  }

  public default double getError() {
    return 0.0;
  }

  public void setRawPercentage(double percentage);

  public void setRelativePercentage(double percentage);

  public void setRawVoltage(Voltage voltage);

  public default void setRawCurrent(Current current) {}

  public void setSetpoint(Distance distance);

  public void setSetpoint(LinearVelocity velocity);

  public void setSetpoint(AngularVelocity velocity);

  public void setSetpoint(Rotation2d angle);

  public void setSetpoint(Angle angle);

  public double getConversionFactorFromRotations();

  public double getConversionFactorToRotations();
}
