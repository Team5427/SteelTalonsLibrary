package lib.motors;

import lib.motors.real.MotorConfiguration;
import lib.motors.simulated.SimMotorConfiguration;

public interface IMotorController {

    public default void apply(MotorConfiguration configuration) {
    }

    public default void apply(SimMotorConfiguration configuration) {
    }

    public void setSetpoint(double setpoint);

    public double getSetpoint();

    public void setEncoderPosition(double position);

    public double getEncoderPosition();

    public double getEncoderVelocity();

    public double getError();

    public void setRawPercentage(double percentage);

    public void setRelativePercentage(double percentage);
}
