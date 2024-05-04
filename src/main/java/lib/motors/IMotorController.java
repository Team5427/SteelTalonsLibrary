package lib.motors;

public interface IMotorController {

    public void apply(MotorConfiguration configuration);

    public void setSetpoint(double setpoint);

    public double getSetpoint();
    
    public void setEncoderPosition(double position);

    public double getEncoderPosition();

    public double getEncoderVelocity();

    public double getError();

    public void setRawPercentage(double percentage);

    public void setRelativePercentage(double percentage);
}
