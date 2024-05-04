package lib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lib.drivers.CANDeviceId;
import lib.motors.MotorConfiguration.IdleState;
import lib.motors.MotorConfiguration.MotorMode;

public class SimpleSparkMax implements IMotorController {

    private CANDeviceId id;
    private CANSparkMax sparkMax;
    private MotorConfiguration configuration;
    private RelativeEncoder relativeEncoder;
    private SparkPIDController controller;

    private ControlType controlType;

    private double setpoint;

    public SimpleSparkMax(CANDeviceId id) {
        this.id = id;

        sparkMax = new CANSparkMax(this.id.getDeviceNumber(), MotorType.kBrushless);

        relativeEncoder = sparkMax.getEncoder();
        relativeEncoder.setMeasurementPeriod(10);

        controller = sparkMax.getPIDController();
    }

    @Override
    public void apply(MotorConfiguration configuration) {
        this.configuration = configuration;
        sparkMax.setInverted(configuration.isInverted);
        sparkMax.setIdleMode(configuration.idleState == IdleState.kBrake ? IdleMode.kBrake: IdleMode.kCoast);
        
        relativeEncoder.setPositionConversionFactor(configuration.unitConversionRatio);
        relativeEncoder.setVelocityConversionFactor(configuration.unitConversionRatio / 60.0);
        
        controller.setP(configuration.kP);
        controller.setI(configuration.kI);
        controller.setD(configuration.kD);
        controller.setFF(configuration.kFF);

        switch (configuration.mode) {
            case kFlywheel:
                controlType = ControlType.kVelocity;
                break;
            case kServo:
            case kLinear:
                controlType = ControlType.kPosition;
                break;
            default:
                controlType = ControlType.kVelocity;
                break;
        }

        sparkMax.burnFlash();
    }

    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        controller.setReference(this.setpoint, controlType);
    }

    public void setSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint.getRadians();
        if (configuration.mode == MotorMode.kFlywheel) {
            DriverStation.reportWarning(
                "Simple Spark Max of id " + id.getDeviceNumber() + " of type flywheel was set with Rotation2d setpoint.", 
                true);
        }
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }

    @Override
    public double getEncoderPosition() {
        return relativeEncoder.getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return relativeEncoder.getVelocity();
    }

    @Override
    public void setRawPercentage(double percentage) {
        sparkMax.set(percentage);
    }

    @Override
    public void setRelativePercentage(double percentage) {
        sparkMax.setVoltage(percentage * sparkMax.getBusVoltage());
    }

    @Override
    public double getError() {
        if (configuration.mode == MotorMode.kFlywheel) {
            return setpoint - getEncoderVelocity();
        }
        return setpoint - getEncoderPosition();
    }

    public CANSparkMax getSparkMax() {
        return sparkMax;
    }

    public RelativeEncoder getRelativeEncoder() {
        return relativeEncoder;
    }

    public MotorConfiguration getConfiguration() {
        return configuration;
    }

}
