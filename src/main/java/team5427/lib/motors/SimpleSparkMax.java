package team5427.lib.motors;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;

public class SimpleSparkMax implements IMotorController {

  private CANDeviceId id;
  private SparkMax sparkMax;
  private MotorConfiguration configuration;
  private SparkClosedLoopController controller;
  private SparkMaxConfig config;
  private SparkBase.ControlType controlType;

  private double setpoint;

  public SimpleSparkMax(CANDeviceId id) {
    this.id = id;
    sparkMax = new SparkMax(id.getDeviceNumber(), MotorType.kBrushless);

    config = new SparkMaxConfig();
    controller = sparkMax.getClosedLoopController();
  }

  @Override
  public void apply(MotorConfiguration configuration) {
    this.configuration = configuration;
    config
        .inverted(configuration.isInverted)
        .idleMode(configuration.idleState == IdleState.kBrake ? IdleMode.kBrake : IdleMode.kCoast);

    config.closedLoop.pidf(configuration.kP, configuration.kI, configuration.kD, configuration.kFF);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.signals.appliedOutputPeriodMs(10);

    switch (configuration.mode) {
      case kFlywheel:
        controlType = SparkBase.ControlType.kVelocity;
        break;
      case kServo:
        controlType = SparkBase.ControlType.kPosition;
        // configured for rotations rather than radians
        config
            .closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(-0.5)
            .positionWrappingMaxInput(0.5);
        break;
      case kLinear:
        controlType = SparkBase.ControlType.kPosition;
        // configured for rotations rather than radians
        config
            .closedLoop
            .positionWrappingEnabled(false)
            .positionWrappingMinInput(-0.5)
            .positionWrappingMaxInput(0.5);
        break;
      default:
        controlType = SparkBase.ControlType.kVelocity;
        break;
    }

    config.smartCurrentLimit(configuration.currentLimit);
    config.voltageCompensation(12.0);

    applySparkConfiguration(config);

    // sparkMax.burnFlash();
  }

  /*
   * Function now uses rotations inside rather than radians
   */
  @Override
  public void setSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint.getRotations();
    if (configuration.mode == MotorMode.kFlywheel) {
      DriverStation.reportWarning(
          "Simple Spark Max of id "
              + id.getDeviceNumber()
              + " of type flywheel was set with Rotation2d setpoint.",
          true);
    }

    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * @param position - In rotations
   */
  @Override
  public void setEncoderPosition(double position) {
    sparkMax.getEncoder().setPosition(position);
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    /** Will now use rotations */
    sparkMax.getEncoder().setPosition(position.getRotations());
  }

  @Override
  public double getEncoderPosition() {
    return sparkMax.getEncoder().getPosition();
  }

  public SparkClosedLoopController getClosedLoopController() {
    return this.controller;
  }

  /**
   * @return RPM of the motor
   */
  @Override
  public double getEncoderVelocity() {
    return sparkMax.getEncoder().getVelocity();
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

  public SparkMax getSparkMax() {
    return sparkMax;
  }

  public RelativeEncoder getRelativeEncoder() {
    return sparkMax.getEncoder();
  }

  @Override
  public MotorConfiguration getMotorConfiguration() {
    return this.configuration;
  }

  public SparkMaxConfig getSparkMaxConfig() {
    return config;
  }

  @Override
  public void setRawVoltage(Voltage voltage) {
    sparkMax.setVoltage(voltage);
  }

  @Override
  public void setSetpoint(Distance distance) {
    this.setpoint = distance.in(Meter) * getConversionFactorToRotations();
    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public void setSetpoint(LinearVelocity velocity) {
    this.setpoint = velocity.in(MetersPerSecond) * getConversionFactorToRotations();
    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public void setSetpoint(AngularVelocity velocity) {
    this.setpoint = velocity.in(RotationsPerSecond) * 60.0;
    // API needs RPM, not RPS
    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public void setSetpoint(Angle angle) {
    this.setpoint = angle.in(Rotation);
    controller.setReference(this.setpoint, controlType);
  }

  @Override
  public double getConversionFactorFromRotations() {
    return 1.0 / getConversionFactorToRotations();
  }

  public boolean isConnected() {
    return sparkMax.hasStickyFault();
  }

  @Override
  public double getConversionFactorToRotations() {
    switch (configuration.mode) {
      case kLinear:
      case kFlywheel:
        return 1.0 / (Math.PI * configuration.finalDiameterMeters);
      case kServo:
        return 1.0;
      default:
        return 1.0;
    }
  }

  public void applySparkConfiguration(SparkMaxConfig config) {
    this.config = config;
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
