package team5427.lib.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.MotorConfiguration.MotorMode;

public class SteelTalonFX implements IMotorController {

  private CANDeviceId id;
  private TalonFX talonFX;
  private MotorConfiguration configuration;

  private double setpoint;

  private boolean withFOC;

  public TalonFXConfiguration talonConfig;

  public TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(Amps.of(0.0));
  public PositionTorqueCurrentFOC positionTorqueCurrentFOCRequest =
      new PositionTorqueCurrentFOC(Rotation.of(0.0));
  public PositionVoltage positionVoltageRequest = new PositionVoltage(Rotation.of(0.0));
  public VelocityVoltage velocityVoltageRequest = new VelocityVoltage(RotationsPerSecond.of(0.0));
  public PositionDutyCycle positionDutyCycleRequest = new PositionDutyCycle(Rotation.of(0.0));
  public VelocityDutyCycle velocityDutyCycleRequest =
      new VelocityDutyCycle(RotationsPerSecond.of(0.0));
  public VelocityTorqueCurrentFOC velocityTorqueCurrentFOCRequest =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0.0));
  public VoltageOut voltageOut = new VoltageOut(Volt.of(0.0));
  private boolean useTorqueCurrentFOC = false;

  // private boolean usePositionVoltage = false;

  public SteelTalonFX(CANDeviceId id) {
    this.id = id;

    talonFX = new TalonFX(this.id.getDeviceNumber(), this.id.getBus());

    withFOC = false;
  }

  @Override
  public void apply(MotorConfiguration configuration) {
    this.configuration = configuration;
    talonConfig = new TalonFXConfiguration();

    talonConfig.Feedback.SensorToMechanismRatio =
        configuration.gearRatio.getSensorToMechanismRatio();

    switch (configuration.idleState) {
      case kBrake:
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        break;
      case kCoast:
        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        break;
      default:
        break;
    }
    talonConfig.MotorOutput.Inverted =
        configuration.isInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    talonConfig.Slot0.kP = configuration.kP;
    talonConfig.Slot0.kI = configuration.kI;
    talonConfig.Slot0.kD = configuration.kD;
    talonConfig.Slot0.kS = configuration.kS;
    talonConfig.Slot0.kV = configuration.kV;
    talonConfig.Slot0.kA = configuration.kA;
    talonConfig.Slot0.kG = configuration.kG;
    talonConfig.Slot0.GravityType =
        configuration.isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;

    withFOC = configuration.withFOC;
    talonConfig.FutureProofConfigs = true;

    switch (configuration.mode) {
      case kFlywheel:
      case kLinear:
        talonConfig.ClosedLoopGeneral.ContinuousWrap = false;
        break;
      case kServo:
        talonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        break;
      default:
        break;
    }

    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.StatorCurrentLimit = configuration.currentLimit;
    talonConfig.CurrentLimits.SupplyCurrentLimit = configuration.currentLimit * 0.5;

    talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = configuration.currentLimit;
    talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -configuration.currentLimit;
    talonFX.getConfigurator().apply(talonConfig);
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public void setEncoderPosition(double position) {
    talonFX.setPosition(position);
  }

  @Override
  public void setEncoderPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  /**
   * @return rotations if a servo, or meters if a flywheel or linear
   */
  public double getEncoderPosition(StatusSignal<Angle> position) {

    return position.getValue().in(Rotation) * getConversionFactorFromRotations();
  }

  /**
   * @return rotations per minute if a servo, meters per second if a linear or flywheel
   */
  public double getEncoderVelocity(StatusSignal<AngularVelocity> velocity) {
    return velocity.getValue().in(RotationsPerSecond)
        * getConversionFactorFromRotations()
        * (configuration.mode == MotorMode.kServo ? 60.0 : 1.0);
  }

  /**
   * @return rotations per minute^2 if a servo, meters per second^2 if a linear or flywheel
   */
  public double getEncoderAcceleration(StatusSignal<AngularAcceleration> acceleration) {
    return acceleration.getValue().in(RotationsPerSecondPerSecond)
        * getConversionFactorFromRotations()
        * (configuration.mode == MotorMode.kServo ? 60.0 : 1.0);
  }

  /**
   * Conversion factors are calculated without the gear ratio (but with wheel final diameter if
   * linear or flywheel) Gear ratios are applied automatically by talonfx
   *
   * @return Flywheel - returns m/s -> rotations/s Linear - returns m -> rotations Servo - returns
   *     rotations -> rotations
   *     <p>This also applies to the integrals and derivatives of these units (ie. m/s -> m OR m/s
   *     -> m/s^2)
   */
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

  /**
   * Conversion factors are calculated without the gear ratio (but with wheel final diameter if
   * linear or flywheel) Gear ratios are applied automatically by talonfx
   *
   * @return Flywheel - returns rotations/s -> m/s Linear - returns rotations -> m Servo - returns
   *     rotations -> rotations
   *     <p>This also applies to the integrals and derivatives of these units (ie. m/s -> m OR m/s
   *     -> m/s^2)
   */
  @Override
  public double getConversionFactorFromRotations() {
    return 1.0 / getConversionFactorToRotations();
  }

  @Override
  public void setRawPercentage(double percentage) {
    talonFX.set(percentage);
  }

  @Override
  public void setRelativePercentage(double percentage) {
    talonFX.setVoltage(percentage * talonFX.getSupplyVoltage().getValueAsDouble());
  }

  @Override
  public void setRawVoltage(Voltage voltage) {
    talonFX.setControl(voltageOut.withOutput(voltage).withEnableFOC(withFOC));
  }

  public void setRawCurrent(Current current) {
    talonFX.setControl(torqueCurrentFOCRequest.withOutput(current));
  }

  @Override
  public double getError() {
    return talonFX.getClosedLoopError().getValue();
  }

  public void setFOC(boolean foc) {
    withFOC = foc;
  }

  public TalonFX getTalonFX() {
    return talonFX;
  }

  @Override
  public void setSetpoint(Distance distance) {
    this.setpoint = distance.in(Meter);
    switch (configuration.mode) {
      case kLinear:
        this.setpoint *= getConversionFactorToRotations();
        talonFX.setControl(
            isUsingTorqueCurrentFOC()
                ? positionTorqueCurrentFOCRequest.withPosition(this.setpoint)
                : positionDutyCycleRequest.withPosition(setpoint).withEnableFOC(withFOC));
        break;
      default:
        DriverStation.reportWarning(
            "SteelTalonFX: id "
                + id.getDeviceNumber()
                + " in bus "
                + id.getBus()
                + " using Distance setpoint on a non-linear motor",
            false);
        break;
    }
  }

  @Override
  public void setSetpoint(LinearVelocity velocity) {
    this.setpoint = velocity.in(MetersPerSecond);
    switch (configuration.mode) {
      case kFlywheel:
        this.setpoint *= getConversionFactorToRotations();
        talonFX.setControl(
            isUsingTorqueCurrentFOC()
                ? velocityTorqueCurrentFOCRequest.withVelocity(this.setpoint)
                : velocityVoltageRequest.withVelocity(setpoint).withEnableFOC(withFOC));
        break;
      default:
        DriverStation.reportWarning(
            "SteelTalonFX: id "
                + id.getDeviceNumber()
                + " in bus "
                + id.getBus()
                + " using LinearVelocity setpoint on a non-flywheel motor",
            false);
        break;
    }
  }

  @Override
  public void setSetpoint(AngularVelocity velocity) {
    this.setpoint = velocity.in(RotationsPerSecond);
    switch (configuration.mode) {
      case kFlywheel:
        talonFX.setControl(
            isUsingTorqueCurrentFOC()
                ? velocityTorqueCurrentFOCRequest.withVelocity(this.setpoint)
                : velocityVoltageRequest.withVelocity(setpoint).withEnableFOC(withFOC));
        break;
      default:
        DriverStation.reportWarning(
            "SteelTalonFX: id "
                + id.getDeviceNumber()
                + " in bus "
                + id.getBus()
                + " using AngularVelocity setpoint on a non-flywheel motor",
            false);
        break;
    }
  }

  @Override
  public void setSetpoint(Angle angle) {
    this.setpoint = angle.in(Rotation);
    switch (configuration.mode) {
      case kServo:
        this.setpoint *= getConversionFactorToRotations();
        talonFX.setControl(
            isUsingTorqueCurrentFOC()
                ? positionTorqueCurrentFOCRequest.withPosition(this.setpoint)
                : positionDutyCycleRequest.withPosition(setpoint).withEnableFOC(withFOC));
        break;
      default:
        DriverStation.reportWarning(
            "SteelTalonFX: id "
                + id.getDeviceNumber()
                + " in bus "
                + id.getBus()
                + " using Angle setpoint on a non-servo motor",
            false);
        break;
    }
  }

  @Override
  public void setSetpoint(Rotation2d angle) {
    this.setpoint = angle.getRotations();
    switch (configuration.mode) {
      case kServo:
        this.setpoint *= getConversionFactorToRotations();
        talonFX.setControl(
            isUsingTorqueCurrentFOC()
                ? positionTorqueCurrentFOCRequest.withPosition(this.setpoint)
                : positionDutyCycleRequest.withPosition(setpoint).withEnableFOC(withFOC));
        break;
      default:
        DriverStation.reportWarning(
            "SteelTalonFX: id "
                + id.getDeviceNumber()
                + " in bus "
                + id.getBus()
                + " using Rotation2d setpoint on a non-servo motor",
            false);
        break;
    }
  }

  public void applyTalonConfig() {
    talonFX.getConfigurator().apply(this.talonConfig);
  }

  @Override
  public MotorConfiguration getMotorConfiguration() {
    return this.configuration;
  }

  public boolean isUsingTorqueCurrentFOC() {
    return this.useTorqueCurrentFOC;
  }

  public void useTorqueCurrentFOC(boolean using) {
    this.useTorqueCurrentFOC = using;
  }
}
