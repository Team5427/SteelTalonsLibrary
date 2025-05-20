package team5427.frc.robot.subsystems.Swerve.io.talon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newton;
import static edu.wpi.first.units.Units.NewtonMeter;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIO;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIO.ModuleIOInputs;
import team5427.lib.motors.SteelTalonFX;

public abstract class ModuleIOTalonFX implements ModuleIO {

  protected final SteelTalonFX steerMotor;
  protected final SteelTalonFX driveMotor;

  protected final CANcoder cancoder;

  protected final int moduleIdx;

  protected SwerveModuleState targetModuleState;
  protected StatusSignal<Voltage> steerMotorVoltage;
  protected StatusSignal<Voltage> driveMotorVoltage;
  protected StatusSignal<Angle> absolutePosition;
  protected StatusSignal<Current> steerMotorCurrent;
  protected StatusSignal<Current> driveMotorCurrent;
  protected StatusSignal<Current> driveTorqueCurrent;
  protected StatusSignal<Current> steerTorqueCurrent;
  protected StatusSignal<Angle> driveMotorPosition;
  protected StatusSignal<Angle> steerMotorPosition;
  protected StatusSignal<AngularVelocity> driveMotorVelocity;

  public ModuleIOTalonFX(SwerveModuleConstants moduleConstants, int moduleIdx) {
    System.out.println(moduleConstants.DriveMotorGains);
    this.moduleIdx = moduleIdx;
    driveMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
    steerMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);
    cancoder =
        new CANcoder(
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getDeviceNumber(),
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getBus());
    cancoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5))
                    .withMagnetOffset(moduleConstants.EncoderOffset)
                    .withSensorDirection(
                        moduleConstants.EncoderInverted
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive));
    cancoder.clearStickyFaults();

    driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);
    steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);

    if (moduleConstants.DriveMotorClosedLoopOutput == ClosedLoopOutputType.TorqueCurrentFOC) {
      driveMotor.setFOC(true);
      driveMotor.useTorqueCurrentFOC(true);
    } else if (moduleConstants.DriveMotorClosedLoopOutput == ClosedLoopOutputType.Voltage) {
      driveMotor.useTorqueCurrentFOC(false);
    }

    if (moduleConstants.SteerMotorClosedLoopOutput == ClosedLoopOutputType.TorqueCurrentFOC) {
      steerMotor.setFOC(true);
      steerMotor.useTorqueCurrentFOC(true);
    } else if (moduleConstants.SteerMotorClosedLoopOutput == ClosedLoopOutputType.Voltage) {
      steerMotor.useTorqueCurrentFOC(false);
    }

    driveMotor.talonConfig =
        driveMotor
            .talonConfig
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        moduleConstants.DriveMotorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(moduleConstants.SlipCurrent))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(moduleConstants.SlipCurrent)
                    .withPeakReverseTorqueCurrent(-moduleConstants.SlipCurrent))
            .withSlot0(moduleConstants.DriveMotorGains);
    steerMotor.talonConfig =
        steerMotor
            .talonConfig
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        moduleConstants.SteerMotorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withSlot0(moduleConstants.SteerMotorGains);
    cancoder.clearStickyFaults();
    absolutePosition = cancoder.getAbsolutePosition();

    steerMotor.talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    steerMotor.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotor.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotor.talonConfig.Feedback.RotorToSensorRatio = moduleConstants.SteerMotorGearRatio;
    driveMotor.applyTalonConfig();
    steerMotor.applyTalonConfig();

    driveMotor.setEncoderPosition(0.0);
    steerMotor.setEncoderPosition(absolutePosition.refresh().getValue().in(Rotations));

    driveMotorPosition = driveMotor.getTalonFX().getPosition();
    steerMotorPosition = steerMotor.getTalonFX().getPosition();

    driveMotorVelocity = driveMotor.getTalonFX().getVelocity();

    driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage();
    steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage();
    driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent();
    steerMotorCurrent = steerMotor.getTalonFX().getStatorCurrent();
    driveTorqueCurrent = driveMotor.getTalonFX().getTorqueCurrent();
    steerTorqueCurrent = steerMotor.getTalonFX().getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kOdometryFrequency, driveMotorPosition, steerMotorPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveMotorVoltage,
        steerMotorVoltage,
        driveMotorCurrent,
        steerMotorCurrent,
        absolutePosition,
        driveMotorVelocity);

    ParentDevice.optimizeBusUtilizationForAll(
        driveMotor.getTalonFX(), steerMotor.getTalonFX(), cancoder);

    BaseStatusSignal.waitForAll(
        0.02, absolutePosition, driveMotorPosition, steerMotorPosition, driveMotorVelocity);

    System.out.println("New Module with idx: " + moduleIdx);
  }

  public ModuleIOTalonFX(int moduleIdx) {

    this.moduleIdx = moduleIdx;

    driveMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[moduleIdx]);
    steerMotor = new SteelTalonFX(SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[moduleIdx]);
    cancoder =
        new CANcoder(
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getDeviceNumber(),
            SwerveConstants.kSwerveUtilInstance.kCancoderIds[moduleIdx].getBus());
    SwerveConstants.kDriveMotorConfiguration.isInverted =
        SwerveConstants.kSwerveUtilInstance.kDriveInversion[moduleIdx];
    SwerveConstants.kSteerMotorConfiguration.isInverted =
        SwerveConstants.kSwerveUtilInstance.kSteerInversion[moduleIdx];
    driveMotor.apply(SwerveConstants.kDriveMotorConfiguration);
    steerMotor.apply(SwerveConstants.kSteerMotorConfiguration);

    driveMotor.getTalonFX().clearStickyFaults();
    steerMotor.getTalonFX().clearStickyFaults();

    CANcoderConfiguration configuration = new CANcoderConfiguration();
    configuration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
    configuration.MagnetSensor.MagnetOffset =
        SwerveConstants.kSwerveUtilInstance.kModuleOffsets[moduleIdx];
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(configuration);

    cancoder.clearStickyFaults();
    absolutePosition = cancoder.getAbsolutePosition();

    steerMotor.talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    steerMotor.talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotor.talonConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerMotor.talonConfig.Feedback.RotorToSensorRatio =
        SwerveConstants.kSteerMotorConfiguration.gearRatio.getSensorToMechanismRatio();
    steerMotor.getTalonFX().getConfigurator().apply(steerMotor.talonConfig);

    steerMotor.setEncoderPosition(absolutePosition.refresh().getValue().in(Rotations));
    driveMotor.setEncoderPosition(0.0);
    driveMotor.useTorqueCurrentFOC(true);

    // steerMotor.useTorqueCurrentFOC(false);
    // steerMotor.usePositionVoltage(true);

    driveMotorPosition = driveMotor.getTalonFX().getPosition();
    steerMotorPosition = steerMotor.getTalonFX().getPosition();

    driveMotorVelocity = driveMotor.getTalonFX().getVelocity();

    System.out.println("New Module with idx: " + moduleIdx);

    driveMotorVoltage = driveMotor.getTalonFX().getMotorVoltage();
    steerMotorVoltage = steerMotor.getTalonFX().getMotorVoltage();
    driveMotorCurrent = driveMotor.getTalonFX().getStatorCurrent();
    steerMotorCurrent = steerMotor.getTalonFX().getStatorCurrent();
    driveTorqueCurrent = driveMotor.getTalonFX().getTorqueCurrent();
    steerTorqueCurrent = steerMotor.getTalonFX().getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kOdometryFrequency, driveMotorPosition, steerMotorPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveMotorVoltage,
        steerMotorVoltage,
        driveMotorCurrent,
        steerMotorCurrent,
        absolutePosition,
        driveMotorVelocity);

    ParentDevice.optimizeBusUtilizationForAll(
        driveMotor.getTalonFX(), steerMotor.getTalonFX(), cancoder);

    BaseStatusSignal.waitForAll(
        0.02, absolutePosition, driveMotorPosition, steerMotorPosition, driveMotorVelocity);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(driveMotorPosition, steerMotorPosition);

    BaseStatusSignal.refreshAll(
        steerMotorVoltage,
        driveMotorVoltage,
        driveMotorCurrent,
        steerMotorCurrent,
        driveMotorVelocity,
        absolutePosition);

    inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));

    inputs.driveMotorPosition = Rotation2d.fromRadians(driveMotorPosition.getValue().in(Radian));
    inputs.driveMotorAngularVelocity = driveMotorVelocity.getValue();
    inputs.driveMotorLinearVelocity =
        MetersPerSecond.of(
            driveMotorVelocity.getValue().in(RotationsPerSecond)
                * Math.PI
                * driveMotor.getMotorConfiguration().finalDiameterMeters);

    inputs.steerPosition =
        Rotation2d.fromRotations(steerMotor.getEncoderPosition(steerMotorPosition));

    inputs.currentModuleState =
        new SwerveModuleState(
            driveMotor.getEncoderVelocity(driveMotorVelocity), inputs.absolutePosition);
    inputs.currentModulePosition =
        new SwerveModulePosition(
            driveMotor.getEncoderPosition(driveMotorPosition), inputs.absolutePosition);

    inputs.driveMotorVoltage = driveMotorVoltage.getValue();
    inputs.steerMotorVoltage = steerMotorVoltage.getValue();

    inputs.driveMotorRotations = driveMotorPosition.getValueAsDouble();
    inputs.driveMotorRotationsPerSecond = driveMotorVelocity.getValueAsDouble();

    inputs.driveMotorConnected = driveMotor.getTalonFX().isConnected();
    inputs.steerMotorConnected = steerMotor.getTalonFX().isConnected();

    inputs.driveMotorCurrent = driveMotorCurrent.getValue();
    inputs.steerMotorCurrent = steerMotorCurrent.getValue();

    inputs.driveTorqueCurrent = driveTorqueCurrent.getValue();
    inputs.steerTorqueCurrent = steerTorqueCurrent.getValue();
  }

  @Override
  public void resetMotorSetpoint(Rotation2d steerPosition) {
    steerMotor.setEncoderPosition(steerPosition);
    driveMotor.setEncoderPosition(0.0);
  }

  @Override
  public void setDriveFeedForward(Force xForce, Force yForce) {
    edu.wpi.first.math.Vector<N2> driveForces =
        VecBuilder.fill(xForce.in(Newton), yForce.in(Newton));
    edu.wpi.first.math.Vector<N2> wheelDirection =
        VecBuilder.fill(
            Rotation2d.fromRadians(steerMotorPosition.getValue().in(Radian)).getCos(),
            Rotation2d.fromRadians(steerMotorPosition.getValue().in(Radian)).getSin());
    Torque wheelTorque =
        NewtonMeter.of(
            driveForces.dot(wheelDirection) * SwerveConstants.kWheelDiameterMeters / 2.0);

    driveMotor.velocityTorqueCurrentFOCRequest.withFeedForward(
        Amps.of(
            (wheelTorque.in(NewtonMeter)
                    / driveMotor.getTalonFX().getMotorKT().refresh().getValueAsDouble())
                * (driveMotor.getMotorConfiguration().gearRatio.getSensorToMechanismRatio())));
  }

  /**
   * Needs to be a <strong>Double</strong>, NOT a double
   *
   * @param speed - <strong>Double</strong>, setpoint for drive motor (module speed)
   */
  @Override
  public void setDriveSpeedSetpoint(LinearVelocity speed) {
    driveMotor.setSetpoint(speed);
  }

  @Override
  public void setDriveFeedForward(Current current) {
    driveMotor.getMotorConfiguration().kFF = current.in(Amps);
  }

  @Override
  public void setDriveSpeedSetpoint(Voltage volts) {
    driveMotor.setRawVoltage(volts);
  }

  @Override
  public void setDriveSpeedSetpoint(Current current) {
    driveMotor.setRawCurrent(current);
  }

  @Override
  public void setSteerPositionSetpoint(Current current) {
    steerMotor.setRawCurrent(current);
  }

  @Override
  public void setSteerPositionSetpoint(Voltage volts) {
    steerMotor.setRawVoltage(volts);
  }

  /**
   * @param setpoint - <strong>Rotation2d</strong> setpoint for steer motor (module angle)
   */
  @Override
  public void setSteerPositionSetpoint(Rotation2d setpoint) {
    steerMotor.setSetpoint(setpoint);
  }

  @Override
  public void setModuleState(SwerveModuleState state) {
    Rotation2d currentAngle = Rotation2d.fromRotations(absolutePosition.getValue().in(Rotations));

    state.cosineScale(currentAngle);

    targetModuleState = state;
    setDriveSpeedSetpoint(MetersPerSecond.of(targetModuleState.speedMetersPerSecond));
    setSteerPositionSetpoint(targetModuleState.angle);
  }
}
