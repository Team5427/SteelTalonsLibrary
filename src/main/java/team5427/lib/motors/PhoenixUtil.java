// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package team5427.lib.motors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;

public final class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Attempts to run the command 5 times until no error is produced. */
  public static void tryUntilOk(Supplier<StatusCode> command) {
    for (int i = 0; i < 5; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;

      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }

  /**
   * @param index of the module
   * @return the Swerve Module Constants of a a swerve module based on the motor configurations of
   *     the swerve module
   */
  public static SwerveModuleConstants getSwerveModuleConstants(int index) {
    SwerveModuleConstants moduleConstants = new SwerveModuleConstants<>();
    moduleConstants.CouplingGearRatio = SwerveConstants.kCoupleRatio;
    moduleConstants.DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
    moduleConstants.DriveMotorId =
        SwerveConstants.kSwerveUtilInstance.kDriveMotorIds[index].getDeviceNumber();
    moduleConstants.DriveMotorInverted = SwerveConstants.kSwerveUtilInstance.kDriveInversion[index];
    moduleConstants.DriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    moduleConstants.EncoderId =
        SwerveConstants.kSwerveUtilInstance.kCancoderIds[index].getDeviceNumber();
    moduleConstants.EncoderOffset = SwerveConstants.kSwerveUtilInstance.kModuleOffsets[index];
    moduleConstants.FeedbackSource = SteerFeedbackType.FusedCANcoder;
    moduleConstants.LocationX = Constants.config.moduleLocations[index].getX();
    moduleConstants.LocationY = Constants.config.moduleLocations[index].getY();
    moduleConstants.SlipCurrent = Constants.config.moduleConfig.driveCurrentLimit;
    moduleConstants.SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage;
    moduleConstants.SteerMotorGains.kA = SwerveConstants.kSteerMotorConfiguration.kA;
    moduleConstants.SteerMotorGains.kD = SwerveConstants.kSteerMotorConfiguration.kD;
    moduleConstants.SteerMotorGains.kP = SwerveConstants.kSteerMotorConfiguration.kP;
    moduleConstants.SteerMotorGains.kV = SwerveConstants.kSteerMotorConfiguration.kV;
    moduleConstants.SteerMotorGains.kS = SwerveConstants.kSteerMotorConfiguration.kS;
    moduleConstants.SteerMotorGains.kI = SwerveConstants.kSteerMotorConfiguration.kI;
    moduleConstants.SteerMotorGains.kG = SwerveConstants.kSteerMotorConfiguration.kG;
    moduleConstants.SteerMotorGearRatio =
        SwerveConstants.kSteerMotorConfiguration.gearRatio.getSensorToMechanismRatio();
    moduleConstants.SteerMotorId =
        SwerveConstants.kSwerveUtilInstance.kSteerMotorIds[index].getDeviceNumber();
    moduleConstants.SteerMotorInverted = SwerveConstants.kSwerveUtilInstance.kSteerInversion[index];
    moduleConstants.SteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    moduleConstants.WheelRadius = SwerveConstants.kWheelRadiusMeters;
    moduleConstants.DriveMotorGains.kA = SwerveConstants.kDriveMotorConfiguration.kA;
    moduleConstants.DriveMotorGains.kV = SwerveConstants.kDriveMotorConfiguration.kV;
    moduleConstants.DriveMotorGains.kG = SwerveConstants.kDriveMotorConfiguration.kG;
    moduleConstants.DriveMotorGains.kS = SwerveConstants.kDriveMotorConfiguration.kS;
    moduleConstants.DriveMotorGains.kP = SwerveConstants.kDriveMotorConfiguration.kP;
    moduleConstants.DriveMotorGains.kI = SwerveConstants.kDriveMotorConfiguration.kI;
    moduleConstants.DriveMotorGains.kD = SwerveConstants.kDriveMotorConfiguration.kD;
    moduleConstants.SpeedAt12Volts = Constants.config.moduleConfig.maxDriveVelocityMPS;
    return moduleConstants;
  }

  /**
   *
   *
   * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
   *
   * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
   * purposes. These changes have no effect on real robot operations and address known simulation
   * bugs:
   *
   * <ul>
   *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
   *       configurations.
   *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
   *       issues.
   *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
   *       simulation performance.
   * </ul>
   *
   * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants
   * used on real robot hardware.</h4>
   */
  public static SwerveModuleConstants regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return moduleConstants;

    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CanCoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(60)
                .withKI(0)
                .withKD(6.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withDriveMotorGains(
            new Slot0Configs()
                .withKP(2.5)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(1.524)
                .withKA(25.5)
                .withKS(0.0)
                .withKG(0.0))
        .withSteerMotorGearRatio(
            SwerveConstants.kSteerMotorConfiguration.gearRatio.getSensorToMechanismRatio())
        // Adjust friction voltages
        .withDriveFrictionVoltage(SwerveConstants.kDriveFrictionVoltage)
        .withSteerFrictionVoltage(SwerveConstants.kSteerFrictionVoltage)
        // Adjust steer inertia
        .withSteerInertia(SwerveConstants.kSteerInertia)
        .withDriveInertia(SwerveConstants.kDriveInertia);
  }
}
