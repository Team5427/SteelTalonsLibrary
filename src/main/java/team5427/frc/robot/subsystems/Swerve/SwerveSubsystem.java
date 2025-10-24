package team5427.frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.generated.TunerConstants;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIO;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOInputsAutoLogged;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOPigeon;
import team5427.frc.robot.subsystems.Swerve.gyro.GyroIOSim;
import team5427.frc.robot.subsystems.Swerve.io.talon.PhoenixOdometryThread;
import team5427.lib.systems.swerve.SteelTalonsDriveSpeeds;
import team5427.lib.systems.swerve.SteelTalonsSwerve;
import team5427.lib.systems.swerve.SwerveUtil;
import team5427.lib.systems.sysid.DrivetrainSysId;

public class SwerveSubsystem extends SubsystemBase
    implements SteelTalonsSwerve, SteelTalonsDriveSpeeds, DrivetrainSysId {

  public static final Lock odometryLock = new ReentrantLock();
  private SwerveSetpointGenerator setpointGenerator;
  @AutoLogOutput private SwerveSetpoint setpoint;
  @AutoLogOutput private ChassisSpeeds currentChassisSpeeds;
  @AutoLogOutput private ChassisSpeeds inputChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private SwerveModule[] swerveModules;
  @Getter private SwerveModuleState[] targetModuleStates;
  @Getter private SwerveModuleState[] actualModuleStates;
  @Getter private SwerveModulePosition[] modulePositions;
  private GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs;
  private DriveFeedforwards driveFeedforwards;
  @Getter private SwerveDriveSimulation kDriveSimulation;

  public static DriveTrainSimulationConfig mapleSimConfig;

  private final SysIdRoutine sysId;

  private OdometryConsumer odometryConsumer;

  private final Alert gyroDisconnectAlert =
      new Alert("Disconnected Gyro :( Now using Kinematics", AlertType.kError);

  private static SwerveSubsystem m_instance;

  public static SwerveSubsystem getInstance() {
    return getInstance(null);
  }

  public static SwerveSubsystem getInstance(OdometryConsumer consumer) {
    if (m_instance == null) {
      m_instance = new SwerveSubsystem(consumer);
    }
    return m_instance;
  }

  private SwerveSubsystem(OdometryConsumer consumer) {
    swerveModules = new SwerveModule[SwerveUtil.kDefaultNumModules];

    mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilogram.of(Constants.config.massKG))
            .withCustomModuleTranslations(Constants.config.moduleLocations)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    SwerveConstants.kDriveFrictionVoltage,
                    SwerveConstants.kSteerFrictionVoltage,
                    Meters.of(SwerveConstants.kWheelRadiusMeters),
                    SwerveConstants.kSteerInertia,
                    Constants.config.moduleConfig.wheelCOF))
            .withBumperSize(SwerveConstants.kBumperXSize, SwerveConstants.kBumperYSize);
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);
    switch (Constants.currentMode) {
      case REAL:
        swerveModules[SwerveUtil.kFrontLeftModuleIdx] =
            new SwerveModule(SwerveUtil.kFrontLeftModuleIdx);
        swerveModules[SwerveUtil.kFrontRightModuleIdx] =
            new SwerveModule(SwerveUtil.kFrontRightModuleIdx);
        swerveModules[SwerveUtil.kRearLeftModuleIdx] =
            new SwerveModule(SwerveUtil.kRearLeftModuleIdx);
        swerveModules[SwerveUtil.kRearRightModuleIdx] =
            new SwerveModule(SwerveUtil.kRearRightModuleIdx);
        gyro = new GyroIOPigeon();

        break;
      case REPLAY:
        swerveModules[SwerveUtil.kFrontLeftModuleIdx] =
            new SwerveModule(SwerveUtil.kFrontLeftModuleIdx);
        swerveModules[SwerveUtil.kFrontRightModuleIdx] =
            new SwerveModule(SwerveUtil.kFrontRightModuleIdx);
        swerveModules[SwerveUtil.kRearLeftModuleIdx] =
            new SwerveModule(SwerveUtil.kRearLeftModuleIdx);
        swerveModules[SwerveUtil.kRearRightModuleIdx] =
            new SwerveModule(SwerveUtil.kRearRightModuleIdx);
        gyro = new GyroIOPigeon();
        break;
      case SIM:
        kDriveSimulation =
            new SwerveDriveSimulation(mapleSimConfig, new Pose2d(3, 3, Rotation2d.k180deg));
        swerveModules[SwerveUtil.kFrontLeftModuleIdx] =
            new SwerveModule(
                SwerveUtil.kFrontLeftModuleIdx,
                kDriveSimulation.getModules()[SwerveUtil.kFrontLeftModuleIdx]);
        swerveModules[SwerveUtil.kFrontRightModuleIdx] =
            new SwerveModule(
                SwerveUtil.kFrontRightModuleIdx,
                kDriveSimulation.getModules()[SwerveUtil.kFrontRightModuleIdx]);
        swerveModules[SwerveUtil.kRearLeftModuleIdx] =
            new SwerveModule(
                SwerveUtil.kRearLeftModuleIdx,
                kDriveSimulation.getModules()[SwerveUtil.kRearLeftModuleIdx]);
        swerveModules[SwerveUtil.kRearRightModuleIdx] =
            new SwerveModule(
                SwerveUtil.kRearRightModuleIdx,
                kDriveSimulation.getModules()[SwerveUtil.kRearRightModuleIdx]);
        gyro = new GyroIOSim(kDriveSimulation.getGyroSimulation());
        break;
      default:
        break;
    }
    targetModuleStates = new SwerveModuleState[SwerveUtil.kDefaultNumModules];
    for (int i = 0; i < SwerveUtil.kDefaultNumModules; i++)
      targetModuleStates[i] = new SwerveModuleState(0.0, Rotation2d.kZero);
    actualModuleStates = new SwerveModuleState[targetModuleStates.length];

    modulePositions = new SwerveModulePosition[SwerveUtil.kDefaultNumModules];
    for (int i = 0; i < SwerveUtil.kDefaultNumModules; i++)
      modulePositions[i] = new SwerveModulePosition();

    driveFeedforwards = DriveFeedforwards.zeros(Constants.config.numModules);
    setpointGenerator =
        new SwerveSetpointGenerator(Constants.config, SwerveConstants.kMaxAngularVelocity);
    setpoint =
        new SwerveSetpoint(new ChassisSpeeds(0, 0, 0), targetModuleStates, driveFeedforwards);

    gyroInputs = new GyroIOInputsAutoLogged();
    odometryConsumer = consumer;

    // Switch these based on the io being used
    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
    // SparkOdometryThread.getInstance().start();

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysId/SwerveSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {

    // Creates a new setpoint based on the previous setpoint and input chassis speeds
    setpoint =
        setpointGenerator.generateSetpoint(setpoint, inputChassisSpeeds, Constants.kLoopSpeed);
    targetModuleStates = setpoint.moduleStates();

    // Lock Odometry (To prevent loss of data and inaccurate data updates)
    odometryLock.lock();
    if (gyro != null) {
      gyro.updateInputs(gyroInputs);
      Logger.processInputs("Swerve/Gyro", gyroInputs);
    }

    for (int i = 0; i < swerveModules.length; i++) {

      if (Constants.currentMode != Mode.SIM) {
        swerveModules[i].setModuleState(targetModuleStates[i], driveFeedforwards);
      } else {
        swerveModules[i].setModuleState(targetModuleStates[i]);
      }

      actualModuleStates[i] = swerveModules[i].getModuleState(); // Read actual module state
      swerveModules[i].periodic(); // Update Module Inputs
    }
    // Unlock Odometry
    odometryLock.unlock();

    // Update Odometry
    double[] odometrySampleTimestamps = swerveModules[0].getOdometryTimestamps();
    int initialTimestampLength = odometrySampleTimestamps.length;
    for (int i = 0; i < initialTimestampLength; i++) {
      SwerveModulePosition[] newModulePositions = new SwerveModulePosition[modulePositions.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modulePositions.length];
      for (int midx = 0; midx < modulePositions.length; midx++) {
        newModulePositions[midx] = swerveModules[midx].getOdometryPositions()[i];
        moduleDeltas[midx] =
            new SwerveModulePosition(
                newModulePositions[midx].distanceMeters - modulePositions[midx].distanceMeters,
                newModulePositions[midx].angle);
        modulePositions[midx] = newModulePositions[midx];
      }

      Rotation2d newGyroInput = Rotation2d.kZero;
      if (gyroInputs.connected && gyro != null) {
        newGyroInput = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d gyroTwist = SwerveConstants.m_kinematics.toTwist2d(moduleDeltas);
        newGyroInput = newGyroInput.plus(Rotation2d.fromRadians(gyroTwist.dtheta));
      }

      odometryConsumer.accept(odometrySampleTimestamps[i], newGyroInput, newModulePositions);
    }

    gyroDisconnectAlert.set(!gyroInputs.connected);

    Logger.recordOutput("SwerveOutput/InputSpeeds", inputChassisSpeeds);
    Logger.recordOutput("SwerveOutput/ModulePositions", getModulePositions());
    Logger.recordOutput("SwerveOutput/ModuleStates", actualModuleStates);
    Logger.recordOutput("SwerveOutput/TargetModuleStates", targetModuleStates);
  }

  @Override
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition().getRadians();
    }
    return values;
  }

  @Override
  public void runDriveCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runDriveCharacterization'");
  }

  @Override
  public void runTurnCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runTurnCharacterization'");
  }

  @Override
  public void runDrivetrainAngularCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'runDrivetrainAngularCharacterization'");
  }

  @Override
  public double scaleDriveComponents(double velocity) {
    return velocity * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
  }

  @Override
  public double scaleDriveComponents(double velocity, double dampeningAmount) {
    return velocity * (1 - dampeningAmount) * SwerveConstants.kDriveMotorConfiguration.maxVelocity;
  }

  public ChassisSpeeds getDriveSpeedsOnlyScaled(
      double xInput, double yInput, double omegaInput, double dampenAmount) {
    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            scaleDriveComponents(xInput, dampenAmount),
            scaleDriveComponents(yInput, dampenAmount),
            scaleDriveComponents(omegaInput, dampenAmount) * Math.PI);
    return rawSpeeds;
  }

  public ChassisSpeeds getDriveSpeedsOnlyScaled(ChassisSpeeds speeds, double dampenAmount) {
    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            scaleDriveComponents(speeds.vxMetersPerSecond, dampenAmount),
            scaleDriveComponents(speeds.vyMetersPerSecond, dampenAmount),
            scaleDriveComponents(speeds.omegaRadiansPerSecond, dampenAmount) * Math.PI);
    return rawSpeeds;
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, double omegaInput, double dampenAmount) {

    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            scaleDriveComponents(xInput, dampenAmount),
            scaleDriveComponents(yInput, dampenAmount),
            scaleDriveComponents(omegaInput, dampenAmount) * Math.PI);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, getGyroRotation().unaryMinus());

    return fieldRelativeSpeeds;
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput,
      double yInput,
      double omegaInput,
      double dampenAmount,
      Rotation2d fieldOrientedRotation) {

    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            scaleDriveComponents(xInput, dampenAmount),
            scaleDriveComponents(yInput, dampenAmount),
            scaleDriveComponents(omegaInput, dampenAmount) * Math.PI);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, fieldOrientedRotation);

    return fieldRelativeSpeeds;
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, Rotation2d targetOmega, double dampenAmount) {

    xInput *= (1 - dampenAmount);
    yInput *= (1 - dampenAmount);

    double calculatedOmega =
        DrivingConstants.kRotationController.calculate(
            RobotPose.getInstance().getAdaptivePose().getRotation().getRadians(),
            targetOmega.getRadians());

    ChassisSpeeds rawSpeeds =
        new ChassisSpeeds(
            xInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            yInput * SwerveConstants.kDriveMotorConfiguration.maxVelocity,
            calculatedOmega);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(rawSpeeds, getGyroRotation().unaryMinus());

    return fieldRelativeSpeeds;
  }

  @Override
  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    this.inputChassisSpeeds = robotRelativeSpeeds;
    this.driveFeedforwards = DriveFeedforwards.zeros(Constants.config.numModules);
  }

  @Override
  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
    this.inputChassisSpeeds = robotRelativeSpeeds;
    System.out.println(driveFeedforwards.robotRelativeForcesXNewtons());
    this.driveFeedforwards = feedforwards;
  }

  @Override
  public Rotation2d getGyroRotation() {

    return gyroInputs.yawPosition;
  }

  public Rotation2d getGyroRotationAdjusted() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return gyroInputs.yawPosition.plus(Rotation2d.k180deg);
    }
    return gyroInputs.yawPosition;
  }

  @Override
  public void resetGyro(Rotation2d newYaw) {
    gyro.resetGyroYawAngle(newYaw);
  }

  @Override
  public ChassisSpeeds getCurrentChassisSpeeds() {
    return Constants.config.toChassisSpeeds(actualModuleStates);
  }

  @Override
  public SwerveModuleState[] getCurrentSwerveModuleStates() {
    return actualModuleStates;
  }

  @Override
  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    return modulePositions;
  }
}
