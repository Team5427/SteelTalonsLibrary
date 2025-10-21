package team5427.frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;
import team5427.lib.motors.MotorUtil;
import team5427.lib.systems.swerve.SwerveUtil;

public final class SwerveConstants {
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.98);
  public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
  public static final double kTrackWidth = Units.inchesToMeters(22.75);
  public static final double kWheelBase = Units.inchesToMeters(22.75);
  public static final Distance kBumperXSize = Inches.of(30.0);
  public static final Distance kBumperYSize = Inches.of(30.0);

  public static final AngularVelocity kMaxAngularVelocity = RotationsPerSecond.of(4.0);

  public static final double kCoupleRatio = 3.125;

  public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.05);
  public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
  // Simulated voltage necessary to overcome friction
  public static final Voltage kSteerFrictionVoltage = Volts.of(0.05);
  public static final Voltage kDriveFrictionVoltage = Volts.of(0.1);

  public static MotorConfiguration kDriveMotorConfiguration = new MotorConfiguration();

  static {
    kDriveMotorConfiguration.gearRatio = SwerveUtil.kSDSL3GearRatio;
    kDriveMotorConfiguration.idleState = IdleState.kBrake;
    kDriveMotorConfiguration.mode = MotorMode.kFlywheel;
    kDriveMotorConfiguration.withFOC = true;

    kDriveMotorConfiguration.currentLimit = 80;
    kDriveMotorConfiguration.finalDiameterMeters = kWheelDiameterMeters;

    kDriveMotorConfiguration.maxVelocity =
        kDriveMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
    kDriveMotorConfiguration.maxAcceleration = kDriveMotorConfiguration.maxVelocity * 2.0;

    kDriveMotorConfiguration.kP = 2.64; // 2.64
    // kDriveMotorConfiguration.kV = 0.75;
    kDriveMotorConfiguration.kA = 0.1;
    kDriveMotorConfiguration.kS = 0.5;
    kDriveMotorConfiguration.altV = kDriveMotorConfiguration.maxVelocity;
    kDriveMotorConfiguration.altA = kDriveMotorConfiguration.maxAcceleration;
  }

  public static MotorConfiguration kSteerMotorConfiguration = new MotorConfiguration();

  static {
    kSteerMotorConfiguration.gearRatio = SwerveUtil.kSDSSteerGearRatioMK4n;
    kSteerMotorConfiguration.idleState = IdleState.kBrake;
    kSteerMotorConfiguration.mode = MotorMode.kServo;
    kSteerMotorConfiguration.currentLimit = 40;
    kSteerMotorConfiguration.withFOC = true;

    kSteerMotorConfiguration.maxVelocity =
        kSteerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
    kSteerMotorConfiguration.maxAcceleration = kSteerMotorConfiguration.maxVelocity * 100.0;

    // Tunable values
    kSteerMotorConfiguration.kP = 4.613; // 7.0
    kSteerMotorConfiguration.kD = 0.0004;
    kSteerMotorConfiguration.kS = 0.5;
    kSteerMotorConfiguration.kA = 0.2;
    // kSteerMotorConfiguration.kA = 8.0;
    kSteerMotorConfiguration.altV = kSteerMotorConfiguration.maxVelocity;
    kSteerMotorConfiguration.altA = kSteerMotorConfiguration.maxAcceleration;
  }

  public static final CANDeviceId kPigeonCANId = new CANDeviceId(11, "*");

  public static final double kDrivetrainRadius =
      Math.max(
          Math.max(
              Math.hypot(kWheelBase / 2, kTrackWidth / 2),
              Math.hypot(kWheelBase / 2, -kTrackWidth / 2)),
          Math.max(
              Math.hypot(-kWheelBase / 2, kTrackWidth / 2),
              Math.hypot(-kWheelBase / 2, -kTrackWidth / 2)));

  public static final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final SwerveUtil kSwerveUtilInstance = new SwerveUtil();

  public static class SimulationConstants {
    public static final double steerkP = 30.0;
    public static final double steerkI = 0.0;
    public static final double steerkD = 2.0;

    public static final double drivekP = 20.0;
    public static final double drivekI = 0.0;
    public static final double drivekD = 0.0;
    public static final double drivekS = 0.0;
    public static final double drivekV = 45.0;
  }

  static {
    kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(3, "*");
    kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(5, "*");
    kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(7, "*");
    kSwerveUtilInstance.kDriveMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(9, "*");

    kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(4, "*");
    kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(6, "*");
    kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(8, "*");
    kSwerveUtilInstance.kSteerMotorIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(10, "*");

    kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontLeftModuleIdx] = new CANDeviceId(12, "*");
    kSwerveUtilInstance.kCancoderIds[SwerveUtil.kFrontRightModuleIdx] = new CANDeviceId(13, "*");
    kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearLeftModuleIdx] = new CANDeviceId(14, "*");
    kSwerveUtilInstance.kCancoderIds[SwerveUtil.kRearRightModuleIdx] = new CANDeviceId(15, "*");

    kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontLeftModuleIdx] = true;
    kSwerveUtilInstance.kDriveInversion[SwerveUtil.kFrontRightModuleIdx] = true;
    kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearLeftModuleIdx] = true;
    kSwerveUtilInstance.kDriveInversion[SwerveUtil.kRearRightModuleIdx] = true;

    kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontLeftModuleIdx] = false;
    kSwerveUtilInstance.kSteerInversion[SwerveUtil.kFrontRightModuleIdx] = false;
    kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearLeftModuleIdx] = false;
    kSwerveUtilInstance.kSteerInversion[SwerveUtil.kRearRightModuleIdx] = false;

    kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontLeftModuleIdx] = 0.20752;

    kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kFrontRightModuleIdx] = -0.407;

    kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearRightModuleIdx] = -0.2105;

    kSwerveUtilInstance.kModuleOffsets[SwerveUtil.kRearLeftModuleIdx] = -0.1358;
  }

  public static final double kDampenerDampeningAmount = 0.95;
}
