package team5427.frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.vision.io.QuestNav;
import team5427.lib.detection.tuples.Tuple2Plus;

public class RobotPose {
  public SwerveDriveOdometry odometry;
  public SwerveDrivePoseEstimator poseEstimator;

  private Pose2d estimatedPose = new Pose2d();
  private Pose2d odometryPose = new Pose2d();

  @Getter private Pose2d questPose = new Pose2d();

  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.008, 0.008, 0.01));

  private static RobotPose instance;

  public static RobotPose getInstance() {
    if (instance == null) {
      instance = new RobotPose();
    }
    return instance;
  }

  public RobotPose() {
    this.odometry =
        new SwerveDriveOdometry(
            SwerveConstants.m_kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero)
            });
    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            SwerveConstants.m_kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero),
              new SwerveModulePosition(0, Rotation2d.kZero)
            },
            Pose2d.kZero,
            odometryStateStdDevs,
            VecBuilder.fill(0.04, 0.04, 9));
  }

  public void addOdometryMeasurement(
      double timestamp, Rotation2d rotation, SwerveModulePosition[] modulePoseModulePositions) {
    odometry.update(rotation, modulePoseModulePositions);
    poseEstimator.updateWithTime(timestamp, rotation, modulePoseModulePositions);
  }

  /**
   * Adds a new timestamped vision measurement. Meant to be a bridge method, actual vision
   * processing should be done in a seperate subsystem
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addQuestMeasurment(Pose2d questPose, double timestamp) {
    this.questPose = questPose;
    poseEstimator.addVisionMeasurement(questPose, timestamp, VecBuilder.fill(0.001, 0.001, 0.01));
  }

  public void addQuestMeasurmentOnlyLog(Pose2d questPose) {
    this.questPose = questPose;
  }

  public Pose2d getOdometryPose() {
    this.odometryPose = odometry.getPoseMeters();
    return this.odometryPose;
  }

  public Pose2d getEstimatedPose() {
    this.estimatedPose = poseEstimator.getEstimatedPosition();
    return this.estimatedPose;
  }

  public void resetAllPose(Pose2d resetPose) {
    // resetPose = resetPose.plus(new Transform2d(0, 0, Rotation2d.k180deg));
    resetOdometryPose(resetPose);
    resetEstimatedPose(resetPose);
    // SwerveSubsystem.getInstance().resetGyro(resetPose.getRotation());
  }

  public void resetAllPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    resetOdometryPose(resetPose, modulePositions, gyroAngle);
    resetEstimatedPose(resetPose, modulePositions, gyroAngle);
    SwerveSubsystem.getInstance().resetGyro(resetPose.getRotation());
  }

  public void resetOdometryPose(Pose2d resetPose) {
    this.odometry.resetPose(resetPose);
  }

  public void resetOdometryPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    this.odometry.resetPosition(gyroAngle, modulePositions, resetPose);
  }

  public void resetEstimatedPose(Pose2d resetPose) {
    this.poseEstimator.resetPose(resetPose);
  }

  public void resetEstimatedPose(
      Pose2d resetPose, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
    this.poseEstimator.resetPosition(gyroAngle, modulePositions, resetPose);
  }

  public Pose2d getAdaptivePose() {
    if (Constants.currentMode.equals(Mode.SIM)) {
      return SwerveSubsystem.getInstance().getKDriveSimulation().getSimulatedDriveTrainPose();
    }
    return getEstimatedPose();
  }

  public void resetHeading(Rotation2d heading) {
    this.odometry.resetRotation(heading);
    this.poseEstimator.resetRotation(heading);
  }

  public Tuple2Plus<Double, Rotation2d> getGyroHeading() {
    return new Tuple2Plus<Double, Rotation2d>(
        Timer.getTimestamp(), SwerveSubsystem.getInstance().getGyroRotationAdjusted());
  }

  public void log() {
    Logger.recordOutput(
        "Localization/Estimation/Robot", RobotPose.getInstance().getEstimatedPose());
    Logger.recordOutput("Localization/Odometry/Robot", RobotPose.getInstance().getOdometryPose());
    Logger.recordOutput("Quest Pose", QuestNav.getInstance().getPose3d());
  }
}
