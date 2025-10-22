package team5427.frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.vision.io.VisionIO;
import team5427.frc.robot.subsystems.vision.io.VisionIO.PoseObservation;
import team5427.frc.robot.subsystems.vision.io.VisionIO.PoseObservationType;
import team5427.frc.robot.subsystems.vision.io.VisionIOInputsAutoLogged;
import team5427.frc.robot.subsystems.vision.io.VisionIOPhoton;
import team5427.frc.robot.subsystems.vision.io.VisionIOPhotonSim;
import team5427.lib.detection.tuples.Tuple2Plus;
import team5427.lib.drivers.VirtualSubsystem;

public class VisionSubsystem extends VirtualSubsystem {
  private VisionIO[] io = new VisionIO[VisionConstants.kCameraCount];
  private VisionIOInputsAutoLogged[] inputsAutoLogged =
      new VisionIOInputsAutoLogged[VisionConstants.kCameraCount];
  private Pose3d referencePose;
  private final Alert[] disconnectedAlerts;

  private final VisionConsumer visionConsumer;

  private static VisionSubsystem m_instance;

  @Getter private Pose3d latestPoseMeasurement;

  public static VisionSubsystem getInstance(
      VisionConsumer consumer,
      Supplier<Pose2d> referencePoseSupplier,
      Supplier<Tuple2Plus<Double, Rotation2d>> referenceHeadingSupplier) {
    if (m_instance == null) {
      m_instance = new VisionSubsystem(consumer, referencePoseSupplier, referenceHeadingSupplier);
    }
    return m_instance;
  }

  public static VisionSubsystem getInstance() {
    if (m_instance == null) {

      DriverStation.reportWarning("Vision Subsystem Not provided Vision Consumer", true);
      return null;
    }
    return m_instance;
  }

  private VisionSubsystem(
      VisionConsumer consumer,
      Supplier<Pose2d> referencePoseSupplier,
      Supplier<Tuple2Plus<Double, Rotation2d>> referenceHeadingSupplier) {
    super();
    switch (Constants.currentMode) {
      case REAL:
        io[0] =
            new VisionIOPhoton(
                VisionConstants.kSwerveCamName,
                VisionConstants.kSwerveCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        io[1] =
            new VisionIOPhoton(
                VisionConstants.kIntakeCamName,
                VisionConstants.kIntakeCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        // io[2] = new VisionIOQuestNav(VisionConstants.kQuestCameraTransform);

        for (int i = 0; i < inputsAutoLogged.length; i++) {
          inputsAutoLogged[i] = new VisionIOInputsAutoLogged();
        }
        break;
      case REPLAY:
      case SIM:
        io[0] =
            new VisionIOPhotonSim(
                VisionConstants.kSwerveCamName,
                VisionConstants.kSwerveCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        io[1] =
            new VisionIOPhotonSim(
                VisionConstants.kIntakeCamName,
                VisionConstants.kIntakeCamTransform,
                referencePoseSupplier,
                referenceHeadingSupplier);
        for (int i = 0; i < inputsAutoLogged.length; i++) {
          inputsAutoLogged[i] = new VisionIOInputsAutoLogged();
        }
        break;
      default:
        break;
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputsAutoLogged.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
    this.visionConsumer = consumer;
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return Rotation2d.fromDegrees(inputsAutoLogged[cameraIndex].poseObservations[0].tx());
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputsAutoLogged[i]);
      Logger.processInputs("Vision/Camera " + Integer.toString(i), inputsAutoLogged[i]);
    }

    int n = 0;

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

      // Loop over pose observations
      for (PoseObservation observation : inputsAutoLogged[cameraIndex].poseObservations) {
        n++;
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > VisionConstants.kMaxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.kMaxZHeight.in(Meter) // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConstants.kAprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConstants.kAprilTagLayout.getFieldWidth()
                // Must not be an impossible pose to acheive based on max drivetrain speeds (scaled
                // to allow for some leeway)
                || (Constants.currentMode.equals(Mode.REAL)
                    && observation
                            .pose()
                            .toPose2d()
                            .relativeTo(RobotPose.getInstance().getAdaptivePose())
                            .getTranslation()
                            .getNorm()
                        > SwerveConstants.kDriveMotorConfiguration.maxVelocity
                            * 4.0
                            * (Timer.getTimestamp() - observation.timestamp()));

        // Add pose to log
        Logger.recordOutput("Vision Pose " + cameraIndex, observation.pose());

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            observation.type().equals(PoseObservationType.PHOTONVISION_SINGLE_TAG)
                ? observation.averageTagDistance() * 2.0
                : Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.kLinearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.kAngularStdDevBaseline * stdDevFactor;
        if (cameraIndex < VisionConstants.kCameraStdDevFactors.length) {
          linearStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
          angularStdDev *= VisionConstants.kCameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        visionConsumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        latestPoseMeasurement = observation.pose();
      }
    }

    Logger.recordOutput("Num of Pose Observations", n);
    n = 0;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
