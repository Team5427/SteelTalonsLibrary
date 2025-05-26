package team5427.frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  public static final String kSwerveCamName = "swerveCam";
  public static final String kIntakeCamName = "intakeCam";
  // public static final String kBackCamName = "backCam";

  public static final int kCameraCount = 2;

  public static final double kMaxAmbiguity = 0.20;

  public static final Distance kMaxZHeight = Meters.of(0.6);

  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Left side of bot
  // Robot to camera
  public static final Transform3d kIntakeCamTransform =
      new Transform3d(
          Units.inchesToMeters(0.5), // 9.375
          Units.inchesToMeters(6.048439965), // 11.048439965
          Units.inchesToMeters(8.540489626),
          new Rotation3d(0, Units.degreesToRadians(-30), 0.0)); // 0.47976945625357

  // Right side of bot
  // Robot to camera
  public static final Transform3d kSwerveCamTransform =
      new Transform3d(
          Units.inchesToMeters(0.5), // 9.375
          Units.inchesToMeters(-6.048439965), // -11.048439965
          Units.inchesToMeters(8.540489626),
          new Rotation3d(0, Units.degreesToRadians(-30), 0.0));

  public static final Transform3d kQuestCameraTransform =
      new Transform3d(
          0.192, 0.358, Units.inchesToMeters(8.098), new Rotation3d(Rotation2d.kCCW_90deg));
  // new Transform3d(
  //     Units.inchesToMeters(0.296),
  //     Units.inchesToMeters(12.5),
  //     Units.inchesToMeters(8.098),
  //     new Rotation3d(Rotation2d.kCCW_90deg));

  public static Transform3d[] kCameraTransforms = new Transform3d[kCameraCount];

  static {
    kCameraTransforms[1] = kIntakeCamTransform;
    kCameraTransforms[0] = kSwerveCamTransform;
  }

  public static final Distance kCameraMaxRange = Meters.of(4.0);

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  /** Larger stddev equals more doubt in Meters */
  public static double kLinearStdDevBaseline = Units.inchesToMeters(5);

  /** Larger stddev equals more doubt in Radians */
  public static double kAngularStdDevBaseline = Units.degreesToRadians(30);

  public static double[] kCameraStdDevFactors =
      new double[] {
        1.0, // Swerve Cam
        1.0 // Intake Cam
      };

  public static double kQuestStdDevBaseline = 0.001;
  public static String kQuestLoggingDirectory = "Quest/";
}
