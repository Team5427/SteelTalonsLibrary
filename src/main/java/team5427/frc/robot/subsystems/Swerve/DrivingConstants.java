package team5427.frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import team5427.lib.drivers.LoggedTunableNumber;

public final class DrivingConstants {
  public static LoggedTunableNumber kRotationKp = new LoggedTunableNumber("Rotation P", 1.0);
  public static LoggedTunableNumber kRotationKd = new LoggedTunableNumber("Rotation D", 0.1);

  public static LoggedTunableNumber kRotationMaxAcceleration =
      new LoggedTunableNumber("Rotation Max Acc.", 2 * Math.PI);
  public static LoggedTunableNumber kRotationMaxVelocity =
      new LoggedTunableNumber("Rotation Max Vel.", 2 * Math.PI);

  public static LoggedTunableNumber kRotationAngleTolerance =
      new LoggedTunableNumber("Rotation Angle Tol.", Units.degreesToRadians(2));
  public static LoggedTunableNumber kRotationVelocityTolerance =
      new LoggedTunableNumber("Rotation Velocity Tol.", Units.degreesToRadians(2));

  public static ProfiledPIDController kRotationController =
      new ProfiledPIDController(
          kRotationKp.get(),
          0,
          kRotationKd.get(),
          new Constraints(kRotationMaxVelocity.get(), kRotationMaxAcceleration.get()));

  static {
    kRotationKp.bindToTrigger(
        (Double number) -> {
          kRotationController.setP(number);
        });

    kRotationController.enableContinuousInput(-Math.PI, Math.PI);
    kRotationController.setTolerance(
        kRotationAngleTolerance.get(), kRotationVelocityTolerance.get());
  }

  public static LoggedTunableNumber kTranslationalKp =
      new LoggedTunableNumber("Translational P", 1.0);
  public static LoggedTunableNumber kTranslationalKd =
      new LoggedTunableNumber("Translational D", 0.1);

  // public static LoggedTunableNumber kTranslationalController = ProfiledPIDController

}
