package team5427.frc.robot.subsystems.Swerve;

import edu.wpi.first.math.util.Units;
import team5427.lib.drivers.LoggedTunableNumber;
import team5427.lib.tunableControls.TunableControls.ControlConstants;
import team5427.lib.tunableControls.TunableControls.TunableControlConstants;
import team5427.lib.tunableControls.TunableControls.TunableProfiledController;

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

  public static TunableProfiledController kRotationController =
      new TunableProfiledController(
          new TunableControlConstants(
              "Swerve/Rotation",
              new ControlConstants()
                  .withPID(kRotationKp.get(), 0, kRotationKd.get())
                  .withProfile(kRotationMaxVelocity.get(), kRotationMaxAcceleration.get())
                  .withTolerance(kRotationAngleTolerance.get(), kRotationVelocityTolerance.get())
                  .withContinuous(-Math.PI, Math.PI)));

  //   public static ProfiledPIDController kRotationController =
  //       new ProfiledPIDController(
  //           kRotationKp.get(),
  //           0,
  //           kRotationKd.get(),
  //           new Constraints(kRotationMaxVelocity.get(), kRotationMaxAcceleration.get()));

  public static LoggedTunableNumber kTranslationalKp =
      new LoggedTunableNumber("Translational P", 1.0);
  public static LoggedTunableNumber kTranslationalKd =
      new LoggedTunableNumber("Translational D", 0.1);
  public static LoggedTunableNumber kTranslationalMaxVelocity =
      new LoggedTunableNumber("Translational Max Velocity (m/s)", 3.0);
  public static LoggedTunableNumber kTranslationalMaxAcceleration =
      new LoggedTunableNumber("Translational Max Acceleration (m/s^2)", 3.0);

  public static LoggedTunableNumber kTranslationalPositionTolerance =
      new LoggedTunableNumber("Translational Position Tolerance (m)", 0.01);
  public static LoggedTunableNumber kTranslationalVelocityTolerance =
      new LoggedTunableNumber("Translational Velocity Tolerance (m/s)", 0.05);

  //   public static ProfiledPIDController kTranslationalController =
  //       new ProfiledPIDController(
  //           kTranslationalKp.get(),
  //           0.0,
  //           kTranslationalKd.get(),
  //           new Constraints(kTranslationalMaxVelocity.get(),
  // kTranslationalMaxAcceleration.get()));

  public static TunableProfiledController kTranslationalController =
      new TunableProfiledController(
          new TunableControlConstants(
              "Swerve/Translation",
              new ControlConstants()
                  .withPID(kRotationKp.get(), 0, kRotationKd.get())
                  .withProfile(kTranslationalMaxVelocity.get(), kTranslationalMaxAcceleration.get())
                  .withTolerance(
                      kTranslationalPositionTolerance.get(),
                      kTranslationalVelocityTolerance.get())));

  //   static {
  //     // Bind tunables to controller updates
  //     kTranslationalKp.bindToTrigger((Double number) -> kTranslationalController.setP(number));
  //     kTranslationalKd.bindToTrigger((Double number) -> kTranslationalController.setD(number));

  //     kTranslationalMaxVelocity.bindToTrigger(
  //         (Double number) ->
  //             kTranslationalController.setConstraints(
  //                 new Constraints(number, kTranslationalMaxAcceleration.get())));
  //     kTranslationalMaxAcceleration.bindToTrigger(
  //         (Double number) ->
  //             kTranslationalController.setConstraints(
  //                 new Constraints(kTranslationalMaxVelocity.get(), number)));
  //   }
}
