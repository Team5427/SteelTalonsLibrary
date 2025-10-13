package team5427.frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.Superstructure;
import team5427.frc.robot.Superstructure.SwerveStates;
import team5427.frc.robot.commands.chassis.ControlledChassisMovement;
import team5427.frc.robot.commands.chassis.RawChassisMovement;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.vision.io.QuestNav;
import team5427.lib.drivers.TelemetryVerbosity;

public class PilotingControls {
  private CommandXboxController joy;
  private Trigger autonTrigger;
  private Trigger disabledTrigger;

  public PilotingControls(TelemetryVerbosity tv) {
    joy = new CommandXboxController(DriverConstants.kDriverJoystickPort);
    initalizeTriggers(tv);
  }

  public PilotingControls(CommandXboxController joy, TelemetryVerbosity tv) {
    this.joy = joy;
    initalizeTriggers(tv);
  }

  /** Made private to prevent multiple calls to this method */
  private void initalizeTriggers(TelemetryVerbosity tv) {

    disabledTrigger =
        new Trigger(
            () -> {
              return DriverStation.isDisabled();
            });

    autonTrigger =
        new Trigger(
            () -> {
              return DriverStation.isAutonomous();
            });
    joy.leftBumper()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState =
                      Superstructure.SwerveStates.CONTROLLED_DRIVING;
                }))
        .toggleOnFalse(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = Superstructure.SwerveStates.RAW_DRIVING;
                }));

    autonTrigger
        .onTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = SwerveStates.AUTON;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = SwerveStates.RAW_DRIVING;
                }));

    disabledTrigger
        .onTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = SwerveStates.DISABLED;
                }))
        .negate()
        .and(autonTrigger.negate())
        .onTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = SwerveStates.RAW_DRIVING;
                }));

    Superstructure.SwerveStates.SwerveTriggers.kRawDriving.whileTrue(new RawChassisMovement(joy, tv));
    Superstructure.SwerveStates.SwerveTriggers.kControlledDriving.whileTrue(
        new ControlledChassisMovement(joy, tv));
    // SwerveSubsystem.getInstance().setDefaultCommand(new RawChassisMovement(joy));
    joy.a()
        .onTrue(
            new InstantCommand(
                    () ->
                        QuestNav.getInstance()
                            .setPose(new Pose2d(10 * Math.random(), 4, Rotation2d.kZero)))
                .ignoringDisable(true));
    if (Constants.currentMode == Constants.Mode.SIM) {

      joy.y()
          .onTrue(
              new InstantCommand(
                  () -> {
                    Pose2d pose =
                        SwerveSubsystem.getInstance(tv)
                            .getKDriveSimulation()
                            .getSimulatedDriveTrainPose();

                    SwerveSubsystem.getInstance(tv).resetGyro(Rotation2d.kZero);

                    pose =
                        new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            SwerveSubsystem.getInstance(tv).getGyroRotation());
                    RobotPose.getInstance().resetHeading(Rotation2d.kZero);
                    SwerveSubsystem.getInstance(tv)
                        .getKDriveSimulation()
                        .setSimulationWorldPose(pose);
                  }));

      // L4 placement
      //   joy.y()
      //       .onTrue(
      //           Commands.runOnce(
      //               () ->
      //                   SimulatedArena.getInstance()
      //                       .addGamePieceProjectile(
      //                           new ReefscapeCoralOnFly(
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getSimulatedDriveTrainPose()
      //                                   .getTranslation(),
      //                               new Translation2d(0.4, 0),
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getSimulatedDriveTrainPose()
      //                                   .getRotation(),
      //                               Meters.of(2),
      //                               MetersPerSecond.of(1.5),
      //                               Degrees.of(-80)))));
      //   // L3 placement
      //   joy.b()
      //       .onTrue(
      //           Commands.runOnce(
      //               () ->
      //                   SimulatedArena.getInstance()
      //                       .addGamePieceProjectile(
      //                           new ReefscapeCoralOnFly(
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getSimulatedDriveTrainPose()
      //                                   .getTranslation(),
      //                               new Translation2d(0.4, 0),
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
      //                               SwerveSubsystem.getInstance()
      //                                   .getKDriveSimulation()
      //                                   .getSimulatedDriveTrainPose()
      //                                   .getRotation(),
      //                               Meters.of(1.35),
      //                               MetersPerSecond.of(1.5),
      //                               Degrees.of(-60)))));
    } else {
      joy.y()
          .onTrue(
              new InstantCommand(
                  () -> {
                    SwerveSubsystem.getInstance(tv).resetGyro(Rotation2d.kZero);
                    RobotPose.getInstance()
                        .resetHeading(SwerveSubsystem.getInstance(tv).getGyroRotation());
                  }));
    }
  }
}
