package team5427.frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.Superstructure;
import team5427.frc.robot.commands.chassis.ChassisMovement;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.vision.io.QuestNav;

public class PilotingControls {
  private CommandXboxController joy;

  public PilotingControls() {
    joy = new CommandXboxController(DriverConstants.kDriverJoystickPort);
    initalizeTriggers();
  }

  public PilotingControls(CommandXboxController joy) {
    this.joy = joy;
    initalizeTriggers();
  }

  /** Made private to prevent multiple calls to this method */
  private void initalizeTriggers() {
       
    Superstructure.SwerveStates.SwerveTriggers.kDriving.onTrue(new ChassisMovement(joy));

    Superstructure.SwerveStates.SwerveTriggers.kAuton.onTrue(new ChassisMovement(joy) ); // Dummy Command, replace with real Auton Driving Command


    // SwerveSubsystem.getInstance().setDefaultCommand(new ChassisMovement(joy));
    joy.a()
        .onTrue(
            new InstantCommand(
                    () ->
                        QuestNav.getInstance()
                            .setPose(new Pose2d(10 * Math.random(), 4, Rotation2d.kZero)))
                .ignoringDisable(true));
    joy.x().onTrue(new InstantCommand(() -> {}));

    // Example Coral Placement Code
    // TODO: delete these code for your own project
    if (Constants.currentMode == Constants.Mode.SIM) {

      joy.y()
          .onTrue(
              new InstantCommand(
                  () -> {
                    Pose2d pose =
                        SwerveSubsystem.getInstance()
                            .getKDriveSimulation()
                            .getSimulatedDriveTrainPose();

                    SwerveSubsystem.getInstance().resetGyro(Rotation2d.kZero);

                    pose =
                        new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            SwerveSubsystem.getInstance().getGyroRotation());
                    RobotPose.getInstance().resetHeading(Rotation2d.kZero);
                    SwerveSubsystem.getInstance()
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
                    SwerveSubsystem.getInstance().resetGyro(Rotation2d.kZero);
                    RobotPose.getInstance()
                        .resetHeading(SwerveSubsystem.getInstance().getGyroRotation());
                  }));
    }
  }
}
