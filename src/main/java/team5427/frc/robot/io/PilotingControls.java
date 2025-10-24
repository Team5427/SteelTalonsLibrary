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
import team5427.frc.robot.commands.chassis.MoveChassisToPose;
import team5427.frc.robot.commands.chassis.RawChassisMovement;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.vision.io.QuestNav;

public class PilotingControls {
  private CommandXboxController joy;
  private Trigger autonTrigger;
  private Trigger disabledTrigger;

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
    DriverProfiles.DriverTriggers.kDualAE
        .and(joy.leftBumper())
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
    DriverProfiles.DriverTriggers.kDualAE
        .and(joy.rightBumper())
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedSwerveState = Superstructure.SwerveStates.AUTO_ALIGN;
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


    Superstructure.SwerveStates.SwerveTriggers.kRawDriving
        .and(autonTrigger.negate())
        .and(disabledTrigger.negate())
        .whileTrue(new RawChassisMovement(joy));
    Superstructure.SwerveStates.SwerveTriggers.kRawDriving.whileTrue(new RawChassisMovement(joy));
    Superstructure.SwerveStates.SwerveTriggers.kControlledDriving.whileTrue(
        new ControlledChassisMovement(joy));
    // SwerveSubsystem.getInstance().setDefaultCommand(new RawChassisMovement(joy));
    Superstructure.SwerveStates.SwerveTriggers.kControlledDriving
        .and(autonTrigger.negate())
        .and(disabledTrigger.negate())
        .whileTrue(new ControlledChassisMovement(joy));
    Superstructure.SwerveStates.SwerveTriggers.kAuto_Align
        .and(disabledTrigger.negate())
        .whileTrue(new MoveChassisToPose(joy, new Pose2d(5, 5.5, Rotation2d.k180deg)));
    joy.a()
        .and(Constants.ModeTriggers.kSim)
        .onTrue(
            new InstantCommand(
                    () ->
                        QuestNav.getInstance()
                            .setPose(new Pose2d(10 * Math.random(), 4, Rotation2d.kZero)))
                .ignoringDisable(true));

    DriverProfiles.DriverTriggers.kDualAE
        .and(joy.y())
        .and(Constants.ModeTriggers.kSim)
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
                  SwerveSubsystem.getInstance().getKDriveSimulation().setSimulationWorldPose(pose);
                }));

    DriverProfiles.DriverTriggers.kDualAE
        .and(joy.y())
        .and(Constants.ModeTriggers.kReal)
        .onTrue(
            new InstantCommand(
                () -> {
                  SwerveSubsystem.getInstance().resetGyro(Rotation2d.kZero);
                  RobotPose.getInstance()
                      .resetHeading(SwerveSubsystem.getInstance().getGyroRotation());
                }));
  }
}
