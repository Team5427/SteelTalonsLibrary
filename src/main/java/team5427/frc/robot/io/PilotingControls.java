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
import team5427.frc.robot.io.DriverProfiles.DriverState;
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

    disabledTrigger = new Trigger(DriverStation::isDisabled);
    autonTrigger = new Trigger(DriverStation::isAutonomous);

    // Swerve State Control Bindings

    // Toggle controlled driving with left bumper
    DriverProfiles.DriverTriggers.kIsState(DriverState.A_E)
        .and(joy.leftBumper())
        .toggleOnTrue(Superstructure.setSwerveStateCommand(SwerveStates.CONTROLLED_DRIVING))
        .toggleOnFalse(Superstructure.setSwerveStateCommand(SwerveStates.RAW_DRIVING));

    // Toggle auto align with right bumper
    DriverProfiles.DriverTriggers.kIsState(DriverState.TEST_DUAL)
        .and(joy.rightBumper())
        .toggleOnTrue(Superstructure.setSwerveStateCommand(SwerveStates.AUTO_ALIGN))
        .toggleOnFalse(Superstructure.setSwerveStateCommand(SwerveStates.CONTROLLED_DRIVING));

    // Auto mode state management
    autonTrigger
        .onTrue(Superstructure.setSwerveStateCommand(SwerveStates.AUTON))
        .onFalse(Superstructure.setSwerveStateCommand(SwerveStates.CONTROLLED_DRIVING));

    // Disabled mode state management
    disabledTrigger.onTrue(Superstructure.setSwerveStateCommand(SwerveStates.DISABLED));

    disabledTrigger
        .negate()
        .and(autonTrigger.negate())
        .onTrue(Superstructure.setSwerveStateCommand(SwerveStates.RAW_DRIVING));

    // State Based Command Bindings

    // Raw driving mode
    Superstructure.swerveStateIs(SwerveStates.RAW_DRIVING)
        .and(autonTrigger.negate())
        .and(disabledTrigger.negate())
        .whileTrue(new RawChassisMovement(joy));

    // Controlled driving mode
    Superstructure.swerveStateIs(SwerveStates.CONTROLLED_DRIVING)
        .and(autonTrigger.negate())
        .and(disabledTrigger.negate())
        .whileTrue(new ControlledChassisMovement(joy));

    // Auto align mode
    Superstructure.swerveStateIs(SwerveStates.AUTO_ALIGN)
        .and(disabledTrigger.negate())
        .whileTrue(new MoveChassisToPose(joy, new Pose2d(5, 5.5, Rotation2d.k180deg)));

    // Utility Bindings

    joy.a()
        .and(Constants.ModeTriggers.kSim)
        .onTrue(
            new InstantCommand(
                    () ->
                        QuestNav.getInstance()
                            .setPose(new Pose2d(10 * Math.random(), 4, Rotation2d.kZero)))
                .ignoringDisable(true));

    DriverProfiles.DriverTriggers.kIsState(DriverState.A_E)
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

    DriverProfiles.DriverTriggers.kIsState(DriverState.TEST_DUAL)
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
