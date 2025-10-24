package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.Superstructure;
import team5427.frc.robot.Superstructure.IntakeStates;
import team5427.frc.robot.commands.intake.IntakeIntaking;
import team5427.frc.robot.commands.intake.IntakeStowed;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;

public class OperatorControls {
  private CommandXboxController joy;

  public OperatorControls() {
    joy = new CommandXboxController(DriverConstants.kOperatorJoystickPort);
    initalizeTriggers();
  }

  public OperatorControls(CommandXboxController joy) {
    this.joy = joy;
    initalizeTriggers();
  }

  /** Made private to prevent multiple calls to this method */
  private void initalizeTriggers() {
    joy.leftTrigger()
        .whileTrue(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedIntakeState = IntakeStates.INTAKING;
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  Superstructure.kSelectedIntakeState = IntakeStates.STOWED;
                }));

    Superstructure.IntakeStates.IntakeTriggers.kIntaking
        .and(Superstructure.SwerveStates.SwerveTriggers.kIntake_Assistance.negate())
        .whileTrue(new IntakeIntaking());

    Superstructure.IntakeStates.IntakeTriggers.kStowed.whileTrue(new IntakeStowed());

    Superstructure.IntakeStates.IntakeTriggers.kDisabled
        .whileTrue(
            new InstantCommand(
                () -> {
                  IntakeSubsystem.getInstance().disablePivotMotor(true);
                  IntakeSubsystem.getInstance().disableRollerMotor(true);
                },
                IntakeSubsystem.getInstance()))
        .onFalse(
            new InstantCommand(
                () -> {
                  IntakeSubsystem.getInstance().disablePivotMotor(false);
                  IntakeSubsystem.getInstance().disableRollerMotor(false);
                }));
  }
}
