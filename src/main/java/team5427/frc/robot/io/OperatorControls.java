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
    // Use command factories instead of inline InstantCommands
    joy.leftTrigger()
        .whileTrue(Superstructure.setIntakeStateCommand(IntakeStates.INTAKING))
        .onFalse(Superstructure.setIntakeStateCommand(IntakeStates.STOWED));

    // Use class-level trigger factory methods instead of nested class references
    Superstructure.intakeStateIs(IntakeStates.INTAKING)
        .and(Superstructure.swerveStateIs(Superstructure.SwerveStates.INTAKE_ASSISTANCE).negate())
        .whileTrue(new IntakeIntaking());

    Superstructure.intakeStateIs(IntakeStates.STOWED).whileTrue(new IntakeStowed());

    Superstructure.intakeStateIs(IntakeStates.DISABLED)
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
