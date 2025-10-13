package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.Superstructure;
import team5427.frc.robot.Superstructure.IntakeStates;
import team5427.frc.robot.commands.intake.IntakeIntaking;
import team5427.frc.robot.commands.intake.IntakeStowed;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;
import team5427.lib.drivers.TelemetryVerbosity;

public class OperatorControls {
  private CommandXboxController joy;

  public OperatorControls(TelemetryVerbosity tv) {
    joy = new CommandXboxController(DriverConstants.kOperatorJoystickPort);
    initalizeTriggers(tv);
  }

  public OperatorControls(CommandXboxController joy, TelemetryVerbosity tv) {
    this.joy = joy;
    initalizeTriggers(tv);
  }

  /** Made private to prevent multiple calls to this method */
  private void initalizeTriggers(TelemetryVerbosity tv) {
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
        .whileTrue(new IntakeIntaking(tv));

    Superstructure.IntakeStates.IntakeTriggers.kStowed.whileTrue(new IntakeStowed(tv));

    Superstructure.IntakeStates.IntakeTriggers.kDisabled
        .whileTrue(
            new InstantCommand(
                () -> {
                  IntakeSubsystem.getInstance(tv).disablePivotMotor(true);
                  IntakeSubsystem.getInstance(tv).disableRollerMotor(true);
                },
                IntakeSubsystem.getInstance(tv)))
        .onFalse(
            new InstantCommand(
                () -> {
                  IntakeSubsystem.getInstance(tv).disablePivotMotor(false);
                  IntakeSubsystem.getInstance(tv).disableRollerMotor(false);
                }));
  }
}
