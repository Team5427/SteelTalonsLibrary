package team5427.frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeIntaking extends Command {
  private IntakeSubsystem subsystem;

  public IntakeIntaking() {
    subsystem = IntakeSubsystem.getInstance();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.setIntakingRotation(IntakeConstants.kPivotIntakeRotation);
    subsystem.setIntakingSpeed(IntakeConstants.kRollerIntakeVelocity);
    subsystem.simulateIntaking(true);
  }

  @Override
  public boolean isFinished() {
    return false; // change this for the method that you did for Hw that finds out if the game piece
    // is intaked
  }

  @Override
  public void end(boolean interrupted) {}
}
