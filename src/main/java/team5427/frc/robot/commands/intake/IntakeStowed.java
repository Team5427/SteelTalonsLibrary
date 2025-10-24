package team5427.frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeStowed extends Command {
  private IntakeSubsystem subsystem;

  public IntakeStowed() {
    subsystem = IntakeSubsystem.getInstance();
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.setIntakingRotation(IntakeConstants.kPivotStartingRotation);
    subsystem.setIntakingSpeed(IntakeConstants.kRollerStowedVelocity);
    subsystem.simulateIntaking(false);
  }

  @Override
  public boolean isFinished() {
    return false; // change this for the method that you did for Hw that finds out if the game piece
    // is intaked
  }

  @Override
  public void end(boolean interrupted) {}
}
