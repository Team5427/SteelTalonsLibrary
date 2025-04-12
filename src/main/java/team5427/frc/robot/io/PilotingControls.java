package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.Constants.DriverConstants;

public class PilotingControls {
  private CommandXboxController joy;

  public PilotingControls() {
    joy = new CommandXboxController(DriverConstants.kDriverJoystickPort);
  }

  public PilotingControls(CommandXboxController joy) {
    this.joy = joy;
  }
}
