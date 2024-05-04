package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class PilotingControls {

    private final int kPort = 0;
    private CommandXboxController joy;

    public PilotingControls() {
        joy = new CommandXboxController(kPort);
    }
    
}
