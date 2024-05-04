package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorControls {

    private final int kPort = 1;
    private CommandXboxController joy;

    public OperatorControls() {
        joy = new CommandXboxController(kPort);
    }
    
}
