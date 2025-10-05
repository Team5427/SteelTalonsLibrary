package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RawChassisMovement extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;

  private TunedJoystick translationJoystick;
  private TunedJoystick rotationJoystick;

  public RawChassisMovement(CommandXboxController driverJoystick) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    translationJoystick = new TunedJoystick(joy.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    rotationJoystick = new TunedJoystick(joy.getHID());
    rotationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    translationJoystick.setDeadzone(DriverConstants.kDriverControllerJoystickDeadzone);
    rotationJoystick.setDeadzone(DriverConstants.kDriverControllerJoystickDeadzone);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    if (DriverStation.isTeleop()) {
      double vx = -translationJoystick.getRightY();
      double vy = -translationJoystick.getRightX();
      double omegaRadians = -rotationJoystick.getLeftX();

      double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

      ChassisSpeeds driverSpeeds = swerveSubsystem.getDriveSpeeds(vx, vy, omegaRadians, dampener);

      if (joy.getLeftTriggerAxis() >= 0.1) {
        driverSpeeds = new ChassisSpeeds(0, 0, 0);
      }
      swerveSubsystem.setInputSpeeds(driverSpeeds);
    } else {
      swerveSubsystem.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.setInputSpeeds(new ChassisSpeeds());
  }
}
