package team5427.frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;
import org.team4206.battleaid.common.TunedJoystick.ResponseCurve;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.subsystems.Swerve.DrivingConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class MoveChassisToPose extends Command {

  private SwerveSubsystem swerveSubsystem;
  private CommandXboxController joy;

  private TunedJoystick translationJoystick;
  private TunedJoystick rotationJoystick;

  private Pose2d targetPose;

  public MoveChassisToPose(CommandXboxController driverJoystick, Pose2d targetPose) {
    swerveSubsystem = SwerveSubsystem.getInstance();
    joy = driverJoystick;
    translationJoystick = new TunedJoystick(joy.getHID());
    translationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    rotationJoystick = new TunedJoystick(joy.getHID());
    rotationJoystick.useResponseCurve(ResponseCurve.LINEAR);

    translationJoystick.setDeadzone(DriverConstants.kDriverControllerJoystickDeadzone);
    rotationJoystick.setDeadzone(
        DriverConstants.kDriverControllerRotationalControlJoystickDeadzone);
    addRequirements(swerveSubsystem);
    this.targetPose = targetPose;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double dampener = (joy.getRightTriggerAxis() * SwerveConstants.kDampenerDampeningAmount);

    Logger.recordOutput("Target Pose", targetPose);

    Pose2d robotPose = RobotPose.getInstance().getAdaptivePose();

    Translation2d error = targetPose.getTranslation().minus(robotPose.getTranslation());
    double distance = error.getNorm();

    double velocity = DrivingConstants.kTranslationalController.calculate(distance, 0);

    double vx = 0.0;
    double vy = 0.0;
    if (distance > 1e-4) {
      vx = -velocity * (error.getX() / distance);
      vy = -velocity * (error.getY() / distance);
    }

    double omegaRadiansPerSecond =
        DrivingConstants.kRotationController.calculate(
            robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds driverSpeeds =
        swerveSubsystem.getDriveSpeeds(
            vx,
            vy,
            omegaRadiansPerSecond,
            dampener,
            swerveSubsystem.getGyroRotationAdjusted().unaryMinus());

    if (joy.getLeftTriggerAxis() >= 0.1) {
      driverSpeeds = new ChassisSpeeds(0, 0, 0);
    }
    swerveSubsystem.setInputSpeeds(driverSpeeds);
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
