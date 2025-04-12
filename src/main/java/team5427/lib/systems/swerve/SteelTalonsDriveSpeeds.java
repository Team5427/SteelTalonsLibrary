package team5427.lib.systems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SteelTalonsDriveSpeeds {
  public double scaleDriveComponents(double velocity);

  public double scaleDriveComponents(double velocity, double dampeningAmount);

  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, double omegaInput, double dampenAmount);

  public ChassisSpeeds getDriveSpeeds(
      double xInput,
      double yInput,
      double omegaInput,
      double dampenAmount,
      Rotation2d fieldOrientedRotation);

  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, Rotation2d targetOmega, double dampenAmount);
}
