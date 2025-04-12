package team5427.lib.systems.swerve;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SteelTalonsSwerve {
  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds);

  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards);

  public default void setInputSpeedsFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {}

  public void resetGyro(Rotation2d gyroYawReset);

  public Rotation2d getGyroRotation();

  public ChassisSpeeds getCurrentChassisSpeeds();

  public SwerveModuleState[] getCurrentSwerveModuleStates();

  public SwerveModulePosition[] getCurrentSwerveModulePositions();

  @FunctionalInterface
  public static interface OdometryConsumer {
    public void accept(
        double timestampSeconds, Rotation2d rotation, SwerveModulePosition[] modulePositions);
  }
}
