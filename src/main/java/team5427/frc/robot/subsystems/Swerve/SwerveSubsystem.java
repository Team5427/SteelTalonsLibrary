package team5427.frc.robot.subsystems.Swerve;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5427.lib.systems.swerve.SteelTalonsDriveSpeeds;
import team5427.lib.systems.swerve.SteelTalonsSwerve;
import team5427.lib.systems.sysid.DrivetrainSysId;

public class SwerveSubsystem extends SubsystemBase
    implements SteelTalonsSwerve, SteelTalonsDriveSpeeds, DrivetrainSysId {

  @Override
  public double[] getWheelRadiusCharacterizationPositions() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'getWheelRadiusCharacterizationPositions'");
  }

  @Override
  public void runDriveCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runDriveCharacterization'");
  }

  @Override
  public void runTurnCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runTurnCharacterization'");
  }

  @Override
  public void runDrivetrainAngularCharacterization(double output) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'runDrivetrainAngularCharacterization'");
  }

  @Override
  public double scaleDriveComponents(double velocity) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'scaleDriveComponents'");
  }

  @Override
  public double scaleDriveComponents(double velocity, double dampeningAmount) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'scaleDriveComponents'");
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, double omegaInput, double dampenAmount) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveSpeeds'");
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput,
      double yInput,
      double omegaInput,
      double dampenAmount,
      Rotation2d fieldOrientedRotation) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveSpeeds'");
  }

  @Override
  public ChassisSpeeds getDriveSpeeds(
      double xInput, double yInput, Rotation2d targetOmega, double dampenAmount) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getDriveSpeeds'");
  }

  @Override
  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInputSpeeds'");
  }

  @Override
  public void setInputSpeeds(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInputSpeeds'");
  }

  @Override
  public void resetGyro(Rotation2d gyroYawReset) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetGyro'");
  }

  @Override
  public Rotation2d getGyroRotation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getGyroRotation'");
  }

  @Override
  public ChassisSpeeds getCurrentChassisSpeeds() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentChassisSpeeds'");
  }

  @Override
  public SwerveModuleState[] getCurrentSwerveModuleStates() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentSwerveModuleStates'");
  }

  @Override
  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'getCurrentSwerveModulePositions'");
  }
}
