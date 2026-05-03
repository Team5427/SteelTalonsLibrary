package team5427.lib.systems.sysid;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface DrivetrainSysId {
  public double[] getWheelRadiusCharacterizationPositions();

  public void runDriveCharacterizationDynamic(SysIdRoutine.Direction direction);

  public void runTurnCharacterizationDynamic(SysIdRoutine.Direction direction);

  public void runDrivetrainAngularCharacterizationDynamic(SysIdRoutine.Direction direction);

  public void runDriveCharacterizationQuasistatic(SysIdRoutine.Direction direction);

  public void runTurnCharacterizationQuasistatic(SysIdRoutine.Direction direction);

  public void runDrivetrainAngularCharacterizationQuasistatic(SysIdRoutine.Direction direction);
}
