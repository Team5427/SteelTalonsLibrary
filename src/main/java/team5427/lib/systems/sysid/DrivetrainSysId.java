package team5427.lib.systems.sysid;

public interface DrivetrainSysId {
  public double[] getWheelRadiusCharacterizationPositions();

  public void runDriveCharacterization(double output);

  public void runTurnCharacterization(double output);

  public void runDrivetrainAngularCharacterization(double output);
}
