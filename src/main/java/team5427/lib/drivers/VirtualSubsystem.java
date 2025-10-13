package team5427.lib.drivers;

import java.util.LinkedList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new LinkedList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll(TelemetryVerbosity tv) {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic( tv);
    }
  }

  public abstract void periodic(TelemetryVerbosity tv);
}
