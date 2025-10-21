package team5427.frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class DriverProfiles {
  public static enum DriverModeType {
    SINGLE,
    DUAL
  }

  public static enum DriverState {
    ERIC(DriverModeType.SINGLE),
    TEST_SINGLE(DriverModeType.SINGLE),
    A_E(DriverModeType.DUAL),
    S_S(DriverModeType.DUAL),
    K_A(DriverModeType.DUAL),
    TEST_DUAL(DriverModeType.DUAL);

    public final DriverModeType modeType;

    DriverState(DriverModeType modeType) {
      this.modeType = modeType;
    }
  }

  public static DriverState kSelectedDriverState = DriverState.A_E;

  public static class DriverTriggers {
    public static final Trigger kIsDualMode =
        new Trigger(
            () -> {
              return kSelectedDriverState.modeType == DriverModeType.DUAL;
            });
    public static final Trigger kIsSingleMode =
        new Trigger(
            () -> {
              return kSelectedDriverState.modeType == DriverModeType.SINGLE;
            });
    public static final Trigger kSingleEric =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.ERIC);
            });
    public static final Trigger kSingleTestSingle =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.TEST_SINGLE);
            });
    public static final Trigger kDualAE =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.A_E);
            });
    public static final Trigger kDualSS =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.S_S);
            });
    public static final Trigger kDualKA =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.K_A);
            });
    public static final Trigger kDualTestDual =
        new Trigger(
            () -> {
              return kSelectedDriverState.equals(DriverState.TEST_DUAL);
            });
  }
}
