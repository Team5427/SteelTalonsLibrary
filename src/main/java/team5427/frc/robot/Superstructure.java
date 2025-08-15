package team5427.frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Superstructure {
  public static enum SwerveStates {
    DRIVING,
    AUTO_ALIGN,
    INTAKE_ASSISTANCE,
    AUTON,
    DISABLED;

    public static class SwerveTriggers {
      public static final Trigger kDriving =
          new Trigger(
              () -> {
                return kSelectedSwerveState.equals(DRIVING);
              });
      public static final Trigger kAuto_Align =
          new Trigger(
              () -> {
                return kSelectedSwerveState.equals(AUTO_ALIGN);
              });
      public static final Trigger kIntake_Assistance =
          new Trigger(
              () -> {
                return kSelectedSwerveState.equals(INTAKE_ASSISTANCE);
              });
      public static final Trigger kAuton =
          new Trigger(
              () -> {
                return kSelectedSwerveState.equals(AUTON);
              });
      public static final Trigger kDisabled =
          new Trigger(
              () -> {
                return kSelectedSwerveState.equals(DISABLED);
              });
    }
  }

  public static SwerveStates kSelectedSwerveState = SwerveStates.DISABLED;

  public static enum IntakeStates {
    INTAKING,
    DISABLED,
    STOWED,
    OUTAKING;

    public static class IntakeTriggers {
      public static final Trigger kIntaking =
          new Trigger(
              () -> {
                return kSelectedIntakeState.equals(INTAKING);
              });

      public static final Trigger kDisabled =
          new Trigger(
              () -> {
                return kSelectedIntakeState.equals(DISABLED);
              });

      public static final Trigger kStowed =
          new Trigger(
              () -> {
                return kSelectedIntakeState.equals(STOWED);
              });

      public static final Trigger kOutaking =
          new Trigger(
              () -> {
                return kSelectedIntakeState.equals(OUTAKING);
              });
    }
  }

  public static IntakeStates kSelectedIntakeState = IntakeStates.STOWED;
}
