package team5427.frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public final class Superstructure {
  public static final String dashboardKey = "/Superstructure";

  // ═══════════════════════════════════════════════════════════════════════════
  // STATE VARIABLES (private - only accessible via getters/setters)
  // ═══════════════════════════════════════════════════════════════════════════
  private static SwerveStates kSelectedSwerveState = SwerveStates.DISABLED;
  private static SwerveStates kPreviousSwerveState = SwerveStates.DISABLED;
  
  private static IntakeStates kSelectedIntakeState = IntakeStates.STOWED;
  private static IntakeStates kPreviousIntakeState = IntakeStates.STOWED;

  // ═══════════════════════════════════════════════════════════════════════════
  // SWERVE STATES
  // ═══════════════════════════════════════════════════════════════════════════
  public static enum SwerveStates {
    RAW_DRIVING,
    CONTROLLED_DRIVING,
    AUTO_ALIGN,
    INTAKE_ASSISTANCE,
    AUTON,
    DISABLED
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // INTAKE STATES
  // ═══════════════════════════════════════════════════════════════════════════
  public static enum IntakeStates {
    INTAKING,
    DISABLED,
    STOWED,
    OUTAKING
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // GETTERS
  // ═══════════════════════════════════════════════════════════════════════════
  public static SwerveStates getSelectedSwerveState() {
    return kSelectedSwerveState;
  }

  public static SwerveStates getPreviousSwerveState() {
    return kPreviousSwerveState;
  }

  public static IntakeStates getSelectedIntakeState() {
    return kSelectedIntakeState;
  }

  public static IntakeStates getPreviousIntakeState() {
    return kPreviousIntakeState;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // STATE REQUEST METHODS
  // ═══════════════════════════════════════════════════════════════════════════
  public static void requestSwerveState(SwerveStates newState) {
    if (kSelectedSwerveState != newState) {
      kPreviousSwerveState = kSelectedSwerveState;
      kSelectedSwerveState = newState;
      Logger.recordOutput(dashboardKey + "/SwerveState", newState.toString());
      Logger.recordOutput(dashboardKey + "/PreviousSwerveState", kPreviousSwerveState.toString());
    }
  }

  public static void requestIntakeState(IntakeStates newState) {
    if (kSelectedIntakeState != newState) {
      kPreviousIntakeState = kSelectedIntakeState;
      kSelectedIntakeState = newState;
      Logger.recordOutput(dashboardKey + "/IntakeState", newState.toString());
      Logger.recordOutput(dashboardKey + "/PreviousIntakeState", kPreviousIntakeState.toString());
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // COMMAND FACTORIES - Returns commands that change state
  // ═══════════════════════════════════════════════════════════════════════════
  public static Command setSwerveStateCommand(SwerveStates state) {
    return Commands.runOnce(() -> requestSwerveState(state))
        .withName("SetSwerveState(" + state.toString() + ")");
  }

  public static Command setIntakeStateCommand(IntakeStates state) {
    return Commands.runOnce(() -> requestIntakeState(state))
        .withName("SetIntakeState(" + state.toString() + ")");
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // TRIGGER FACTORY - Creates triggers for any state condition
  // ═══════════════════════════════════════════════════════════════════════════
  public static Trigger swerveStateIs(SwerveStates state) {
    return new Trigger(() -> kSelectedSwerveState == state);
  }

  public static Trigger swerveStateIsAnyOf(SwerveStates... states) {
    return new Trigger(() -> {
      for (SwerveStates state : states) {
        if (kSelectedSwerveState == state) return true;
      }
      return false;
    });
  }

  public static Trigger intakeStateIs(IntakeStates state) {
    return new Trigger(() -> kSelectedIntakeState == state);
  }

  public static Trigger intakeStateIsAnyOf(IntakeStates... states) {
    return new Trigger(() -> {
      for (IntakeStates state : states) {
        if (kSelectedIntakeState == state) return true;
      }
      return false;
    });
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // TRANSITION TRIGGERS - Detect state changes
  // ═══════════════════════════════════════════════════════════════════════════
  public static Trigger swerveStateChangedTo(SwerveStates state) {
    return new Trigger(() -> 
        kSelectedSwerveState == state && kPreviousSwerveState != state);
  }

  public static Trigger swerveStateChangedFrom(SwerveStates state) {
    return new Trigger(() -> 
        kPreviousSwerveState == state && kSelectedSwerveState != state);
  }

  public static Trigger intakeStateChangedTo(IntakeStates state) {
    return new Trigger(() -> 
        kSelectedIntakeState == state && kPreviousIntakeState != state);
  }

  public static Trigger intakeStateChangedFrom(IntakeStates state) {
    return new Trigger(() -> 
        kPreviousIntakeState == state && kSelectedIntakeState != state);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // COMPOUND TRIGGERS - Combine multiple conditions
  // ═══════════════════════════════════════════════════════════════════════════
  public static Trigger when(BooleanSupplier condition) {
    return new Trigger(condition);
  }

  public static Trigger swerveAndIntakeStatesAre(SwerveStates swerve, IntakeStates intake) {
    return swerveStateIs(swerve).and(intakeStateIs(intake));
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // VALIDATION - Prevent invalid state combinations
  // ═══════════════════════════════════════════════════════════════════════════
  public static boolean canTransitionSwerve(SwerveStates from, SwerveStates to) {
    // Add validation rules here
    // Example: Cannot go to AUTO_ALIGN while DISABLED
    if (from == SwerveStates.DISABLED && to == SwerveStates.AUTO_ALIGN) {
      return false;
    }
    return true;
  }

  public static void requestSwerveStateValidated(SwerveStates newState) {
    if (canTransitionSwerve(kSelectedSwerveState, newState)) {
      requestSwerveState(newState);
    } else {
      Logger.recordOutput(dashboardKey + "/InvalidTransition", 
          kSelectedSwerveState.toString() + " -> " + newState.toString());
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // LOGGING
  // ═══════════════════════════════════════════════════════════════════════════
  public static void logStates() {
    Logger.recordOutput(dashboardKey + "/SwerveState", kSelectedSwerveState.toString());
    Logger.recordOutput(dashboardKey + "/IntakeState", kSelectedIntakeState.toString());
    Logger.recordOutput(dashboardKey + "/PreviousSwerveState", kPreviousSwerveState.toString());
    Logger.recordOutput(dashboardKey + "/PreviousIntakeState", kPreviousIntakeState.toString());
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // STATIC TRIGGER CONSTANTS (for convenience)
  // ═══════════════════════════════════════════════════════════════════════════
  public static final class SwerveTriggers {
    public static final Trigger kRawDriving = swerveStateIs(SwerveStates.RAW_DRIVING);
    public static final Trigger kControlledDriving = swerveStateIs(SwerveStates.CONTROLLED_DRIVING);
    public static final Trigger kAutoAlign = swerveStateIs(SwerveStates.AUTO_ALIGN);
    public static final Trigger kIntakeAssistance = swerveStateIs(SwerveStates.INTAKE_ASSISTANCE);
    public static final Trigger kAuton = swerveStateIs(SwerveStates.AUTON);
    public static final Trigger kDisabled = swerveStateIs(SwerveStates.DISABLED);
  }

  public static final class IntakeTriggers {
    public static final Trigger kIntaking = intakeStateIs(IntakeStates.INTAKING);
    public static final Trigger kDisabled = intakeStateIs(IntakeStates.DISABLED);
    public static final Trigger kStowed = intakeStateIs(IntakeStates.STOWED);
    public static final Trigger kOutaking = intakeStateIs(IntakeStates.OUTAKING);
  }
}