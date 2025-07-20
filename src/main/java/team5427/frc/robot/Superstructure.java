package team5427.frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure {
    public static enum SwerveStates{
        DRIVING,
        AUTO_ALIGN,
        INTAKE_ASSISTANCE,
        AUTON;
        public static class SwerveTriggers {
            public static final Trigger kDriving = new Trigger(() ->{
                return kSelectedSwerveState.equals(DRIVING);
            });
            public static final Trigger kAuto_Align = new Trigger(() ->{
                return kSelectedSwerveState.equals(AUTO_ALIGN);
            });
            public static final Trigger kIntake_Assistance = new Trigger(() ->{
                return kSelectedSwerveState.equals(INTAKE_ASSISTANCE);
            });
            public static final Trigger kAuton = new Trigger(() ->{
                return kSelectedSwerveState.equals(AUTON);
            });
        }
    }
    public static SwerveStates kSelectedSwerveState = SwerveStates.DRIVING;
}
