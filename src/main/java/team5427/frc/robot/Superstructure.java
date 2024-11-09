package team5427.frc.robot;

/**
 * @apiNote <pre> Superstructure </pre> Represents the different distinct states that the robot can be in
 * which can correspond to varous actions, profiles, and behaviors.
*/
public enum Superstructure {

    ACTIVE,
    INACTIVE;

    /**
     * @apiNote <pre> Superstructure.Modes </pre> Represents the different Modes of the robot
     * to adjust the driving profiles, debugging levels, and motor speeds
     * accordingly
    */
    public enum Modes {
        COMPETITIONMODE,
        FREEMODE,
        KIDMODE,
        /**
            ONLY USE FOR TESTING NOT FOR REAL MATCHES
        */
        TESTINGMODE;
    };

    /**  
     * @apiNote <pre> Superstructure.Actions </pre> Represents the different actions the robot
    could be doing
    */
    public enum Actions {
        DEFENSE,
    };

    /**
     * @apiNote <pre> Superstructure.Errors </pre> Represents the different problematic
     * components of the robot.
     * This notifies the driver of potential issues, and compensates accordingly. 
    */
    public enum Errors {
        VISIONPOSEESTIMATION,
        CATASTROPHICISSUE,
    };
}
