package team5427.frc.robot.commands.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.RobotPose;

public class TargetingCommand extends Command {
    private static final Translation2d kTargetPosition = new Translation2d(0.0, 0.0);

    private final RobotPose robotPose;
    private final InterpolatingDoubleTreeMap shooterAngleLUT;
    private final InterpolatingDoubleTreeMap flywheel1SpeedLUT;
    private final InterpolatingDoubleTreeMap flywheel2SpeedLUT;

    private double currentDistance = 0.0;
    private double shooterAngle = 0.0;
    private double flywheel1Speed = 0.0;
    private double flywheel2Speed = 0.0;

    public TargetingCommand() {
        this.robotPose = RobotPose.getInstance();
        this.shooterAngleLUT = new InterpolatingDoubleTreeMap();
        this.flywheel1SpeedLUT = new InterpolatingDoubleTreeMap();
        this.flywheel2SpeedLUT = new InterpolatingDoubleTreeMap();

        shooterAngleLUT.put(1.0, 45.0);
        shooterAngleLUT.put(2.0, 35.0);
        shooterAngleLUT.put(3.0, 30.0);
        shooterAngleLUT.put(4.0, 25.0);
        shooterAngleLUT.put(5.0, 20.0);

        flywheel1SpeedLUT.put(1.0, 3000.0);
        flywheel1SpeedLUT.put(2.0, 3500.0);
        flywheel1SpeedLUT.put(3.0, 4000.0);
        flywheel1SpeedLUT.put(4.0, 4500.0);
        flywheel1SpeedLUT.put(5.0, 5000.0);

        flywheel2SpeedLUT.put(1.0, 3000.0);
        flywheel2SpeedLUT.put(2.0, 3500.0);
        flywheel2SpeedLUT.put(3.0, 4000.0);
        flywheel2SpeedLUT.put(4.0, 4500.0);
        flywheel2SpeedLUT.put(5.0, 5000.0);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Targeting/Active", true);
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotPose.getAdaptivePose();
        currentDistance = currentPose.getTranslation().getDistance(kTargetPosition);

        shooterAngle = shooterAngleLUT.get(currentDistance);
        flywheel1Speed = flywheel1SpeedLUT.get(currentDistance);
        flywheel2Speed = flywheel2SpeedLUT.get(currentDistance);

        Logger.recordOutput("Targeting/Distance", currentDistance);
        Logger.recordOutput("Targeting/ShooterAngle", shooterAngle);
        Logger.recordOutput("Targeting/Flywheel1Speed", flywheel1Speed);
        Logger.recordOutput("Targeting/Flywheel2Speed", flywheel2Speed);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Targeting/Active", false);
        if (interrupted) {
            Logger.recordOutput("Targeting/Interrupted", true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getDistance() {
        return currentDistance;
    }

    public double getShooterAngle() {
        return shooterAngle;
    }

    public double getFlywheel1Speed() {
        return flywheel1Speed;
    }

    public double getFlywheel2Speed() {
        return flywheel2Speed;
    }
}
