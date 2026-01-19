package team5427.frc.robot.commands.chassis;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignRobotToHub extends Command{
    private SwerveSubsystem swerveSubsystem;

    private final Translation2d targetPose = new Translation2d(5,7); 
    private final Rotation2d targetAngle = Rotation2d.fromRadians(Math.PI);

    private Rotation2d rotation;
    private Translation2d robotPose;

    private final PIDController pidController = new PIDController(1,1,1); //not sure what to input in the parameters
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1);

    public AlignRobotToHub(Pose2d robotPose2d){
        swerveSubsystem = SwerveSubsystem.getInstance();

        this.robotPose = robotPose2d.getTranslation();
        this.rotation = robotPose2d.getRotation();
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        ChassisSpeeds driverSpeeds;

        Translation2d poseDistance = targetPose.minus(robotPose);
        Rotation2d angleDifference = targetAngle.minus(rotation);

        double vX = pidController.calculate(poseDistance.getMeasureX().in(Meters));
        double vY = pidController.calculate(poseDistance.getMeasureY().in(Meters));
        double omegaRadians = pidController.calculate(angleDifference.getRadians());

        driverSpeeds = new ChassisSpeeds(vX,vY,omegaRadians);
        swerveSubsystem.setInputSpeeds(driverSpeeds);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setInputSpeeds(new ChassisSpeeds());
    }
}
