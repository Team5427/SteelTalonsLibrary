package team5427.frc.robot.commands.chassis;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignRobotToHub extends Command{
    private SwerveSubsystem swerveSubsystem;

    private final Translation2d targetPose = new Translation2d(5,7); 

    private Rotation2d rotation;
    private Translation2d robotPose;
    private double robotVelocityX;
    private double robotVelocityY;//meters per second

    private final PIDController pidRotation = new PIDController(0.05, 0, 0); //random numbers

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1);

    public AlignRobotToHub(Pose2d robotPose2d, double velocityX, double velocityY){
        swerveSubsystem = SwerveSubsystem.getInstance();

        this.robotPose = robotPose2d.getTranslation();
        this.robotVelocityX = velocityX;
        this.robotVelocityY = velocityY;
        this.rotation = robotPose2d.getRotation();
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        ChassisSpeeds driverSpeeds;

        //robots x and y distance from the hub
        double dX = targetPose.getX()-robotPose.getX();
        double dY = targetPose.getY()-robotPose.getY();
        
        //predicts the robots next future pose
        double vX = robotVelocityX;
        double vY = robotVelocityY;

        dX = dX+vX*Constants.kLoopSpeed; 
        dY = dY+vY*Constants.kLoopSpeed;

        //angle needed to align the robot to the hub
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(dX,dY));
        //difference between the robot's current angle and the required angle
        Rotation2d angleDifference = targetAngle.minus(rotation);

        //recalculation using PID to account for real world errors
        double omegaRadians = pidRotation.calculate(angleDifference.getRadians(), 0);

        //rotates the robot
        driverSpeeds = new ChassisSpeeds(0,0,omegaRadians);
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
