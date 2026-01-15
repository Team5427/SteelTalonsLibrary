package team5427.frc.robot.commands.chassis;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignRobotToHub extends Command{
    private final Distance targetX = Meters.of(5); 
    private final Distance targetY = Meters.of(7);
    private final Rotation2d targetAngle = Rotation2d.fromDegrees(180);

    private Distance poseX;
    private Distance poseY;
    private Rotation2d rotation;
    private Pose2d robotPose2d;

    private final PIDController pidController = new PIDController(1,1,1); //not sure what to input in the parameters
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 1);

    public AlignRobotToHub(Pose2d robotPose2d){
        this.robotPose2d = robotPose2d;
        this.poseX = robotPose2d.getMeasureX();
        this.poseY = robotPose2d.getMeasureY();
        this.rotation = robotPose2d.getRotation();
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        Distance distanceX = targetX.minus(poseX);
        Distance distanceY = targetY.minus(poseY);
        Rotation2d angleDifference = targetAngle.minus(rotation);

        distanceX = Meters.of(pidController.calculate(distanceX.in(Meters)));
        distanceY = Meters.of(pidController.calculate(distanceY.in(Meters)));
        angleDifference = Rotation2d.fromDegrees(pidController.calculate(angleDifference.getDegrees()));

        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
