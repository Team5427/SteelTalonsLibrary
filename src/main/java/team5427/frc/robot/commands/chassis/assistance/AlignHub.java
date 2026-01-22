package team5427.frc.robot.commands.chassis.assistance;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.team4206.battleaid.common.TunedJoystick;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5427.frc.robot.RobotPose;
import team5427.frc.robot.Constants.RobotConfigConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignHub extends Command{
     private SwerveSubsystem swerve;
    private HolonomicDriveController driveController;
    private CommandXboxController joy;
    private TunedJoystick translationalJoystick;
    private static Pose2d targetPose;
    private boolean isRight;
    public AlignHub(CommandXboxController driverJoystick){
        swerve = SwerveSubsystem.getInstance();
        driveController = new HolonomicDriveController(SwerveConstants.kTranslationXPIDController, SwerveConstants.kTranslationYPIDController, SwerveConstants.kRotationPIDController);
        driveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromRadians(Math.PI/36)));
        
        joy = driverJoystick;
        translationalJoystick = new TunedJoystick(joy.getHID());
        if(DriverStation.getAlliance().equals(Alliance.Blue)){
            targetPose = RobotConfigConstants.BLUEPOSE2D;

        }
        else{
            targetPose = RobotConfigConstants.REDPOSE2D;
        }


    }
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        Logger.recordOutput("Target Pose", targetPose);
        Pose2d robotPose = RobotPose.getInstance().getAdaptivePose();
        Logger.recordOutput("Relative Target Pose", targetPose.relativeTo(robotPose));
        ChassisSpeeds speeds = driveController.calculate(Pose2d.kZero, targetPose.relativeTo(robotPose), SwerveConstants.kAutoAlignTranslationalMaxSpeed, targetPose.relativeTo(robotPose).getRotation());
        if(Math.abs(robotPose.getRotation().getRadians())<Math.PI/2){
            speeds.vyMetersPerSecond *=-1;
        }
        if(DriverStation.getAlliance().get()==Alliance.Blue){
            speeds.vyMetersPerSecond*=-1;
        }
        if(driveController.getThetaController().atSetpoint()&&driveController.getYController().atSetpoint()){
            speeds = new ChassisSpeeds(0, 0, 0);

        }
        double vx = translationalJoystick.getRightY();
        double dampener = (joy.getRightTriggerAxis()*SwerveConstants.kDampenerDampeningAmount);
        ChassisSpeeds driverSpeeds = swerve.getDriveSpeeds(0.0,speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, dampener);
        driverSpeeds.vxMetersPerSecond = swerve.getDriveSpeeds(-vx, 0.0, 0.0, dampener, targetPose.getRotation()).vxMetersPerSecond;
        swerve.setInputSpeeds(driverSpeeds);
    }
    @Override
    public boolean isFinished(){
        return false;

    }
    @Override
    public void end(boolean interrupted){
        swerve.setInputSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
