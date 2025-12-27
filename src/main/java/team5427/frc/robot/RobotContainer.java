// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5427.frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants.DriverConstants;
import team5427.frc.robot.io.DriverProfiles;
import team5427.frc.robot.io.OperatorControls;
import team5427.frc.robot.io.PilotingControls;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.intake.IntakeSubsystem;
import team5427.frc.robot.subsystems.vision.VisionSubsystem;
import team5427.frc.robot.subsystems.vision.io.QuestNav;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    try {
      Constants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.out.println("Robot Config not loading from GUI Settings");
      e.printStackTrace();
      return;
    }

    switch (Constants.currentMode) {
      case REAL:
        SwerveSubsystem.getInstance(RobotPose.getInstance()::addOdometryMeasurement);
        IntakeSubsystem.getInstance();
        break;
      case REPLAY:
        SwerveSubsystem.getInstance(RobotPose.getInstance()::addOdometryMeasurement);
        IntakeSubsystem.getInstance();
        break;
      case SIM:
        SwerveSubsystem.getInstance(RobotPose.getInstance()::addOdometryMeasurement);
        SimulatedArena.getInstance()
            .addDriveTrainSimulation(SwerveSubsystem.getInstance().getKDriveSimulation());
        SimulatedArena.getInstance().clearGamePieces();
        IntakeSubsystem.getInstance(SwerveSubsystem.getInstance()::getKDriveSimulation);
        break;
      default:
        break;
    }
    VisionSubsystem.getInstance(
        RobotPose.getInstance()::addVisionMeasurement,
        () -> RobotPose.getInstance().getAdaptivePose(),
        () -> RobotPose.getInstance().getGyroHeading());
    QuestNav.getInstance().setPose(new Pose2d(10 * Math.random(), 4, Rotation2d.kZero));

    buttonBindings();
  }

  private void buttonBindings() {
    new PilotingControls(
        DriverProfiles.kSelectedDriverState.modeType.equals(DriverProfiles.DriverModeType.SINGLE)
            ? new CommandXboxController(DriverConstants.kDriverJoystickPort)
            : new CommandXboxController(DriverConstants.kDriverJoystickPort));
    new OperatorControls(
        DriverProfiles.kSelectedDriverState.modeType.equals(DriverProfiles.DriverModeType.SINGLE)
            ? new CommandXboxController(DriverConstants.kDriverJoystickPort)
            : new CommandXboxController(DriverConstants.kOperatorJoystickPort));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    Pose2d pose = new Pose2d(3, 3, Rotation2d.kZero);

    SwerveSubsystem.getInstance().getKDriveSimulation().setSimulationWorldPose(pose);
    RobotPose.getInstance().resetAllPose(pose);
    SwerveSubsystem.getInstance().resetGyro(Rotation2d.kZero);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        SwerveSubsystem.getInstance().getKDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
