// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5427.frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;

import team5427.frc.robot.io.PilotingControls;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    try{
    Constants.config = RobotConfig.fromGUISettings();
    } catch(Exception e){
      System.out.println("Robot Config not loading from GUI Settings");
      e.printStackTrace();
      return;
    }

    switch (Constants.currentMode) {
      case REAL:
        SwerveSubsystem.getInstance(RobotState.getInstance()::addOdometryMeasurement);
        break;
      case REPLAY:
        SwerveSubsystem.getInstance(RobotState.getInstance()::addOdometryMeasurement);
        break;
      case SIM:
        SwerveSubsystem.getInstance(RobotState.getInstance()::addOdometryMeasurement);
        SimulatedArena.getInstance()
            .addDriveTrainSimulation(SwerveSubsystem.getInstance().getKDriveSimulation());
        SimulatedArena.getInstance().clearGamePieces();
        
        break;
      default:
        break;
    }
    buttonBindings();
  }

  private void buttonBindings() {
    new PilotingControls();
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

        SwerveSubsystem.getInstance().getKDriveSimulation().setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", SwerveSubsystem.getInstance().getKDriveSimulation().getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
