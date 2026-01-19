package team5427.frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BuildConstants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import team5427.frc.robot.subsystems.Swerve.DrivingConstants;
import team5427.frc.robot.subsystems.Swerve.SwerveConstants;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.frc.robot.subsystems.vision.io.QuestNav;
import team5427.lib.drivers.JoystickLogger;
import team5427.lib.drivers.VirtualSubsystem;
import team5427.lib.kinematics.shooter.projectiles.parabolic.AdjustedParabolicThread;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  public Robot() {

    Logger.recordMetadata("Reefscape", "Steel Talons 5427 Robot Code for the Game Reefscape, 2025");
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (RobotBase.isReal()) {
      Constants.currentMode = Constants.Mode.REAL;
    } else if (RobotBase.isSimulation()) {
      Constants.currentMode = Constants.Mode.SIM;
    } else {
      Constants.currentMode = Constants.Mode.REPLAY;
    }
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.registerURCL(URCL.startExternal());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    AutoLogOutputManager.addPackage("team5427.lib");

    Logger.start();
    loadConstants();
    JoystickLogger.logJoystickData();
    m_robotContainer = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
    AdjustedParabolicThread.getInstance().setShouldCompute(true);
    AdjustedParabolicThread.getInstance().start();
  }

  private void loadConstants() {
    System.out.println(SwerveConstants.class);
    System.out.println(IntakeConstants.class);
    System.out.println(DrivingConstants.class);
    // loading a random LoggedTunableNumber
    System.out.println(DrivingConstants.kRotationKp);
    System.out.println(Superstructure.class);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    VirtualSubsystem.periodicAll();
    RobotPose.getInstance().log();
    Superstructure.logStates();
    QuestNav.getInstance().processHeartbeat();
    QuestNav.getInstance().cleanupResponses();

    if (Constants.ModeTriggers.kSim.getAsBoolean()) {
      Translation3d target =
          new Pose3d(RobotPose.getInstance().getAdaptivePose())
              .plus(new Transform3d(0, 0, 4, Rotation3d.kZero))
              .getTranslation();
      AdjustedParabolicThread.getInstance().setTarget(target);

      Logger.recordOutput(
          "Rotation Output: ",
          MathUtil.inputModulus(
              AdjustedParabolicThread.getInstance().getOutputState().r.getDegrees(), 0, 360));
      Logger.recordOutput(
          "Velocity Output: ",
          new Translation2d(
              AdjustedParabolicThread.getInstance().getOutputState().t.in(MetersPerSecond),
              AdjustedParabolicThread.getInstance().getOutputState().a.in(MetersPerSecond)));
      Logger.recordOutput("Target", target);
      Logger.recordOutput("Thread Interupted", AdjustedParabolicThread.interrupted());
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    System.out.println("Switched to Auton");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    m_robotContainer.resetSimulationField();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.updateSimulation();
  }
}
