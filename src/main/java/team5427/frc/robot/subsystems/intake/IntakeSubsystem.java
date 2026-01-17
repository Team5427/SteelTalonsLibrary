package team5427.frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.Constants;
import team5427.frc.robot.Constants.Mode;
import team5427.frc.robot.subsystems.intake.io.IntakeIO;
import team5427.frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import team5427.frc.robot.subsystems.intake.io.IntakeIOMagicTalonFX;
import team5427.frc.robot.subsystems.intake.io.IntakeIOSim;

public class IntakeSubsystem extends SubsystemBase {
  private LinearVelocity intakingSpeed;
  private Rotation2d intakingAngle;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputsAutoLogged;

  public static IntakeSubsystem m_instance;

  public final Alert kIntakingSpeedOutOfBounds =
      new Alert("OutOfBounds", "Intaking Speed Requested Out of Bounds", AlertType.kWarning);
  public final Alert kIntakingRotationOutOfBounds =
      new Alert("OutOfBounds", "Intaking Rotation Requested Out of Bounds", AlertType.kWarning);

  public static IntakeSubsystem getInstance(
      Supplier<SwerveDriveSimulation> swerveDriveSimulationSupplier) {
    if (m_instance == null) {
      m_instance = new IntakeSubsystem(Optional.of(swerveDriveSimulationSupplier));
    }
    return m_instance;
  }

  public static IntakeSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSubsystem(Optional.empty());
    }
    return m_instance;
  }

  private IntakeSubsystem(Optional<Supplier<SwerveDriveSimulation>> swerveDriveSimulationSupplier) {
    inputsAutoLogged = new IntakeIOInputsAutoLogged();
    switch (Constants.currentMode) {
      case REAL:
        io = new IntakeIOMagicTalonFX();
        break;
      case SIM:
        if (swerveDriveSimulationSupplier.isEmpty()) {
          DriverStation.reportWarning(
              "Intake Subsystem Simulation did not receive a Swerve Drive Simulation Supplier",
              true);
        }
        io = new IntakeIOSim(swerveDriveSimulationSupplier.get());
        break;
      default:
        break;
    }
    intakingSpeed = MetersPerSecond.of(0.0);
    intakingAngle = Rotation2d.kZero;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputsAutoLogged);

    if (Math.abs(intakingSpeed.in(MetersPerSecond)) > 10.0) {
      kIntakingSpeedOutOfBounds.set(true);
    } else {
      kIntakingSpeedOutOfBounds.set(false);
      io.setRollerSpeed(intakingSpeed);
    }
    if (intakingAngle.getDegrees() > IntakeConstants.kPivotMaximumRotation.getDegrees()
        || intakingAngle.getDegrees() < IntakeConstants.kPivotMinimumRotation.getDegrees()) {
      kIntakingRotationOutOfBounds.set(true);
    } else {
      kIntakingRotationOutOfBounds.set(false);
      io.setPivotRotation(intakingAngle);
    }
    Logger.processInputs("Intake/Inputs", inputsAutoLogged);
    log();
  }

  public void simulateIntaking(boolean isIntaking) {
    if (Constants.currentMode.equals(Mode.SIM)) {
      IntakeIOSim ioSim = (IntakeIOSim) io;
      ioSim.setRunning(isIntaking);
    }
  }

  public void setIntakingSpeed(LinearVelocity speed) {
    intakingSpeed = speed;
  }

  public void setIntakingRotation(Rotation2d angle) {
    intakingAngle = angle;
  }

  public void disableRollerMotor(boolean shouldDisable) {
    io.disableRollerMotor(shouldDisable);
  }

  public void disablePivotMotor(boolean shouldDisable) {
    io.disablePivotMotor(shouldDisable);
  }

  public boolean isRollerMotorDisabled() {
    return inputsAutoLogged.rollerMotorDisabled;
  }

  public boolean isPivotMotorDisabled() {
    return inputsAutoLogged.pivotMotorDisabled;
  }

  public void log() {
    Logger.recordOutput("Intake/IntakingSpeed", intakingSpeed.in(MetersPerSecond));
    Logger.recordOutput("Intake/IntakingAngle", intakingAngle);
  }
}
