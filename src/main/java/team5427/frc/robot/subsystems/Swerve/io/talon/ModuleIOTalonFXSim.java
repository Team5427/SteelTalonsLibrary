package team5427.frc.robot.subsystems.Swerve.io.talon;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import team5427.frc.robot.subsystems.Swerve.io.ModuleIO.ModuleIOInputs;
import team5427.lib.motors.PhoenixUtil;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(int index, SwerveModuleSimulation simulation) {

    super(
        PhoenixUtil.regulateModuleConstantForSimulation(
            PhoenixUtil.getSwerveModuleConstants(index)),
        index);

    this.simulation = simulation;
    simulation.useDriveMotorController(
        new PhoenixUtil.TalonFXMotorControllerSim(driveMotor.getTalonFX()));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            steerMotor.getTalonFX(), cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsMeters =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(
                (angle) -> angle.in(Rotation) * driveMotor.getConversionFactorFromRotations())
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setSteerPositionSetpoint(Rotation2d angle) {
    if (Math.abs(super.absolutePosition.getValue().in(Degree) - angle.getDegrees()) > 0.01) {
      super.setSteerPositionSetpoint(angle);
    } else {
      steerMotor.setSetpoint(super.steerMotorPosition.getValue());
    }
  }
}
