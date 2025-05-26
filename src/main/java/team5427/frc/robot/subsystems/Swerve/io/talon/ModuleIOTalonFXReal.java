package team5427.frc.robot.subsystems.Swerve.io.talon;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;

public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
  // Queue to read inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> steerPositionQueue;

  public ModuleIOTalonFXReal(int index) {
    super(index);

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.driveMotorPosition);
    this.steerPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.steerMotorPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            // converts the raw rotations -> radians -> meters
            .mapToDouble(
                (Double value) -> {
                  return value * driveMotor.getConversionFactorFromRotations();
                })
            .toArray();
    inputs.odometryTurnPositions =
        steerPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    steerPositionQueue.clear();
  }
}
