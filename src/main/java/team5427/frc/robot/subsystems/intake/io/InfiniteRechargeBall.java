package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class InfiniteRechargeBall extends GamePieceOnFieldSimulation {

  public static final GamePieceInfo INFINITE_RECHARGE_BALL_INFO =
      new GamePieceInfo(
          "Ball",
          Geometry.createCircle(Units.inchesToMeters(14) / 2),
          Inches.of(7),
          Grams.of(142),
          3.5,
          5,
          0.3);

  public InfiniteRechargeBall(Translation2d initialPosition) {
    super(INFINITE_RECHARGE_BALL_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
