package team5427.lib.kinematics.shooter.projectiles.parabolic;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Getter;
import lombok.Setter;
import team5427.lib.detection.tuples.Tuple3Plus;

public class AdjustedParabolicThread extends Thread {

  private static AdjustedParabolicThread instance = null;
  @Getter private AdjustedParabolic parabolic = null;

  @Getter @Setter
  private Tuple3Plus<Rotation2d, LinearVelocity, LinearVelocity> outputState =
      new Tuple3Plus<Rotation2d, LinearVelocity, LinearVelocity>(
          Rotation2d.kZero, MetersPerSecond.of(-1), MetersPerSecond.of(-1));

  @Getter @Setter private boolean shouldCompute = false;

  private AdjustedParabolicThread() {
    setName("AdjustedParabolicSolverThread");
    setDaemon(true);
    parabolic = new AdjustedParabolic(new AdjustedParabolicConfiguration());
    parabolic.setProjectilePositionToTarget(Translation3d.kZero);
  }

  public static AdjustedParabolicThread getInstance() {
    if (instance == null) {
      instance = new AdjustedParabolicThread();
    }
    return instance;
  }

  public void setConfiguration(AdjustedParabolicConfiguration configuration) {
    parabolic.setConfiguration(configuration);
  }

  @Override
  public void start() {
    if (parabolic != null) {
      super.start();
    }
  }

  public AdjustedParabolicConfiguration getConfiguration() {
    return parabolic.getConfiguration();
  }

  public void setTarget(Translation3d translation3d) {
    parabolic.setProjectilePositionToTarget(translation3d);
  }

  @Override
  public void run() {
    System.out.println("Running");
    while (true) {

      outputState = parabolic.calculateSetpoints();
      super.run();
    }
  }
}
