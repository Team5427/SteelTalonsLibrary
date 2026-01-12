package team5427.lib.tables;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

// Distance -> Pivot Angle
// Distance -> Flywheel 1 speed
// Distance -> Flywheel 2 speed

public class LookUpTable {
  private InterpolatingTreeMap<Double, Double> pivotAngleLookup;
  private InterpolatingTreeMap<Double, Double> flywheel1Lookup;
  private InterpolatingTreeMap<Double, Double> flywheel2Lookup;

  public LookUpTable() {
    pivotAngleLookup =
        new InterpolatingTreeMap<>(new TableInverseInterpolator(), new TableInterpolator());
    flywheel1Lookup =
        new InterpolatingTreeMap<>(new TableInverseInterpolator(), new TableInterpolator());
    flywheel2Lookup =
        new InterpolatingTreeMap<>(new TableInverseInterpolator(), new TableInterpolator());
  }

  public void addpivotAngle(Double distance, Double pivotAngle) {
    pivotAngleLookup.put(distance, pivotAngle);
  }

  public void addflyWheel1Speed(Double distance, Double speed) {
    flywheel1Lookup.put(distance, speed);
  }

  public void addflyWheel2Speed(Double distance, Double speed) {
    flywheel2Lookup.put(distance, speed);
  }

  public Double getpivotAngle(Double distance) {
    return pivotAngleLookup.get(distance);
  }

  public Double getflyWheel1Speed(Double distance) {
    return flywheel1Lookup.get(distance);
  }

  public Double getflyWheel2Speed(Double distance) {
    return flywheel2Lookup.get(distance);
  }
}
