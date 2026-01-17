package team5427.lib.tables;

import edu.wpi.first.math.interpolation.InverseInterpolator;

public class TableInverseInterpolator implements InverseInterpolator<Double> {
  @Override
  public double inverseInterpolate(Double startValue, Double endValue, Double q) {
    return (q - startValue) / (endValue - startValue);
  }
}
