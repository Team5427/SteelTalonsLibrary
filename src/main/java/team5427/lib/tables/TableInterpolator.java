package team5427.lib.tables;

import edu.wpi.first.math.interpolation.Interpolator;

public class TableInterpolator implements Interpolator<Double> {

  // 60 - 70 .5
  @Override
  public Double interpolate(Double startValue, Double endValue, double t) {
    Double interpolatedValue = startValue + (endValue - startValue) * t;
    return interpolatedValue;
  }
}
