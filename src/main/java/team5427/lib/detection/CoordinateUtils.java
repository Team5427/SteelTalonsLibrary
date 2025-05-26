package team5427.lib.detection;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public final class CoordinateUtils {
  public static Transform2d transform3dTo2d(Transform3d transform3d) {
    return new Transform2d(
        transform3d.getX(), transform3d.getY(), transform3d.getRotation().toRotation2d());
  }
}
