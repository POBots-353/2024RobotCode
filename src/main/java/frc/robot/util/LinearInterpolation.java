package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.awt.geom.Point2D;

public class LinearInterpolation extends InterpolatingDoubleTreeMap {
  public LinearInterpolation(Point2D... points) {
    for (Point2D point : points) {
      put(point.getX(), point.getY());
    }
  }
}
