package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.awt.geom.Point2D;
import java.util.List;

public class LinearInterpolation extends InterpolatingDoubleTreeMap {
  public LinearInterpolation(List<Point2D> points) {
    for (Point2D point : points) {
      put(point.getX(), point.getY());
    }
  }
}
