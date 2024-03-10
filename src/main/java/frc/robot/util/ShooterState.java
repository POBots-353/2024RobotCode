package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShooterState(double topSpeed, double bottomSpeed)
    implements Interpolatable<ShooterState> {
  public ShooterState() {
    this(0.0);
  }

  public ShooterState(double speed) {
    this(speed, speed);
  }

  @Override
  public ShooterState interpolate(ShooterState endValue, double t) {
    double topValue = MathUtil.interpolate(topSpeed, endValue.topSpeed, t);
    double bottomValue = MathUtil.interpolate(bottomSpeed, endValue.bottomSpeed, t);

    return new ShooterState(topValue, bottomValue);
  }
}
