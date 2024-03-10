package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record ShooterState(double topSpeed, double bottomSpeed)
    implements Interpolatable<ShooterState>, StructSerializable {
  public ShooterState() {
    this(0.0);
  }

  public ShooterState(double speed) {
    this(speed, speed);
  }

  public ShooterState minus(ShooterState other) {
    return new ShooterState(topSpeed - other.topSpeed, bottomSpeed - other.bottomSpeed);
  }

  public ShooterState plus(ShooterState other) {
    return new ShooterState(topSpeed + other.topSpeed, bottomSpeed + other.bottomSpeed);
  }

  public boolean isNear(ShooterState expected, double tolerance) {
    return MathUtil.isNear(expected.topSpeed, topSpeed, tolerance)
        && MathUtil.isNear(expected.bottomSpeed, bottomSpeed, tolerance);
  }

  @Override
  public ShooterState interpolate(ShooterState endValue, double t) {
    double topValue = MathUtil.interpolate(topSpeed, endValue.topSpeed, t);
    double bottomValue = MathUtil.interpolate(bottomSpeed, endValue.bottomSpeed, t);

    return new ShooterState(topValue, bottomValue);
  }

  public static final ShooterStateStruct struct = new ShooterStateStruct();

  public static class ShooterStateStruct implements Struct<ShooterState> {
    @Override
    public Class<ShooterState> getTypeClass() {
      return ShooterState.class;
    }

    @Override
    public String getTypeString() {
      return "struct:ShooterState";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
      return "double topSpeed;double bottomSpeed";
    }

    @Override
    public ShooterState unpack(ByteBuffer bb) {
      double topSpeed = bb.getDouble();
      double bottomSpeed = bb.getDouble();

      return new ShooterState(topSpeed, bottomSpeed);
    }

    @Override
    public void pack(ByteBuffer bb, ShooterState value) {
      bb.putDouble(value.topSpeed());
      bb.putDouble(value.bottomSpeed());
    }
  }
}
