package frc.lib.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class VirtualJoystick extends CommandJoystick {
  private Map<Integer, Boolean> virtualButtons = new HashMap<>();
  private Map<AxisType, Optional<Double>> virtualAxes = new HashMap<>();

  private boolean virtualButtonsEnabled = false;
  private boolean virtualAxesEnabled = false;

  public VirtualJoystick(int port) {
    super(port);

    for (AxisType axis : AxisType.values()) {
      virtualAxes.put(axis, Optional.empty());
    }

    new Trigger(DriverStation::isDisabled)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      clearVirtualAxes();
                      clearVirtualButtons();
                    })
                .ignoringDisable(true));
  }

  @Override
  public Trigger button(int button, EventLoop loop) {
    virtualButtons.putIfAbsent(button, false);

    return super.button(button, loop).or(() -> virtualButtonsEnabled && virtualButtons.get(button));
  }

  @Override
  public double getX() {
    if (!virtualAxesEnabled) {
      return super.getX();
    }
    if (virtualAxes.get(AxisType.kX).isPresent() && !DriverStation.isFMSAttached()) {
      return virtualAxes.get(AxisType.kX).get();
    }

    return super.getX();
  }

  @Override
  public double getY() {
    if (!virtualAxesEnabled) {
      return super.getY();
    }
    if (virtualAxes.get(AxisType.kY).isPresent() && !DriverStation.isFMSAttached()) {
      return virtualAxes.get(AxisType.kY).get();
    }

    return super.getY();
  }

  @Override
  public double getZ() {
    if (!virtualAxesEnabled) {
      return super.getZ();
    }
    if (virtualAxes.get(AxisType.kZ).isPresent() && !DriverStation.isFMSAttached()) {
      return virtualAxes.get(AxisType.kZ).get();
    }

    return super.getZ();
  }

  @Override
  public double getTwist() {
    if (!virtualAxesEnabled) {
      return super.getTwist();
    }
    if (virtualAxes.get(AxisType.kTwist).isPresent() && !DriverStation.isFMSAttached()) {
      return virtualAxes.get(AxisType.kTwist).get();
    }

    return super.getTwist();
  }

  @Override
  public double getThrottle() {
    if (!virtualAxesEnabled) {
      return super.getThrottle();
    }
    if (virtualAxes.get(AxisType.kThrottle).isPresent() && !DriverStation.isFMSAttached()) {
      return virtualAxes.get(AxisType.kThrottle).get();
    }

    return super.getThrottle();
  }

  public void clearVirtualAxes() {
    if (!virtualAxesEnabled) {
      return;
    }
    virtualAxesEnabled = false;
    for (AxisType axis : virtualAxes.keySet()) {
      virtualAxes.replace(axis, Optional.empty());
    }
  }

  public void clearVirtualButtons() {
    for (int button : virtualButtons.keySet()) {
      virtualButtons.replace(button, false);
    }
    virtualButtonsEnabled = false;
  }

  public void setButton(int button, boolean value) {
    if (DriverStation.isFMSAttached()) {
      return;
    }
    virtualButtons.putIfAbsent(button, false);
    virtualButtonsEnabled = true;

    virtualButtons.replace(button, value);
  }

  public void setVirtualAxis(AxisType axis, double value) {
    if (DriverStation.isFMSAttached()) {
      DriverStation.reportError("Cannot set virtual axis while FMS is connected", true);
      return;
    }
    virtualAxesEnabled = true;
    virtualAxes.replace(axis, Optional.of(value));
  }

  public void setX(double value) {
    setVirtualAxis(AxisType.kX, value);
  }

  public void setY(double value) {
    setVirtualAxis(AxisType.kY, value);
  }

  public void setZ(double value) {
    setVirtualAxis(AxisType.kZ, value);
  }

  public void setTwist(double value) {
    setVirtualAxis(AxisType.kTwist, value);
  }

  public void setThrottle(double value) {
    setVirtualAxis(AxisType.kThrottle, value);
  }
}
