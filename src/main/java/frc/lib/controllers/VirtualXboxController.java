package frc.lib.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class VirtualXboxController extends CommandXboxController {
  private Map<XboxController.Button, Boolean> virtualButtons = new HashMap<>();
  private Map<XboxController.Axis, Optional<Double>> virtualAxes = new HashMap<>();

  private boolean virtualAxesEnabled = false;
  private boolean virtualButtonsEnabled = false;

  private boolean virtualPOVEnabled = false;
  private int virtualPOVValue = -1;

  public VirtualXboxController(int port) {
    super(port);

    for (XboxController.Button button : XboxController.Button.values()) {
      virtualButtons.put(button, false);
    }

    for (XboxController.Axis axis : XboxController.Axis.values()) {
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

  public Command rumbleFor(double duration, RumbleType rumbleType, double value) {
    return Commands.sequence(
            Commands.runOnce(() -> getHID().setRumble(rumbleType, value)),
            Commands.waitSeconds(duration))
        .finallyDo(() -> getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  // New
  public Command rumble(RumbleType rumbleType, double value) {
    return Commands.run(() -> getHID().setRumble(rumbleType, value));
  }

  public Command stopRumble() {
    return Commands.run(() -> getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  @Override
  public Trigger leftBumper(EventLoop loop) {
    return super.leftBumper(loop)
        .or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kLeftBumper));
  }

  public boolean getLeftBumper() {
    return getHID().getLeftBumper()
        || (virtualButtonsEnabled && virtualButtons.get(Button.kLeftBumper));
  }

  @Override
  public Trigger rightBumper(EventLoop loop) {
    return super.rightBumper(loop)
        .or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kRightBumper));
  }

  public boolean getRightBumper() {
    return getHID().getRightBumper()
        || (virtualButtonsEnabled && virtualButtons.get(Button.kRightBumper));
  }

  @Override
  public Trigger leftStick(EventLoop loop) {
    return super.leftStick(loop)
        .or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kLeftStick));
  }

  @Override
  public Trigger rightStick(EventLoop loop) {
    return super.rightStick(loop)
        .or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kRightStick));
  }

  @Override
  public Trigger a(EventLoop loop) {
    return super.a(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kA));
  }

  @Override
  public Trigger b(EventLoop loop) {
    return super.b(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kB));
  }

  @Override
  public Trigger x(EventLoop loop) {
    return super.x(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kX));
  }

  @Override
  public Trigger y(EventLoop loop) {
    return super.y(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kY));
  }

  @Override
  public Trigger start(EventLoop loop) {
    return super.start(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kStart));
  }

  @Override
  public Trigger back(EventLoop loop) {
    return super.back(loop).or(() -> virtualButtonsEnabled && virtualButtons.get(Button.kBack));
  }

  @Override
  public double getLeftX() {
    if (!virtualAxesEnabled) {
      return super.getLeftX();
    }
    if (virtualAxes.get(Axis.kLeftX).isPresent()) {
      return virtualAxes.get(Axis.kLeftX).get();
    }
    return super.getLeftX();
  }

  @Override
  public double getRightX() {
    if (!virtualAxesEnabled) {
      return super.getRightX();
    }
    if (virtualAxes.get(Axis.kRightX).isPresent()) {
      return virtualAxes.get(Axis.kRightX).get();
    }
    return super.getRightX();
  }

  @Override
  public double getLeftY() {
    if (!virtualAxesEnabled) {
      return super.getLeftY();
    }
    if (virtualAxes.get(Axis.kLeftY).isPresent()) {
      return virtualAxes.get(Axis.kLeftY).get();
    }
    return super.getLeftY();
  }

  @Override
  public double getRightY() {
    if (!virtualAxesEnabled) {
      return super.getRightY();
    }
    if (virtualAxes.get(Axis.kRightY).isPresent()) {
      return virtualAxes.get(Axis.kRightY).get();
    }
    return super.getRightY();
  }

  @Override
  public double getLeftTriggerAxis() {
    if (!virtualAxesEnabled) {
      return super.getLeftTriggerAxis();
    }
    if (virtualAxes.get(Axis.kLeftTrigger).isPresent()) {
      return virtualAxes.get(Axis.kLeftTrigger).get();
    }
    return super.getLeftTriggerAxis();
  }

  @Override
  public double getRightTriggerAxis() {
    if (!virtualAxesEnabled) {
      return super.getRightTriggerAxis();
    }
    if (virtualAxes.get(Axis.kRightTrigger).isPresent()) {
      return virtualAxes.get(Axis.kRightTrigger).get();
    }
    return super.getRightTriggerAxis();
  }

  public int getPOV(int pov) {
    if (virtualPOVEnabled && !DriverStation.isFMSAttached()) {
      return virtualPOVValue;
    }
    return getHID().getPOV(pov);
  }

  public int getPOV() {
    return getPOV(0);
  }

  public Trigger pov(int angle, EventLoop loop) {
    return pov(0, angle, loop);
  }

  public Trigger pov(int pov, int angle, EventLoop loop) {
    return new Trigger(loop, () -> getPOV(pov) == angle);
  }

  public Trigger povUp(EventLoop loop) {
    return pov(0, loop);
  }

  public Trigger povUpRight(EventLoop loop) {
    return pov(45, loop);
  }

  public Trigger povRight(EventLoop loop) {
    return pov(90, loop);
  }

  public Trigger povDownRight(EventLoop loop) {
    return pov(135, loop);
  }

  public Trigger povDown(EventLoop loop) {
    return pov(180, loop);
  }

  public Trigger povDownLeft(EventLoop loop) {
    return pov(225, loop);
  }

  public Trigger povLeft(EventLoop loop) {
    return pov(270, loop);
  }

  public Trigger povUpLeft(EventLoop loop) {
    return pov(315, loop);
  }

  public Trigger povCenter(EventLoop loop) {
    return pov(-1, loop);
  }

  public void enableVirtualPOV() {
    virtualPOVEnabled = true;
  }

  public void disableVirtualPOV() {
    virtualPOVEnabled = false;
  }

  public void setPOV(int angle) {
    if (DriverStation.isFMSAttached()) {
      return;
    }
    virtualPOVEnabled = true;
    virtualPOVValue = angle;
  }

  public void enableVirtualAxis(XboxController.Axis axis) {
    if (DriverStation.isFMSAttached()) {
      DriverStation.reportError("Cannot enable virtual axis while FMS is connected", true);
      return;
    }

    virtualAxesEnabled = true;
    virtualAxes.replace(axis, Optional.of(0.0));
  }

  public void setVirtualAxis(XboxController.Axis axis, double value) {
    if (DriverStation.isFMSAttached()) {
      DriverStation.reportError("Cannot set virtual axis while FMS is connected", true);
      return;
    }

    virtualAxesEnabled = true;
    virtualAxes.replace(axis, Optional.of(value));
  }

  public void setVirtualButton(XboxController.Button button, boolean state) {
    if (DriverStation.isFMSAttached()) {
      DriverStation.reportError("Cannot set virtual button while FMS is connected", true);
      return;
    }

    virtualButtons.replace(button, state);
    virtualButtonsEnabled = true;
  }

  public void clearVirtualAxes() {
    if (!virtualAxesEnabled) {
      return;
    }
    virtualAxesEnabled = false;
    for (Axis axis : virtualAxes.keySet()) {
      virtualAxes.replace(axis, Optional.empty());
    }
  }

  public void clearVirtualButtons() {
    for (Button button : virtualButtons.keySet()) {
      virtualButtons.replace(button, false);
    }
    virtualButtonsEnabled = false;
  }

  public void setLeftX(double value) {
    setVirtualAxis(Axis.kLeftX, value);
  }

  public void setRightX(double value) {
    setVirtualAxis(Axis.kRightX, value);
  }

  public void setLeftY(double value) {
    setVirtualAxis(Axis.kLeftY, value);
  }

  public void setRightY(double value) {
    setVirtualAxis(Axis.kRightY, value);
  }

  public void setLeftTriggerAxis(double value) {
    setVirtualAxis(Axis.kLeftTrigger, value);
  }

  public void setRightTriggerAxis(double value) {
    setVirtualAxis(Axis.kRightTrigger, value);
  }

  public void setLeftBumper(boolean state) {
    setVirtualButton(Button.kLeftBumper, state);
  }

  public void setRightBumper(boolean state) {
    setVirtualButton(Button.kRightBumper, state);
  }

  public void setLeftStickButton(boolean state) {
    setVirtualButton(Button.kLeftStick, state);
  }

  public void setRightStickButton(boolean state) {
    setVirtualButton(Button.kRightStick, state);
  }

  public void setAButton(boolean state) {
    setVirtualButton(Button.kA, state);
  }

  public void setBButton(boolean state) {
    setVirtualButton(Button.kB, state);
  }

  public void setXButton(boolean state) {
    setVirtualButton(Button.kX, state);
  }

  public void setYButton(boolean state) {
    setVirtualButton(Button.kY, state);
  }

  public void setBackButton(boolean state) {
    setVirtualButton(Button.kBack, state);
  }

  public void setStartButton(boolean state) {
    setVirtualButton(Button.kStart, state);
  }
}
