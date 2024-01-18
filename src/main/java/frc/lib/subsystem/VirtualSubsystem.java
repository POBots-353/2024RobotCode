// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem extends SubsystemBase {
  protected List<Alert> prematchAlerts = new ArrayList<Alert>();
  protected String systemStatus = "Pre-Match not ran";

  public final void cancelCurrentCommand() {
    Command currentCommand = getCurrentCommand();
    Command defaultCommand = getDefaultCommand();

    if (currentCommand != null && !(defaultCommand != null && currentCommand == defaultCommand)) {
      currentCommand.cancel();
    }
  }

  public final String getAlertGroup() {
    return getName() + "/Alerts";
  }

  public void clearAlerts() {
    for (Alert alert : prematchAlerts) {
      alert.removeFromGroup();
    }

    prematchAlerts.clear();
  }

  private final void addAlert(Alert alert) {
    alert.set(true);
    prematchAlerts.add(alert);
  }

  public final void addInfo(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.INFO));
  }

  public final void addWarning(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.WARNING));
  }

  public final void addError(String message) {
    addAlert(new Alert(getAlertGroup(), message, AlertType.ERROR));
    setSystemStatus("Pre-Match failed with reason: \"" + message + "\"");
  }

  public final void setSystemStatus(String status) {
    systemStatus = status;
  }

  public final String getSystemStatus() {
    return systemStatus;
  }

  public final boolean containsErrors() {
    for (Alert alert : prematchAlerts) {
      if (alert.getType() == AlertType.ERROR) {
        return true;
      }
    }

    return false;
  }

  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.none();
  }

  public Command buildPrematch(VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  cancelCurrentCommand();
                  clearAlerts();
                  setSystemStatus("Running Pre-Match Check");
                }),
            getPrematchCheckCommand(controller, joystick))
        .until(this::containsErrors)
        .finallyDo(
            (interrupted) -> {
              cancelCurrentCommand();

              if (interrupted && !containsErrors()) {
                addError("Pre-Match Interrpted");
              } else if (!interrupted && !containsErrors()) {
                setSystemStatus("Pre-Match Successful!");
              }

              controller.clearVirtualAxes();
              controller.clearVirtualButtons();

              joystick.clearVirtualAxes();
              joystick.clearVirtualButtons();
            });
  }
}
