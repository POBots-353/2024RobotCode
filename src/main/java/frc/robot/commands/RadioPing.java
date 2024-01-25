// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import java.net.InetAddress;
import java.util.function.Consumer;

public class RadioPing extends Command {
  private Notifier notifier;

  private InetAddress ipAddress;

  private boolean receivedConnection = false;
  private int responseCount = 0;

  boolean initialFailed = false;
  Consumer<Boolean> onInitialFailed;

  /** Creates a new RadioPing. */
  public RadioPing(Consumer<Boolean> onInitialFailed) {
    try {
      ipAddress = InetAddress.getByName("10.3.53.1");
    } catch (Exception e) {
      e.printStackTrace();
    }

    this.onInitialFailed = onInitialFailed;
    notifier = new Notifier(this::pingIPAddress);
  }

  public RadioPing() {
    this((value) -> {});
  }

  private void pingIPAddress() {
    try {
      if (ipAddress.isReachable(250)) {
        synchronized (this) {
          receivedConnection = true;
          responseCount++;
        }
      } else {
        synchronized (this) {
          initialFailed = true;
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notifier.startPeriodic(0.500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    notifier.stop();
    onInitialFailed.accept(initialFailed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!initialFailed) {
      return receivedConnection;
    } else {
      return receivedConnection && responseCount >= 1;
    }
  }
}
