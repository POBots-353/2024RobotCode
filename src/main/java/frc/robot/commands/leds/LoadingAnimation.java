// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDs;

public class LoadingAnimation extends Command {
  private final LEDs leds;
  private final Color color;

  private static final int length = 6;
  private static final int incrementAmount = 1;
  private boolean increasing = true;
  private int startOffset = 0;

  /** Creates a new LoadingAnimation. */
  public LoadingAnimation(Color color, LEDs leds) {
    this.color = color;
    this.leds = leds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    increasing = true;
    startOffset = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AddressableLEDBuffer buffer = leds.getBuffer();
    if (increasing) {
      startOffset += incrementAmount;
    } else {
      startOffset -= incrementAmount;
    }

    if (increasing && startOffset + length >= buffer.getLength()) {
      startOffset = buffer.getLength() - length;
      increasing = false;
    } else if (!increasing && startOffset <= 0) {
      startOffset = 0;
      increasing = true;
    }

    for (int i = 0; i < buffer.getLength(); i++) {
      if (i >= startOffset && i <= startOffset + length) {
        buffer.setLED(i, color);
      } else {
        buffer.setLED(i, LEDConstants.transparent);
      }
    }
    leds.updateBuffer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
