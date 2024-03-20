// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDs;

public class Binary353 extends Command {
  private final Color color;
  private final LEDs leds;

  private static final String binaryString = "3330555550333"; // "101100001";

  /** Creates a new Binary353. */
  public Binary353(Color color, LEDs leds) {
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
    AddressableLEDBuffer buffer = leds.getBuffer();
    int startIndex = buffer.getLength() - binaryString.length();
    for (int i = 0; i < startIndex; i++) {
      buffer.setLED(i, LEDConstants.transparent);
    }
    for (int i = 0; i < binaryString.length(); i++) {
      if (i + startIndex >= buffer.getLength()) {
        break;
      }
      if (binaryString.charAt(i) == '1') {
        buffer.setLED(i + startIndex, color);
      } else if (binaryString.charAt(i) == '3') {
        buffer.setLED(i + startIndex, Color.kBlue);
      } else if (binaryString.charAt(i) == '5') {
        buffer.setLED(i + startIndex, Color.kRed);
      } else {
        buffer.setLED(i + startIndex, LEDConstants.transparent);
      }
    }

    leds.updateBuffer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
