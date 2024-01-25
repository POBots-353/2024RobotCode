// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.Timer;

public class FlashingLights extends Command {
  private final Color color;
  private final LEDs leds;
  private Timer timer = new Timer();
  double time;

  /** Creates a new FlashingLights. */
  public FlashingLights(double time,Color color, LEDs leds) {
    this.color = color;
    this.leds = leds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double getTime = timer.get();
    AddressableLEDBuffer buffer = leds.getBuffer();
    if (getTime <= time) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
      leds.updateBuffer();
    } else if(getTime >= 2*time) {
      timer.restart();
    } else {
      for (int i = 0; i < buffer.getLength(); i++){
      buffer.setLED(i, LEDConstants.transparent);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
