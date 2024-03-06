// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDs;

public class FlashingLights extends Command {
  private final Color color;
  private final LEDs leds;
  private Timer timer = new Timer();

  private double period;
  private boolean ledsOn = false;

  /** Creates a new FlashingLights. */
  public FlashingLights(double period, Color color, LEDs leds) {
    this.period = period;
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
    timer.restart();
    timer.start();
    ledsOn = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(period)) {
      AddressableLEDBuffer buffer = leds.getBuffer();
      ledsOn = !ledsOn;
      if (ledsOn) {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, color);
        }
      } else {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, LEDConstants.transparent);
        }
      }

      leds.updateBuffer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
