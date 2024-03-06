// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  private AddressableLED addressableLED = new AddressableLED(LEDConstants.ledPort);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.bufferLength);

  /** Creates a new LEDs. */
  public LEDs() {
    DataLogManager.log("[LEDs] Initializing");
    addressableLED.setLength(buffer.getLength());
    addressableLED.setData(buffer);
    addressableLED.start();
    DataLogManager.log("[LEDs] Initialization Complete");
  }

  public AddressableLEDBuffer getBuffer() {
    return buffer;
  }

  public void updateBuffer() {
    addressableLED.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
