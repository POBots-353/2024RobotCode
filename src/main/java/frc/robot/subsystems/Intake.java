// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.revrobotics.RelativeEncoder;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends VirtualSubsystem implements Logged {
  /** Creates a new Intake. */
  CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.intakeMotorOneID, MotorType.kBrushless);
  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  DigitalInput irBreakBeam = new DigitalInput(IntakeConstants.intakeMotorOneID);

  private final double prematchDelay = 2.5;

  public Intake() {}

  @Log.NT(key = "IR Break Beam State")
  public boolean isIRBeamBreakBroken() {
    return !irBreakBeam.get();
  }

  public void feedToShooter() {
    intakeMotor.set(IntakeConstants.intakeMotorSpeed);
  }

  public void outtakeNoteInIntake() {
    intakeMotor.set(-IntakeConstants.intakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0.0);
  }

  public void intake() {
    if (!isIRBeamBreakBroken()) {
      feedToShooter();
    } else {
    stopIntakeMotor();
    }
  }

  @Log.NT(key = "Intake Motor Velocity")
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Note?", isIRBeamBreakBroken());
    SmartDashboard.putNumber("Intake Motor Velocity", getVelocity());
  }

  @Override 
  public Command getPrematchCheckCommand(VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
      // Check for hardware errors
      Commands.runOnce(
        () -> {
              REVLibError error = intakeMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Intake motor error: " + error.name());
              } else {
                addInfo("Intake motor contains no errors");
              }
            }),
      // Checks Intake Motor
      Commands.runOnce(
        () -> {
          joystick.setButton(OperatorConstants.intakeNoteButton, true);
        }),
      Commands.waitSeconds(prematchDelay),
      Commands.runOnce(
        () -> {
          if (getVelocity() == 0) {
            addError("Intake Motor isn't working");
          } else {
            addInfo("Intake Motor is moving");
          }
          joystick.clearVirtualButtons();
        }),
      Commands.waitSeconds(0.5),
      // Checks IR Beam Break functionality
      Commands.runOnce(
        () -> {
          joystick.setButton(OperatorConstants.intakeNoteButton, true);
        }),
      Commands.waitSeconds(prematchDelay),
      Commands.runOnce(
        () -> {
          if (isIRBeamBreakBroken() && getVelocity() != 0) {
            addError("IR Break Beam isn't stopping the motor");
          }
          else {
            addInfo("IR Break Beak is functioning");
          }
        }),
      Commands.runOnce(
        () -> {
          joystick.clearVirtualButtons();
        }));
  }
}
