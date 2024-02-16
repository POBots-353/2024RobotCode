// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.SparkMaxUtil;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends VirtualSubsystem implements Logged {
  /** Creates a new Intake. */
  private CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private DigitalInput irBreakBeam = new DigitalInput(IntakeConstants.beamBreakID);

  private final double prematchDelay = 2.5;

  public Intake() {
    intakeMotor.setCANTimeout(100);
    for (int i = 0; i < 5; i++) {
      intakeMotor.setInverted(true);
      intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeCurrentLimit);
      intakeMotor.setIdleMode(IdleMode.kBrake);

      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, SparkMaxUtil.disableFramePeriod);
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, SparkMaxUtil.disableFramePeriod);
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, SparkMaxUtil.disableFramePeriod);
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, SparkMaxUtil.disableFramePeriod);
      intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, SparkMaxUtil.disableFramePeriod);

      if (intakeMotor.getLastError() == REVLibError.kOk) {
        break;
      }
    }
    intakeMotor.setCANTimeout(0);
  }

  @Log.NT(key = "Break Broken")
  public boolean beamBroken() {
    return !irBreakBeam.get();
  }

  public Command intakeUntilBeamBreak() {
    return run(this::intake).until(this::beamBroken).finallyDo(this::stopIntakeMotor);
  }

  public Command autoFeedToShooter() {
    return run(this::feedToShooter).until(() -> !beamBroken());
  }

  public void feedToShooter() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void outtakeNoteInIntake() {
    intakeMotor.set(-IntakeConstants.intakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(0.0);
  }

  public void intake() {
    if (!beamBroken()) {
      intakeMotor.set(IntakeConstants.intakeSpeed);
    } else {
      stopIntakeMotor();
    }
  }

  @Log.NT(key = "Velocity")
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
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
              if (Math.abs(intakeEncoder.getVelocity()) <= 1e-4) {
                addError("Intake Motor is not moving");
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
              if (beamBroken() && getVelocity() != 0) {
                addError("IR Break Beam isn't stopping the motor");
              } else {
                addInfo("IR Break Beak is functioning");
              }
            }),
        Commands.runOnce(
            () -> {
              joystick.clearVirtualButtons();
            }));
  }
}
