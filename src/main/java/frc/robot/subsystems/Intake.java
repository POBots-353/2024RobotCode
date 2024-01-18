// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intakeMotor1 =
      new CANSparkMax(IntakeConstants.intakeMotorOneID, MotorType.kBrushless);

  DigitalInput irBreakBeam = new DigitalInput(IntakeConstants.intakeMotorOneID);

  public Intake() {}

  public void feedToShooter() {
    intakeMotor1.set(IntakeConstants.intakeMotorSpeed);
  }

  public void outtakeNoteInIntake() {
    intakeMotor1.set(-IntakeConstants.intakeMotorSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor1.set(0.0);
  }

  public void intake() {
    if (irBreakBeam.get()) {
      feedToShooter();
    }
    stopIntakeMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Note?", !irBreakBeam.get());
  }
}
