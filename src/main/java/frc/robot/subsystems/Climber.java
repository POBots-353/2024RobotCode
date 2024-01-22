// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.SparkMaxUtil;

public class Climber extends SubsystemBase {
  private CANSparkMax mainMotor =
      new CANSparkMax(ClimberConstants.mainMotorID, MotorType.kBrushless);
  private CANSparkMax followerMotor =
      new CANSparkMax(ClimberConstants.followerMotorID, MotorType.kBrushless);

  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private final double prematchDelay = 2.5;

  public Climber() {
    followerMotor.follow(mainMotor, true);
    SparkMaxUtil.configureFollower(followerMotor);
  }

public double getVelocity() {
  return climberEncoder.getVelocity();
}
  public void setClimberUp() {
    mainMotor.set(ClimberConstants.climberMotorSpeed);
  }

  public void setClimberDown() {
    mainMotor.set(-ClimberConstants.climberMotorSpeed);
  }

  public void stopClimberMotors() {
    mainMotor.set(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Temperature", mainMotor.getMotorTemperature());
    SmartDashboard.putNumber("Climber/Current", mainMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/Applied Output", mainMotor.getAppliedOutput());
  }
}

@Override 
public Command getPrematchCheckCommand(
  VirtualXboxController controller, VirtualJoystick joystick) {
 return  Commands.sequence(
  // Check for hardware motors
  Commands.runOnce(
  () -> {
    REVLibError error = climberMotor.getLastError();
    if (error != REVLibError.kOk) {
      addError("Climber motor error: " + error.name());
    } else {
      addInfo("Climber motor contains no errors"); 
    }

  }
  )
 )},
 // Checks climber motor 
 Commands.runOnce(
  () -> {
    joystick.setButton(operatorConstants.climberUpButton, true);
  }),
  Commands.waitSeconds(prematchDelay),
  Commands.runOnce(
    () -> {
      if (getVelocity() < 10) {
        addError("Climber motor isn't working");
      } else {
        addInfo("Climber motor is moving");
      }
      joystick.clearVirtualButtons();
    }),
    Commands.waitSeconds(0.5),

    Commands.runOnce(
      () -> {
        joystick.setButton(operatorConstants.climberDownButton, true);
      }),
      Commands.waitSeconds(prematchDelay),
      Commands.runOnce(
      () -> {
        if(getVelocity() > -10) {
          addError("Climber motor isn't working");
        } else {
          addInfo("Climber motor is moving");
        }
        joystick.clearVirtualButtons();
      }
    )
