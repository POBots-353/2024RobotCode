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

  public Climber() {
    followerMotor.follow(mainMotor, true);
    SparkMaxUtil.configureFollower(followerMotor);
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
    SmartDashboard.putNumber("Climber Motor Temperature", mainMotor.getMotorTemperature());
    SmartDashboard.putNumber("Climber Motor Current", mainMotor.getOutputCurrent());
  }
}
