// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.SparkMaxUtil;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterMain = new CANSparkMax(ShooterConstants.shooterMainId, MotorType.kBrushless);
  private CANSparkMax shooterFollower = new CANSparkMax(ShooterConstants.shooterFollowerId, MotorType.kBrushless);
  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.shooterKs, ShooterConstants.shooterKv, ShooterConstants.shooterKa);

  private SparkPIDController shooterPID = shooterMain.getPIDController();

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMain.restoreFactoryDefaults();
    shooterFollower.follow(shooterMain, true);
    shooterMain.setIdleMode(IdleMode.kCoast);
    SparkMaxUtil.configureFollower(shooterFollower);

    shooterPID.setP(ShooterConstants.shooterP);
  }

  public void setMotorSpeed(double velocity) {
    double feedForward = shooterFeedforward.calculate(velocity);

    shooterPID.setReference(velocity, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
