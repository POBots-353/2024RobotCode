// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Climber extends SubsystemBase {
  /** IDK man it's a climber */

  private DoubleSolenoid climberPiston = new DoubleSolenoid(IntakeConstants.pneumaticHubID,
      PneumaticsModuleType.CTREPCM,
      IntakeConstants.climberPistonForwardID, IntakeConstants.PistonReverseID);

  private DoubleSolenoid manipulatorBreak = new DoubleSolenoid(IntakeConstants.pneumaticHubID,
      PneumaticsModuleType.CTREPCM,
      IntakeConstants.manipulatorBreakForwardID, IntakeConstants.manipulatorBreakReverseID);

  private RelativeEncoder climberEncoder;

  public Climber() {
    climberEncoder.setPosition(IntakeConstants.startingConfigurationHeight); 
  }

  // Just put some functions in here I
  public void toggleClimber() {
    climberPiston.toggle();
  }
  public void climberTiltIn() {
    climberPiston.set(Value.kReverse);
  }
  public void climberTiltOut() {
    climberPiston.set(Value.kForward);
  }

  public void toggleOnManipulatorBreak() {
    manipulatorBreak.set(Value.kForward);
  }

  public void toggleOffManipulatorBreak() {
    manipulatorBreak.set(Value.kReverse);
  }

//Maybe use motors?
  public double getMotorCurrent() {
    return 0;
  }
  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

//There will likely be a climber position for 0
  public void zeroClimberPosition() {
    climberEncoder.setPosition(0);
  }

//I'm just guessing there might be a double solenoid?
  public DoubleSolenoid.Value getPistonState() {
    return climberPiston.get();
  }

  @Override
  public void periodic() {}
}