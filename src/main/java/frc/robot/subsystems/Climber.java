// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  /** Created Climber */
  private DoubleSolenoid climberPiston1 =
      new DoubleSolenoid(climberConstants.pneumaticHubID,PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID1, climberConstants.pistonReverseID1);

  private DoubleSolenoid climberPiston2 =
      new DoubleSolenoid(climberConstants.pneumaticHubID,PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID2, climberConstants.pistonReverseID2);

          CANSparkMax climberMotor1 = new CANSparkMax(climberConstants.climberMotorOneID, MotorType.kBrushless);
          CANSparkMax climberMotor2 = new CANSparkMax(climberConstants.climberMotorTwoID, MotorType.kBrushless);
          
  private RelativeEncoder climberEncoder1;
  private RelativeEncoder climberEncoder2;
  
  public Climber() {}
  // Methods are self explanatory (I made two in case we need two climbers)
  public void toggleClimber() {
    climberPiston1.toggle();
    climberPiston2.toggle();
  }
  public void climber1ExtendsUp() {
    climberPiston1.set(Value.kForward);
  }

  public void climber2ExtendsUp() {
    climberPiston1.set(Value.kForward);
  }

  public void climber1RestractDown() {
    climberPiston1.set(Value.kReverse);
  }

  public void climber2RestractDown() {
    climberPiston2.set(Value.kReverse);
  }

  public void setClimberMotor1Speed() {
  climberMotor1.set(climberConstants.climberMotorSpeed);
  }

  public void setClimberMotor2Speed() {
    climberMotor1.set(-climberConstants.climberMotorSpeed);
  }

  public void stopClimberMotor1() {
    climberMotor1.set(0.0);
  }
   public void stopClimberMotor2(){
    climberMotor2.set(0.0);
  }
  
  public double getMotor1Current() {
    return climberMotor1.getOutputCurrent();
  }
    public double getMotor2Current() {
    return climberMotor2.getOutputCurrent();
  }

  public double getClimberPiston1Position() {
    return climberEncoder1.getPosition();
  }
  public double getClimberPiston2Position(){
  return climberEncoder2.getPosition();
  }

  // Position 0 for all climber pistons
  public void zeroClimberPositionPiston() {
    climberEncoder1.setPosition(0);
    climberEncoder2.setPosition(0);
  }

  // Get Piston Positions
  public DoubleSolenoid.Value getPistonState1() {
    return climberPiston1.get();
    }
  public DoubleSolenoid.Value getPistonState2() {
    return climberPiston2.get();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber 1 position", climberEncoder1.getPosition());
    SmartDashboard.putNumber("Climber 2 position", climberEncoder2.getPosition()); 
    /* Posts the climber position to SmartDashboard */
    
    // Posts the climber motor temperatures/current
    SmartDashboard.putNumber("Climber Motor 1 Temperature", climberMotor1.getMotorTemperature());
    SmartDashboard.putNumber("Climber Motor 2 Temperature", climberMotor2.getMotorTemperature());
    SmartDashboard.putNumber("Climber Motor 1 Current", climberMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Climber Motor 2 Current", climberMotor2.getOutputCurrent());
  }
}