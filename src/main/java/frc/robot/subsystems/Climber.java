// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climberConstants;

public class Climber extends SubsystemBase {
  /** Created Climber */
  private DoubleSolenoid climberPiston1 =
      new DoubleSolenoid(
          climberConstants.pneumaticHubID,
          PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID1,
          climberConstants.pistonReverseID1);

  private DoubleSolenoid climberPiston2 =
      new DoubleSolenoid(
          climberConstants.pneumaticHubID,
          PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID2,
          climberConstants.pistonReverseID2);

  private DoubleSolenoid climberPiston3 =
      new DoubleSolenoid(
          climberConstants.pneumaticHubID,
          PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID3,
          climberConstants.pistonReverseID3);

  private DoubleSolenoid climberPiston4 =
      new DoubleSolenoid(
          climberConstants.pneumaticHubID,
          PneumaticsModuleType.CTREPCM,
          climberConstants.climberPistonForwardID4,
          climberConstants.pistonReverseID4);

  private RelativeEncoder climberEncoder1;
  private RelativeEncoder climberEncoder2;
  private RelativeEncoder climberEncoder3;
  private RelativeEncoder climberEncoder4;

  public Climber() {}

  // Maybe methods
  public void toggleClimber() {
    climberPiston1.toggle();
    climberPiston2.toggle();
  }

  public void climberTiltIn() {
    climberPiston1.set(Value.kReverse);
    climberPiston2.set(Value.kReverse);
  }

  public void climberTiltOut() {
    climberPiston1.set(Value.kForward);
    climberPiston2.set(Value.kForward);
  }

  public void climberExtendsUp() {
    climberPiston3.set(Value.kForward);
    climberPiston4.set(Value.kForward);
  }

  public void climberRestractDown() {
    climberPiston3.set(Value.kForward);
    climberPiston4.set(Value.kForward);
  }

  // I don't know if there might be motors
  public double getMotorCurrent() {
    return 0;
  }

  public double getClimberPosition() {
    return climberEncoder1.getPosition();
  }

  // Position 0 for all climber pistons
  public void zeroClimberPositionTilt() {
    climberEncoder1.setPosition(0);
    climberEncoder2.setPosition(0);
  }

  public void zeroClimberPositionExtend() {
    climberEncoder3.setPosition(0);
    climberEncoder4.setPosition(0);
  }

  // Get Piston Positions
  public DoubleSolenoid.Value[] getPistonState() {
    return new DoubleSolenoid.Value[] {
      climberPiston1.get(), climberPiston2.get(), climberPiston3.get(), climberPiston4.get()
    };
  }

  // add teleop stuff here
  @Override
  public void periodic() {}
}
