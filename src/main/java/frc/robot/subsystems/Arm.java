// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);

  private SparkPIDController armPIDController = armMotor.getPIDController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  private TrapezoidProfile armProfile = new TrapezoidProfile(ArmConstants.profileConstraints);

  @Log.NT
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          ArmConstants.armKg, ArmConstants.armKs, ArmConstants.armKv, ArmConstants.armKa);

  /** Creates a new Arm. */
  public Arm() {
    armEncoder.setPositionConversionFactor(ArmConstants.armPositionConversionFactor);
    armEncoder.setVelocityConversionFactor(ArmConstants.armVelocityConversionFactor);

    armPIDController.setP(ArmConstants.armKp);
    armPIDController.setI(ArmConstants.armKi);
    armPIDController.setD(ArmConstants.armKd);
  }

  public void setProfileState(TrapezoidProfile.State state) {
    double feedforward = armFeedforward.calculate(state.position, state.velocity);

    armPIDController.setReference(
        state.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
  }

  public void setPosition(double position) {
    TrapezoidProfile.State currentState = getCurrentState();
    TrapezoidProfile.State desiredState = new TrapezoidProfile.State(position, 0.0);

    setProfileState(armProfile.calculate(0.020, currentState, desiredState));
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runj
  }
}
