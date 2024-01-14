// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);

  private SparkPIDController armPIDController = armMotor.getPIDController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();
  private DutyCycleEncoder absoluteEncoder =
      new DutyCycleEncoder(ArmConstants.armAbsoluteEncoderID);

  private TrapezoidProfile armProfile = new TrapezoidProfile(ArmConstants.profileConstraints);

  @Log.NT
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          ArmConstants.armKg, ArmConstants.armKs, ArmConstants.armKv, ArmConstants.armKa);

  private SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                armMotor.setVoltage(volts.in(Volts));
              },
              null,
              this));

  /** Creates a new Arm. */
  public Arm() {
    absoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    armEncoder.setPositionConversionFactor(ArmConstants.armPositionConversionFactor);
    armEncoder.setVelocityConversionFactor(ArmConstants.armVelocityConversionFactor);

    armPIDController.setP(ArmConstants.armKp);
    armPIDController.setI(ArmConstants.armKi);
    armPIDController.setD(ArmConstants.armKd);
  }

  public Command moveToPosition(double position) {
    return run(() -> setPosition(position))
        .until(() -> Math.abs(armEncoder.getPosition() - position) < Units.degreesToRadians(0.5));
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Absolute Position", absoluteEncoder.getAbsolutePosition());

    SmartDashboard.putNumber("Arm/Velocity", armEncoder.getVelocity());
  }

  public Command quasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command quasistaticBackward() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command dynamicForward() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command dynamicBackward() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }
}
