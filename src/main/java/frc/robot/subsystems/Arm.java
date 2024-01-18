// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.SparkMaxUtil;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends SubsystemBase implements Logged {
  private CANSparkMax mainMotor = new CANSparkMax(ArmConstants.mainMotorID, MotorType.kBrushless);
  private CANSparkMax followerMotor =
      new CANSparkMax(ArmConstants.followerID, MotorType.kBrushless);

  private SparkPIDController armPIDController = mainMotor.getPIDController();
  private RelativeEncoder armEncoder = mainMotor.getEncoder();
  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.absoluteEncoderID);

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
                mainMotor.setVoltage(volts.in(Volts));
              },
              null,
              this));

  /** Creates a new Arm. */
  public Arm() {
    absoluteEncoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);

    configureMainMotor();
    configureFollowerMotor();

    Commands.sequence(Commands.waitSeconds(1.0), Commands.runOnce(this::resetToAbsolute))
        .ignoringDisable(true)
        .schedule();
  }

  private void configureMainMotor() {
    mainMotor.setCANTimeout(250);
    armEncoder.setPositionConversionFactor(ArmConstants.armPositionConversionFactor);
    armEncoder.setVelocityConversionFactor(ArmConstants.armVelocityConversionFactor);

    armPIDController.setP(ArmConstants.armKp);
    armPIDController.setI(ArmConstants.armKi);
    armPIDController.setD(ArmConstants.armKd);
    armPIDController.setOutputRange(-1.0, 1.0);

    mainMotor.setIdleMode(IdleMode.kBrake);
    mainMotor.setCANTimeout(0);
  }

  private void configureFollowerMotor() {
    followerMotor.setCANTimeout(250);
    followerMotor.follow(mainMotor, true);
    SparkMaxUtil.configureFollower(followerMotor);
    followerMotor.setCANTimeout(0);
  }

  private void resetToAbsolute() {
    mainMotor.setCANTimeout(250);
    double position = getAbsoluteAngle().minus(ArmConstants.absoluteOffset).getRadians();

    boolean failed = true;
    for (int i = 0; i < 250; i++) {
      if (armEncoder.setPosition(position) == REVLibError.kOk) {
        failed = false;
      }
      if (armEncoder.getPosition() == position) {
        break;
      }
    }

    if (failed) {
      DriverStation.reportError("Failed to set absolute posiiton of arm motor", false);
      DataLogManager.log("Failed to set absolute posiiton of arm motor");
    }
    mainMotor.setCANTimeout(0);
  }

  public Command moveToPosition(Rotation2d position) {
    return run(() -> setDesiredPosition(position))
        .until(
            () ->
                Math.abs(armEncoder.getPosition() - position.getRadians())
                    < Units.degreesToRadians(0.5));
  }

  public void setProfileState(TrapezoidProfile.State state) {
    double feedforward = armFeedforward.calculate(state.position, state.velocity);

    armPIDController.setReference(
        state.position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kVoltage);
  }

  public void setDesiredPosition(Rotation2d position) {
    TrapezoidProfile.State currentState = getCurrentState();
    TrapezoidProfile.State desiredState = new TrapezoidProfile.State(position.getRadians(), 0.0);

    setProfileState(armProfile.calculate(0.020, currentState, desiredState));
  }

  public void setSpeed(double speed) {
    mainMotor.set(speed);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
  }

  @Log.NT(key = "Absolute Angle")
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition());
  }

  @Log.NT(key = "Angle")
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(armEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Position", Units.radiansToDegrees(armEncoder.getPosition()));
    SmartDashboard.putNumber(
        "Arm/Absolute Position", Units.radiansToDegrees(absoluteEncoder.getAbsolutePosition()));

    SmartDashboard.putNumber("Arm/Velocity", Units.radiansToDegrees(armEncoder.getVelocity()));
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
