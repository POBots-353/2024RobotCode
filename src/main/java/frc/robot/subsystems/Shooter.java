// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.SparkMaxUtil;
import monologue.Annotations.Log;
import monologue.Logged;

public class Shooter extends VirtualSubsystem implements Logged {
  private CANSparkMax bottomShooter =
      new CANSparkMax(ShooterConstants.bottomShooterID, MotorType.kBrushless);
  private CANSparkMax topShooter =
      new CANSparkMax(ShooterConstants.topShooterID, MotorType.kBrushless);

  private RelativeEncoder bottomEncoder = bottomShooter.getEncoder();
  private RelativeEncoder topEncoder = topShooter.getEncoder();

  private SimpleMotorFeedforward topFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.topShooterKs,
          ShooterConstants.topShooterKv,
          ShooterConstants.topShooterKa);

  private SimpleMotorFeedforward bottomFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.bottomShooterKs,
          ShooterConstants.bottomShooterKv,
          ShooterConstants.bottomShooterKa);

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                bottomShooter.setVoltage(volts.in(Volts));
                topShooter.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("Top")
                    .angularVelocity(Rotations.per(Minute).of(topEncoder.getVelocity()))
                    .angularPosition(Rotations.of(topEncoder.getPosition()));
                log.motor("Bottom")
                    .angularVelocity(Rotations.per(Minute).of(bottomEncoder.getVelocity()))
                    .angularPosition(Rotations.of(bottomEncoder.getPosition()));
              },
              this));

  private SparkPIDController bottomPID = bottomShooter.getPIDController();
  private SparkPIDController topPID = topShooter.getPIDController();

  private double velocitySetpoint = 0.0;

  /** Creates a new Shooter. */
  public Shooter() {
    bottomShooter.setCANTimeout(100);
    bottomShooter.restoreFactoryDefaults();
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.setInverted(false);
    bottomShooter.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    bottomShooter.enableVoltageCompensation(ShooterConstants.voltageCompensation);

    bottomEncoder.setAverageDepth(4);
    bottomEncoder.setMeasurementPeriod(32);

    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, SparkMaxUtil.disableFramePeriod);
    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, SparkMaxUtil.disableFramePeriod);
    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus5, SparkMaxUtil.disableFramePeriod);
    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus6, SparkMaxUtil.disableFramePeriod);
    bottomShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus7, SparkMaxUtil.disableFramePeriod);

    bottomPID.setP(ShooterConstants.shooterKp);
    bottomPID.setOutputRange(0.0, 1.0);
    bottomShooter.setCANTimeout(0);

    topShooter.setCANTimeout(100);
    topShooter.restoreFactoryDefaults();
    topShooter.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    topShooter.enableVoltageCompensation(ShooterConstants.voltageCompensation);
    topShooter.setIdleMode(IdleMode.kCoast);

    topPID.setP(ShooterConstants.shooterKp);
    topPID.setOutputRange(0.0, 1.0);

    topEncoder.setAverageDepth(4);
    topEncoder.setMeasurementPeriod(32);

    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, SparkMaxUtil.disableFramePeriod);
    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus4, SparkMaxUtil.disableFramePeriod);
    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus5, SparkMaxUtil.disableFramePeriod);
    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus6, SparkMaxUtil.disableFramePeriod);
    topShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus7, SparkMaxUtil.disableFramePeriod);
    topShooter.setCANTimeout(0);
  }

  public void setMotorSpeed(double velocity) {
    double topFF = topFeedforward.calculate(velocity);
    double bottomFF = bottomFeedforward.calculate(velocity);

    bottomPID.setReference(velocity, ControlType.kVelocity, 0, bottomFF, ArbFFUnits.kVoltage);
    topPID.setReference(velocity, ControlType.kVelocity, 0, topFF, ArbFFUnits.kVoltage);

    velocitySetpoint = velocity;
  }

  public void setMotorSpeedDifferential(double topVelocity, double bottomVelocity) {
    double topFF = topFeedforward.calculate(topVelocity);
    double bottomFF = bottomFeedforward.calculate(bottomVelocity);

    topPID.setReference(topVelocity, ControlType.kVelocity, 0, topFF, ArbFFUnits.kVoltage);
    bottomPID.setReference(bottomVelocity, ControlType.kVelocity, 0, bottomFF, ArbFFUnits.kVoltage);
  }

  public void stopMotor() {
    bottomShooter.set(0.0);
    topShooter.set(0.0);
  }

  @Log.NT(key = "Top Velocity")
  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  @Log.NT(key = "Bottom Velocity")
  public double getBottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Bottom Velocity", getBottomVelocity());
    SmartDashboard.putNumber("Shooter/Top Velocity", getTopVelocity());
    SmartDashboard.putNumber("Shooter/Velocity Difference", getTopVelocity() - getBottomVelocity());
    SmartDashboard.putBoolean(
        "Shooter/At Setpoint",
        velocitySetpoint - getBottomVelocity() < ShooterConstants.velocityTolerance);
  }

  @Override
  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              REVLibError mainMotorError = bottomShooter.getLastError();
              if (mainMotorError != REVLibError.kOk) {
                addError("Main Shooter motor error: " + mainMotorError.name());
              } else {
                addInfo("Main Shooter motor contains no errors");
              }
            }),
        // checks both shooter motors
        Commands.runOnce(
            () -> {
              joystick.setButton(OperatorConstants.shootButton, true);
            }),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () -> {
              double velocityDifference =
                  Math.abs(getBottomVelocity() - ShooterConstants.shooterVelocity);
              if (velocityDifference > ShooterConstants.velocityTolerance) {
                addError("Shooter Motor not near desired velocity");
              } else {
                addInfo("Shooter motors at desired velocity");
              }

              joystick.clearVirtualButtons();
            }));
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
