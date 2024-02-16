// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
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
import monologue.Logged;

public class Shooter extends VirtualSubsystem implements Logged {
  private CANSparkMax shooterMain =
      new CANSparkMax(ShooterConstants.shooterMainID, MotorType.kBrushless);
  private CANSparkMax shooterFollower =
      new CANSparkMax(ShooterConstants.shooterFollowerId, MotorType.kBrushless);

  private RelativeEncoder mainShooterEncoder = shooterMain.getEncoder();
  private RelativeEncoder followerEncoder = shooterFollower.getEncoder();

  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.shooterKs, ShooterConstants.shooterKv, ShooterConstants.shooterKa);

  private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.04, 0.02);

  private double filteredVelocity = 0.0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                shooterMain.setVoltage(volts.in(Units.Volts));
                shooterFollower.setVoltage(volts.in(Units.Volts));
              },
              null,
              this));

  private SparkPIDController mainPID = shooterMain.getPIDController();
  private SparkPIDController followerPID = shooterFollower.getPIDController();

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMain.setCANTimeout(100);
    shooterMain.restoreFactoryDefaults();
    shooterMain.setIdleMode(IdleMode.kCoast);
    shooterMain.setInverted(false);
    shooterMain.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);

    mainShooterEncoder.setAverageDepth(4);
    mainShooterEncoder.setMeasurementPeriod(32);

    shooterMain.setPeriodicFramePeriod(PeriodicFrame.kStatus3, SparkMaxUtil.disableFramePeriod);
    shooterMain.setPeriodicFramePeriod(PeriodicFrame.kStatus4, SparkMaxUtil.disableFramePeriod);
    shooterMain.setPeriodicFramePeriod(PeriodicFrame.kStatus5, SparkMaxUtil.disableFramePeriod);
    shooterMain.setPeriodicFramePeriod(PeriodicFrame.kStatus6, SparkMaxUtil.disableFramePeriod);
    shooterMain.setPeriodicFramePeriod(PeriodicFrame.kStatus7, SparkMaxUtil.disableFramePeriod);

    mainPID.setP(ShooterConstants.shooterP);
    mainPID.setOutputRange(0.0, 1.0);
    shooterMain.setCANTimeout(0);

    shooterFollower.setCANTimeout(100);
    shooterFollower.restoreFactoryDefaults();
    shooterFollower.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    shooterFollower.setIdleMode(IdleMode.kCoast);

    followerPID.setP(ShooterConstants.shooterP);
    followerPID.setOutputRange(0.0, 1.0);

    followerEncoder.setAverageDepth(4);
    followerEncoder.setMeasurementPeriod(32);

    SparkMaxUtil.configureFollower(shooterFollower);
    shooterFollower.setCANTimeout(0);
  }

  public void setMotorSpeed(double velocity) {
    double feedForward = shooterFeedforward.calculate(velocity);

    mainPID.setReference(velocity, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
    followerPID.setReference(velocity, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
  }

  public void stopMotor() {
    shooterMain.set(0.0);
    shooterFollower.set(0.0);
  }

  public double getVelocity() {
    return mainShooterEncoder.getVelocity();
    // return filteredVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    filteredVelocity = velocityFilter.calculate(mainShooterEncoder.getVelocity());

    // if (Math.abs(filteredVelocity) < 1e-5) {
    //   filteredVelocity = 0.0;
    // }

    SmartDashboard.putNumber("Shooter/Velocity Raw", mainShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/Velocity Filtered", filteredVelocity);
  }

  @Override
  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              REVLibError mainMotorError = shooterMain.getLastError();
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
        Commands.waitSeconds(10),
        Commands.runOnce(
            () -> {
              double velocityDifference =
                  Math.abs(getVelocity() - ShooterConstants.shooterVelocity);
              if (velocityDifference > ShooterConstants.velocityTolerance) {
                addError("Shooter Motor not near desired velocity");
              } else {
                addInfo("Shooter motors at desired velocity");
              }
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
