// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.shooterKs, ShooterConstants.shooterKv, ShooterConstants.shooterKa);
  private PIDController velocityPIDController =
      new PIDController(ShooterConstants.shooterP, 0.0, 0.0);

  private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.08, 0.02);

  private double filteredVelocity = 0.0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> shooterMain.setVoltage(volts.in(Units.Volts)), null, this));

  private SparkPIDController shooterPID = shooterMain.getPIDController();

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMain.setCANTimeout(100);
    shooterMain.restoreFactoryDefaults();
    shooterMain.setIdleMode(IdleMode.kCoast);
    shooterMain.setInverted(false);
    shooterMain.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    mainShooterEncoder.setAverageDepth(4);

    shooterPID.setP(ShooterConstants.shooterP);
    shooterPID.setOutputRange(0.0, 1.0);
    shooterMain.setCANTimeout(0);

    shooterFollower.setCANTimeout(100);
    shooterFollower.restoreFactoryDefaults();
    shooterFollower.setSmartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    shooterFollower.setIdleMode(IdleMode.kCoast);

    SparkMaxUtil.configureFollower(shooterFollower);
    shooterFollower.setCANTimeout(0);
  }

  public void setMotorSpeed(double velocity) {
    double feedForward = shooterFeedforward.calculate(velocity);
    double pidOutput = velocityPIDController.calculate(filteredVelocity, velocity);
    pidOutput = MathUtil.clamp(pidOutput, 0.0, 1.0);

    shooterMain.setVoltage(pidOutput * 12.0 + feedForward);
    shooterFollower.setVoltage(pidOutput * 12.0 + feedForward);
  }

  public void stopMotor() {
    shooterMain.set(0);
  }

  public double getVelocity() {
    return filteredVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    filteredVelocity = velocityFilter.calculate(mainShooterEncoder.getVelocity());

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
