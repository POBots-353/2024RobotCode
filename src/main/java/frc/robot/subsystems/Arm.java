// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.SparkMaxUtil;
import monologue.Annotations.Log;
import monologue.Logged;

public class Arm extends VirtualSubsystem implements Logged {
  private CANSparkMax mainMotor = new CANSparkMax(ArmConstants.mainMotorID, MotorType.kBrushless);
  private CANSparkMax followerMotor =
      new CANSparkMax(ArmConstants.followerID, MotorType.kBrushless);

  private SparkPIDController armPIDController = mainMotor.getPIDController();
  private RelativeEncoder armEncoder = mainMotor.getEncoder();
  private SparkAbsoluteEncoder absoluteEncoder =
      mainMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private TrapezoidProfile armProfile = new TrapezoidProfile(ArmConstants.profileConstraints);
  private PIDController pidController =
      new PIDController(ArmConstants.armKp, ArmConstants.armKi, ArmConstants.armKd);

  private Alert absolutePositionNotSet =
      new Alert("Arm failed to set to absolute position", AlertType.ERROR);

  private final double prematchDelay = 2.5;
  private final double prematchAngleTolerance = Units.degreesToRadians(1.5);

  private TrapezoidProfile.State previousSetpoint = new TrapezoidProfile.State();

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
                followerMotor.setVoltage(volts.in(Volts));
              },
              null,
              this));

  /** Creates a new Arm. */
  public Arm() {
    configureMainMotor();
    configureFollowerMotor();
    configureAbsoluteEncoder();

    Commands.sequence(Commands.waitSeconds(1.0), Commands.runOnce(this::resetToAbsolute))
        .ignoringDisable(true)
        .schedule();
  }

  private void configureMainMotor() {
    mainMotor.setCANTimeout(100);
    mainMotor.setInverted(ArmConstants.mainMotorInverted);

    armPIDController.setP(ArmConstants.armKp);
    armPIDController.setI(ArmConstants.armKi);
    armPIDController.setD(ArmConstants.armKd);
    armPIDController.setOutputRange(-1.0, 1.0);

    armPIDController.setFeedbackDevice(armEncoder);

    armEncoder.setPositionConversionFactor(ArmConstants.armPositionConversionFactor);
    armEncoder.setVelocityConversionFactor(ArmConstants.armVelocityConversionFactor);

    mainMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
    mainMotor.enableVoltageCompensation(12.3);

    mainMotor.setIdleMode(IdleMode.kBrake);

    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // Absolute encoder position
    mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // Absolute encoder velocity

    mainMotor.setCANTimeout(0);
  }

  private void configureFollowerMotor() {
    followerMotor.setCANTimeout(100);
    followerMotor.restoreFactoryDefaults();
    followerMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setInverted(!ArmConstants.mainMotorInverted);
    followerMotor.enableVoltageCompensation(12.3);

    SparkMaxUtil.configureFollower(followerMotor);
    followerMotor.setCANTimeout(0);
  }

  private void configureAbsoluteEncoder() {
    absoluteEncoder.setZeroOffset(0.0);
    absoluteEncoder.setInverted(ArmConstants.absoluteEncoderInverted);
    absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);
  }

  private void resetToAbsolute() {
    mainMotor.setCANTimeout(100);
    double position = absoluteEncoder.getPosition() - ArmConstants.absoluteOffset.getRadians();

    boolean failed = true;
    for (int i = 0; i < 5; i++) {
      if (armEncoder.setPosition(position) == REVLibError.kOk) {
        failed = false;
      }
      if (armEncoder.getPosition() == position) {
        break;
      }
      Timer.delay(0.010);
    }

    if (failed) {
      DriverStation.reportError("Failed to set absolute posiiton of arm motor", false);
      DataLogManager.log("Failed to set absolute posiiton of arm motor");
      absolutePositionNotSet.set(true);
    } else {
      absolutePositionNotSet.set(false);
    }
    mainMotor.setCANTimeout(0);
  }

  public Command moveToPosition(Rotation2d position) {
    return run(() -> setDesiredPosition(position))
        .until(
            () ->
                Math.abs(armEncoder.getPosition() - position.getRadians())
                    <= ArmConstants.angleTolerance)
        .finallyDo(() -> setSpeed(0.0));
  }

  public void setProfileState(TrapezoidProfile.State state) {
    double feedforward =
        armFeedforward.calculate(
            state.position, state.velocity, (state.velocity - previousSetpoint.velocity) / 0.020);

    previousSetpoint = state;

    log("Feedforward Voltage", feedforward);

    double pidOutput = pidController.calculate(getPosition().getRadians(), state.position);

    mainMotor.setVoltage(pidOutput * mainMotor.getBusVoltage() + feedforward);
    followerMotor.setVoltage(pidOutput * mainMotor.getBusVoltage() + feedforward);
  }

  public void setDesiredPosition(Rotation2d position) {
    TrapezoidProfile.State currentState = previousSetpoint;
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(position.getRadians(), 0.0);

    TrapezoidProfile.State setpoint = armProfile.calculate(0.020, currentState, goalState);

    // Replan profile if it's too far from position or if it's finished
    if (Math.abs(setpoint.position - armEncoder.getPosition()) > ArmConstants.replanningError
        || armProfile.isFinished(0.0)) {
      setpoint = armProfile.calculate(0.020, getCurrentState(), goalState);
    }

    setProfileState(setpoint);
  }

  public void setSpeed(double speed) {
    mainMotor.set(speed);
    followerMotor.set(speed);

    previousSetpoint = getCurrentState();
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
  }

  @Log.NT(key = "Absolute Angle")
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRadians(absoluteEncoder.getPosition());
  }

  @Log.NT(key = "Angle")
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(armEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Position", Units.radiansToDegrees(armEncoder.getPosition()));
    SmartDashboard.putNumber("Arm/Absolute Position", getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber(
        "Arm/Absolute Encoder Position", Units.radiansToDegrees(absoluteEncoder.getPosition()));

    SmartDashboard.putNumber("Arm/Velocity", Units.radiansToDegrees(armEncoder.getVelocity()));
    SmartDashboard.putNumber(
        "Arm/Absolute Encoder Velocity", Units.radiansToDegrees(absoluteEncoder.getVelocity()));

    SmartDashboard.putNumber(
        "Arm/Position Setpoint", Units.radiansToDegrees(previousSetpoint.position));
    SmartDashboard.putNumber(
        "Arm/Velocity Setpoint", Units.radiansToDegrees(previousSetpoint.velocity));

    SmartDashboard.putNumber(
        "Arm/Position Error",
        Units.radiansToDegrees(previousSetpoint.position - armEncoder.getPosition()));
    SmartDashboard.putNumber(
        "Arm/Velocity Error",
        Units.radiansToDegrees(previousSetpoint.velocity - armEncoder.getVelocity()));
  }

  @Override
  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
        // Check for hardware errors
        Commands.runOnce(
            () -> {
              REVLibError error = mainMotor.getLastError();
              if (error != REVLibError.kOk) {
                addError("Arm motor error: " + error.name());
              } else {
                addInfo("Main arm motor contains no errors");
              }
            }),
        // Move to pickup height
        Commands.runOnce(
            () -> {
              joystick.setButton(OperatorConstants.armToPickup, true);
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(armEncoder.getPosition() - ArmConstants.pickupAngle.getRadians())
                  > prematchAngleTolerance) {
                addError("Arm did not sufficiently reach pickup position");
              } else {
                addInfo("Arm successfully reached ground pickup position");
              }
              joystick.clearVirtualButtons();
            }),
        Commands.waitSeconds(0.5),
        // Move to amp height
        Commands.runOnce(
            () -> {
              joystick.setButton(OperatorConstants.armToAmp, true);
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(armEncoder.getPosition() - ArmConstants.ampAngle.getRadians())
                  > prematchAngleTolerance) {
                addError("Arm did not sufficiently reach amp position");
              } else {
                addInfo("Arm successfully reached amp position");
              }
              joystick.clearVirtualButtons();
            }),
        Commands.waitSeconds(0.5),
        // Move to subwoofer shooting angle
        Commands.runOnce(
            () -> {
              joystick.setButton(OperatorConstants.armToSubwoofer, true);
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(armEncoder.getPosition() - ArmConstants.subwooferAngle.getRadians())
                  > prematchAngleTolerance) {
                addError("Arm did not sufficiently reach subwoofer position");
              } else {
                addInfo("Arm successfully reached subwoofer position");
              }
              joystick.clearVirtualButtons();
            }),
        Commands.waitSeconds(0.5),
        // Move to podium shooting angle
        Commands.runOnce(
            () -> {
              joystick.setButton(OperatorConstants.armToPodium, true);
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(armEncoder.getPosition() - ArmConstants.podiumAngle.getRadians())
                  > prematchAngleTolerance) {
                addError("Arm did not sufficiently reach podium shooting angle");
              } else {
                addInfo("Arm successfully reached podium shooting angle");
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
