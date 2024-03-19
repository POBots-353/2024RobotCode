// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.FaultLogger;
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

  private PIDController holdPIDController =
      new PIDController(ArmConstants.holdKp, ArmConstants.holdKi, ArmConstants.holdKd);

  // private Alert absolutePositionNotSet =
  //     new Alert("Arm failed to set to absolute position", AlertType.ERROR);

  private final double prematchDelay = 1.5;
  private final double prematchAngleTolerance = Units.degreesToRadians(0.5);

  private Debouncer setpointDebouncer = new Debouncer(ArmConstants.movementDebounceTime);
  private Debouncer autoDebouncer = new Debouncer(ArmConstants.autoMovementDebounceTime);

  private TrapezoidProfile.State previousSetpoint = new TrapezoidProfile.State();

  private final double mechanismVisualizationWidth = Units.inchesToMeters(50.0);
  private final double mechanismVisualizationHeight = Units.inchesToMeters(40.0);
  private final double lineWidth = 5.0;

  private Mechanism2d angleVisualizer =
      new Mechanism2d(mechanismVisualizationWidth, mechanismVisualizationHeight);

  private Mechanism2d setpointVisualizer =
      new Mechanism2d(mechanismVisualizationWidth, mechanismVisualizationHeight);

  private MechanismLigament2d currentAngleLigament;
  private MechanismLigament2d setpointLigament;

  @Log.NT
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          ArmConstants.armKg, ArmConstants.armKs, ArmConstants.armKv, ArmConstants.armKa);

  private final DCMotor armGearbox = DCMotor.getNEO(2);

  private SingleJointedArmSim armSimulation =
      new SingleJointedArmSim(
          armGearbox,
          1.0 / ArmConstants.armGearRatio,
          SingleJointedArmSim.estimateMOI(ArmConstants.armLength, ArmConstants.armMass),
          ArmConstants.armLength,
          ArmConstants.reverseMovementLimitAngle,
          ArmConstants.forwardMovementLimitAngle,
          true,
          0.0,
          VecBuilder.fill(Units.degreesToRadians(0.0025)));

  private final Pose3d origin =
      new Pose3d(Units.inchesToMeters(6.22), 0.0, Units.inchesToMeters(12.13), new Rotation3d());

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
    DataLogManager.log("[Arm] Initializing");
    configureMainMotor();
    configureFollowerMotor();
    configureAbsoluteEncoder();

    mainMotor.setCANTimeout(100);
    for (int i = 0; i < 5; i++) {
      if (mainMotor.getInverted() == ArmConstants.mainMotorInverted) {
        break;
      }

      mainMotor.setInverted(ArmConstants.mainMotorInverted);

      Timer.delay(0.005);
    }

    for (int i = 0; i < 5; i++) {
      if (absoluteEncoder.getInverted() == ArmConstants.absoluteEncoderInverted) {
        break;
      }

      absoluteEncoder.setInverted(ArmConstants.absoluteEncoderInverted);

      Timer.delay(0.005);
    }
    mainMotor.setCANTimeout(0);

    Commands.sequence(Commands.waitSeconds(1.0), Commands.runOnce(this::resetToAbsolute))
        .ignoringDisable(true)
        .schedule();

    FaultLogger.register(mainMotor);
    FaultLogger.register(followerMotor);

    REVPhysicsSim.getInstance().addSparkMax(mainMotor, armGearbox);

    MechanismRoot2d measuredRoot =
        angleVisualizer.getRoot(
            "Measured",
            mechanismVisualizationWidth / 2 - ArmConstants.armPivotX,
            ArmConstants.armPivotZ);
    MechanismRoot2d setpointRoot =
        setpointVisualizer.getRoot(
            "Setpoint",
            mechanismVisualizationWidth / 2 - ArmConstants.armPivotX,
            ArmConstants.armPivotZ);

    currentAngleLigament =
        measuredRoot.append(
            new MechanismLigament2d(
                "Arm Measured", ArmConstants.armLength, 0.0, lineWidth, new Color8Bit(Color.kRed)));

    setpointLigament =
        setpointRoot.append(
            new MechanismLigament2d(
                "Arm Setpoint",
                ArmConstants.armLength,
                0.0,
                lineWidth,
                new Color8Bit(Color.kBlue)));

    // Add intake (might be helpful at some point who knows)
    currentAngleLigament.append(
        new MechanismLigament2d(
            "Intake", Units.inchesToMeters(11.5), 60.0, lineWidth, new Color8Bit(Color.kRed)));

    setpointLigament.append(
        new MechanismLigament2d(
            "Intake", Units.inchesToMeters(11.5), 60.0, lineWidth, new Color8Bit(Color.kBlue)));

    SmartDashboard.putData("Arm/Arm Measured", angleVisualizer);
    SmartDashboard.putData("Arm/Arm Setpoint", setpointVisualizer);

    DataLogManager.log("[Arm] Initialization Complete");
  }

  private void configureMainMotor() {
    DataLogManager.log("[Arm] Configuring Main Motor");
    SparkMaxUtil.configure(
        mainMotor,
        () -> mainMotor.clearFaults(),
        () -> SparkMaxUtil.setInverted(mainMotor, ArmConstants.mainMotorInverted),
        () -> armPIDController.setP(ArmConstants.armKp),
        () -> armPIDController.setI(ArmConstants.armKi),
        () -> armPIDController.setD(ArmConstants.armKd),
        () -> armPIDController.setOutputRange(-1.0, 1.0),
        () -> armPIDController.setFeedbackDevice(armEncoder),
        () -> armPIDController.setPositionPIDWrappingEnabled(false),
        () -> armEncoder.setPositionConversionFactor(ArmConstants.armPositionConversionFactor),
        () -> armEncoder.setVelocityConversionFactor(ArmConstants.armVelocityConversionFactor),
        () -> mainMotor.setSmartCurrentLimit(ArmConstants.currentLimit),
        () -> mainMotor.enableVoltageCompensation(12.0),
        () -> mainMotor.setIdleMode(IdleMode.kBrake),
        () -> mainMotor.enableSoftLimit(SoftLimitDirection.kForward, true),
        () -> absoluteEncoder.setZeroOffset(0.0),
        () -> absoluteEncoder.setInverted(ArmConstants.absoluteEncoderInverted),
        () -> absoluteEncoder.setPositionConversionFactor(2 * Math.PI),
        () -> absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0),
        () ->
            mainMotor.setSoftLimit(
                SoftLimitDirection.kForward, (float) ArmConstants.forwardMovementLimitAngle),
        () -> mainMotor.enableSoftLimit(SoftLimitDirection.kReverse, true),
        () ->
            mainMotor.setSoftLimit(
                SoftLimitDirection.kReverse, (float) ArmConstants.reverseMovementLimitAngle),
        () ->
            mainMotor.setPeriodicFramePeriod(
                PeriodicFrame.kStatus3, SparkMaxUtil.disableFramePeriod),
        () ->
            mainMotor.setPeriodicFramePeriod(
                PeriodicFrame.kStatus4, SparkMaxUtil.disableFramePeriod),
        () -> mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20),
        () -> mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20),
        () ->
            mainMotor.setPeriodicFramePeriod(
                PeriodicFrame.kStatus7, SparkMaxUtil.disableFramePeriod));
  }

  private void configureFollowerMotor() {
    DataLogManager.log("[Arm] Configuring Follower Motor");
    SparkMaxUtil.configure(
        followerMotor,
        () -> followerMotor.clearFaults(),
        () -> followerMotor.setSmartCurrentLimit(ArmConstants.currentLimit),
        () -> followerMotor.setIdleMode(IdleMode.kBrake),
        () -> followerMotor.follow(mainMotor, true));

    SparkMaxUtil.configureFollower(followerMotor);
  }

  private void configureAbsoluteEncoder() {
    DataLogManager.log("[Arm] Configuring Absolute Encoder");
    SparkMaxUtil.configureNoReset(
        mainMotor,
        () -> absoluteEncoder.setZeroOffset(0.0),
        () -> absoluteEncoder.setInverted(ArmConstants.absoluteEncoderInverted),
        () -> absoluteEncoder.setPositionConversionFactor(2 * Math.PI),
        () -> absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0));
    // mainMotor.setCANTimeout(100);
    // absoluteEncoder.setZeroOffset(0.0);
    // absoluteEncoder.setInverted(ArmConstants.absoluteEncoderInverted);
    // absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    // absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60.0);
    // mainMotor.setCANTimeout(0);
  }

  public void resetToAbsolute() {
    DataLogManager.log("[Arm] Resetting Position to Absolute");
    // mainMotor.setCANTimeout(100);
    double position = getAbsoluteAngle().minus(ArmConstants.absoluteOffset).getRadians();

    SparkMaxUtil.configureNoReset(mainMotor, () -> armEncoder.setPosition(position));

    // boolean failed = true;
    // for (int i = 0; i < 5; i++) {
    //   if (armEncoder.setPosition(position) == REVLibError.kOk) {
    //     failed = false;
    //   }
    //   if (armEncoder.getPosition() == position) {
    //     break;
    //   }
    //   Timer.delay(0.010);
    // }

    // if (failed) {
    //   DriverStation.reportError("Failed to set absolute posititon of arm motor", false);
    //   DataLogManager.log("Failed to set absolute posititon of arm motor");
    //   absolutePositionNotSet.set(true);
    // } else {
    //   absolutePositionNotSet.set(false);
    // }
    // mainMotor.setCANTimeout(0);
  }

  public Command autoMoveToPosition(Rotation2d position) {
    return new FunctionalCommand(
            () -> previousSetpoint = getCurrentState(),
            () -> setDesiredPosition(position),
            (interrupted) -> setSpeed(0.0),
            () -> {
              double positionError = getPosition().getRadians() - position.getRadians();
              return autoDebouncer.calculate(
                  Math.abs(positionError) <= ArmConstants.autonomousAngleTolerance);
            },
            this)
        .withName("Arm Auto Move to " + position.getDegrees() + " Degrees");
  }

  public Command preciseMoveToPosition(Rotation2d position) {
    return new FunctionalCommand(
            () -> previousSetpoint = getCurrentState(),
            () -> setDesiredPosition(position),
            (interrupted) -> setSpeed(0.0),
            () -> {
              double positionError = getPosition().getRadians() - position.getRadians();
              return setpointDebouncer.calculate(
                  Math.abs(positionError) <= ArmConstants.angleTolerance);
            },
            this)
        .withName("Arm Precise Move to " + position.getDegrees() + " Degrees");
  }

  public Command moveToPosition(Rotation2d position) {
    return new FunctionalCommand(
            () -> previousSetpoint = getCurrentState(),
            () -> setDesiredPosition(position),
            (interrupted) -> setSpeed(0.0),
            () -> {
              double positionError = getPosition().getRadians() - position.getRadians();
              return setpointDebouncer.calculate(
                  Math.abs(positionError) <= ArmConstants.angleTolerance);
            },
            this)
        .withName("Arm Move to " + position.getDegrees() + " Degrees");
  }

  private void setMotionProfileState(TrapezoidProfile.State state) {
    double feedforward =
        armFeedforward.calculate(
            state.position, state.velocity, (state.velocity - previousSetpoint.velocity) / 0.020);

    previousSetpoint = state;

    log("FeedForward Voltage", feedforward);

    double pidOutput = pidController.calculate(getPosition().getRadians(), state.position);
    pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);

    mainMotor.setVoltage(pidOutput * RobotController.getBatteryVoltage() + feedforward);
  }

  private void setHoldState(TrapezoidProfile.State state) {
    double feedforward =
        armFeedforward.calculate(
            state.position, state.velocity, (state.velocity - previousSetpoint.velocity) / 0.020);

    previousSetpoint = state;

    log("FeedForward Voltage", feedforward);

    double pidOutput = holdPIDController.calculate(getPosition().getRadians(), state.position);
    pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);

    mainMotor.setVoltage(pidOutput * RobotController.getBatteryVoltage() + feedforward);
  }

  public void setDesiredPosition(Rotation2d position) {
    TrapezoidProfile.State currentState = previousSetpoint;
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(position.getRadians(), 0.0);

    TrapezoidProfile.State setpoint = armProfile.calculate(0.020, currentState, goalState);

    double positionError = Math.abs(setpoint.position - getPosition().getRadians());

    // Replan profile if it's too far from position or if it's finished
    if (positionError > ArmConstants.replanningError) {
      DataLogManager.log("Replanning arm profile: Position error too high");
      setpoint = armProfile.calculate(0.020, getCurrentState(), goalState);
    } else if (armProfile.isFinished(0.0) && positionError > Units.degreesToRadians(7.5)) {
      DataLogManager.log(
          "Raplanning arm profile: current profile is finished but too far from setpoint");
      setpoint = armProfile.calculate(0.020, getCurrentState(), goalState);
    }

    setMotionProfileState(setpoint);
  }

  public void setHoldPosition(Rotation2d position) {
    TrapezoidProfile.State currentState = previousSetpoint;
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(position.getRadians(), 0.0);

    TrapezoidProfile.State setpoint = armProfile.calculate(0.020, currentState, goalState);

    setHoldState(setpoint);
  }

  public void setSpeed(double speed) {
    mainMotor.setVoltage(speed * RobotController.getBatteryVoltage());
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPosition().getRadians(), getVelocity());
  }

  public void setProfileSetpoint(TrapezoidProfile.State state) {
    previousSetpoint = state;
  }

  @Log.NT(key = "Absolute Angle")
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRadians(absoluteEncoder.getPosition());
  }

  @Log.NT(key = "Angle")
  public Rotation2d getPosition() {
    if (RobotBase.isReal()) {
      return getAbsoluteAngle().minus(ArmConstants.absoluteOffset);
    } else {
      return Rotation2d.fromRadians(armSimulation.getAngleRads());
    }
  }

  @Log.NT(key = "3D Pose")
  public Pose3d get3DPose() {
    return new Pose3d(
        origin.getTranslation(), new Rotation3d(0.0, getPosition().getRadians() - Math.PI, 0.0));
  }

  public double getVelocity() {
    if (RobotBase.isReal()) {
      return armEncoder.getVelocity();
    } else {
      return armSimulation.getVelocityRadPerSec();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Position", Units.radiansToDegrees(getPosition().getRadians()));
    SmartDashboard.putNumber("Arm/Absolute Position", getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber(
        "Arm/Absolute Encoder Position", Units.radiansToDegrees(absoluteEncoder.getPosition()));

    SmartDashboard.putNumber("Arm/Velocity", Units.radiansToDegrees(getVelocity()));
    SmartDashboard.putNumber(
        "Arm/Absolute Encoder Velocity", Units.radiansToDegrees(absoluteEncoder.getVelocity()));

    SmartDashboard.putNumber(
        "Arm/Position Setpoint", Units.radiansToDegrees(previousSetpoint.position));
    SmartDashboard.putNumber(
        "Arm/Velocity Setpoint", Units.radiansToDegrees(previousSetpoint.velocity));

    SmartDashboard.putNumber(
        "Arm/Position Error",
        Units.radiansToDegrees(previousSetpoint.position - getPosition().getRadians()));
    SmartDashboard.putNumber(
        "Arm/Velocity Error", Units.radiansToDegrees(previousSetpoint.velocity - getVelocity()));

    currentAngleLigament.setAngle(Rotation2d.fromRadians(Math.PI).minus(getPosition()));
    setpointLigament.setAngle(Rotation2d.fromRadians(Math.PI - previousSetpoint.position));
  }

  @Override
  public void simulationPeriodic() {
    if (DriverStation.isEnabled()) {
      // REV it makes literally 0 sense for this to be in voltage, especially considering the fact
      // that it's called applied output...
      armSimulation.setInput(mainMotor.getAppliedOutput());
    } else {
      armSimulation.setInput(0.0);
    }

    armSimulation.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSimulation.getCurrentDrawAmps()));
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
              if (Math.abs(getPosition().getRadians() - ArmConstants.pickupAngle.getRadians())
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
        Commands.waitSeconds(2.5),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition().getRadians() - ArmConstants.ampAngle.getRadians())
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
        Commands.waitSeconds(2.3),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition().getRadians() - ArmConstants.subwooferAngle.getRadians())
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
              joystick.setButton(OperatorConstants.armToSource, true);
            }),
        Commands.waitSeconds(prematchDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(getPosition().getRadians() - ArmConstants.sourceAngle.getRadians())
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
