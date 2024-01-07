package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private String moduleName;

  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private SparkPIDController drivePID;
  private SparkPIDController turnPID;

  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
      SwerveConstants.driveKv, SwerveConstants.driveKa);

  private Rotation2d angleOffset;

  private boolean isOpenLoop;
  private boolean allowTurnInPlace;

  private SwerveModuleState desiredState = new SwerveModuleState();
  private SwerveModuleState previousState = new SwerveModuleState();

  public SwerveModule(String moduleName, int driveID, int turnID, int canCoderID, Rotation2d angleOffset) {
    this.moduleName = moduleName;
    this.angleOffset = angleOffset;

    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();

    turnEncoder = turnMotor.getEncoder();
    turnPID = turnMotor.getPIDController();

    configureDriveMotor();
    configureTurnMotor();
  }

  private void configureDriveMotor() {
    driveMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);

    driveMotor.setInverted(SwerveConstants.driveMotorInverted);

    driveMotor.setOpenLoopRampRate(SwerveConstants.openLoopRamp);
    driveMotor.setClosedLoopRampRate(SwerveConstants.closedLoopRamp);

    driveMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    driveMotor.setSmartCurrentLimit(SwerveConstants.driveCurrentLimit);

    driveMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000 / SwerveConstants.odometryUpdateFrequency);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    drivePID.setP(SwerveConstants.driveP);
    drivePID.setOutputRange(-1, 1);

    driveEncoder.setPositionConversionFactor(SwerveConstants.drivePositionConversion);
    driveEncoder.setVelocityConversionFactor(SwerveConstants.driveVelocityConversion);

    driveMotor.setCANTimeout(0);
  }

  private void configureTurnMotor() {
    turnMotor.restoreFactoryDefaults();

    turnMotor.setCANTimeout(250);

    turnMotor.setInverted(SwerveConstants.turnMotorInverted);

    turnMotor.enableVoltageCompensation(SwerveConstants.voltageCompensation);
    turnMotor.setSmartCurrentLimit(SwerveConstants.turnCurrentLimit);

    turnMotor.setIdleMode(IdleMode.kCoast);

    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000 / SwerveConstants.odometryUpdateFrequency);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);

    turnEncoder.setPositionConversionFactor(SwerveConstants.turnPositionConversion);

    turnPID.setP(SwerveConstants.turnP);
    turnPID.setD(SwerveConstants.turnD);
    turnPID.setOutputRange(-1.0, 1.0);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);

    turnMotor.setCANTimeout(0);
  }

  public void resetToAbsolute() {
    Rotation2d position = Rotation2d
        .fromDegrees(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());

    turnMotor.setCANTimeout(250);

    turnEncoder.setPosition(position.getRadians());

    turnMotor.setCANTimeout(0);
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop, boolean allowTurnInPlace) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

    if (optimizedState.speedMetersPerSecond == 0.0 && !allowTurnInPlace) {
      optimizedState.angle = desiredState.angle;

      optimizedState = SwerveModuleState.optimize(optimizedState, getAngle());
    }

    previousState = desiredState;
    desiredState = optimizedState;
    this.isOpenLoop = isOpenLoop;
    this.allowTurnInPlace = allowTurnInPlace;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public double getVelocity() {
    return driveEncoder.getVelocity();
  }

  // Temporarily stubbed
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(0.0);
  }

  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromDegrees(0.0);
  }

  private void setSpeed(double speedMetersPerSecond) {
    if (isOpenLoop) {
      driveMotor.set(speedMetersPerSecond / SwerveConstants.maxModuleSpeed);
    } else {
      double feedForward = driveFeedforward.calculate(speedMetersPerSecond,
          (speedMetersPerSecond - previousState.speedMetersPerSecond) / 0.020);

      drivePID.setReference(speedMetersPerSecond, ControlType.kVelocity, 0, feedForward, ArbFFUnits.kVoltage);
    }
  }

  private void setAngle(Rotation2d angle) {
    turnPID.setReference(angle.getRadians(), ControlType.kPosition);
  }

  public void periodic() {
    setSpeed(desiredState.speedMetersPerSecond);
    setAngle(desiredState.angle);
  }
}
