// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import frc.robot.util.AutoShootMathUtil;
import frc.robot.util.ShooterState;
import java.util.function.DoubleSupplier;

public class ShootWhileMoving extends Command {
  private static InterpolatingDoubleTreeMap driveAngleToleranceMap =
      new InterpolatingDoubleTreeMap();

  static {
    driveAngleToleranceMap.put(1.36, 30.0353);
    driveAngleToleranceMap.put(1.88, 25.0);
    driveAngleToleranceMap.put(3.0, 15.0);
    driveAngleToleranceMap.put(4.6, 10.0);
  }

  private static InterpolatingDoubleTreeMap armAngleToleranceMap = new InterpolatingDoubleTreeMap();

  static {
    armAngleToleranceMap.put(1.36, 1.5);
    armAngleToleranceMap.put(1.8, 0.80);
    armAngleToleranceMap.put(3.53, 0.60);
  }

  private static final Rotation2d desiredAngleOffset = Rotation2d.fromRadians(Math.PI);

  private final Arm arm;
  private final Intake intake;
  private final Shooter shooter;
  private final Swerve swerve;

  private DoubleSupplier forwardSpeed;
  private DoubleSupplier strafeSpeed;

  private DoubleSupplier maxTranslationalSpeed;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);

  private PIDController turnToAngleController = new PIDController(1.0, 0.10, 0.001);

  private Pose2d speakerPose;

  private ChassisSpeeds previouSpeeds = new ChassisSpeeds();

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.10;
  private final double feedTime = 0.100;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);
  private Debouncer shooterDebouncer = new Debouncer(setpointDebounceTime);

  private boolean simShotNote = false;

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(
      DoubleSupplier forwardSpeed,
      DoubleSupplier strafeSpeed,
      DoubleSupplier maxTranslationalSpeed,
      Arm arm,
      Intake intake,
      Shooter shooter,
      Swerve swerve) {
    this.arm = arm;
    this.intake = intake;
    this.shooter = shooter;
    this.swerve = swerve;

    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    this.maxTranslationalSpeed = maxTranslationalSpeed;

    turnToAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, shooter, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose = AllianceUtil.getSpeakerPose();

    previouSpeeds = swerve.getFieldRelativeSpeeds();

    arm.setProfileSetpoint(arm.getCurrentState());

    setpointDebouncer.calculate(false);
    shooterDebouncer.calculate(false);
    simShotNote = false;

    accelXFilter.reset();
    accelYFilter.reset();

    swerve.setIgnoreArducam(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();
    Translation2d robotTranslation = robotPose.getTranslation();
    ChassisSpeeds fieldSpeeds = swerve.getFieldRelativeSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previouSpeeds).div(0.020);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("Auto Shoot/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("Auto Shoot/Acceleration Y", fieldAccelY);

    double speakerDistance = swerve.getSpeakerDistance();
    double distance = speakerDistance;

    double shotTime = AutoShootConstants.autoShootTimeInterpolation.get(distance);
    Rotation2d armAngle = AutoShootConstants.autoShootAngleMap.get(distance);

    Translation2d virtualGoalLocation = speakerPose.getTranslation();

    int iterations = 0;

    for (int i = 0; i < 5; i++) {
      iterations = i + 1;

      double virtualGoalX =
          speakerPose.getX()
              - shotTime * (fieldSpeeds.vxMetersPerSecond + fieldAccelX * feedTime * 0.5);
      double virtualGoalY =
          speakerPose.getY()
              - shotTime * (fieldSpeeds.vyMetersPerSecond + fieldAccelY * feedTime * 0.5);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double newDistance = robotTranslation.getDistance(virtualGoalLocation);
      double newShotTime = AutoShootConstants.autoShootTimeInterpolation.get(newDistance);

      Rotation2d newArmAngle = AutoShootConstants.autoShootAngleMap.get(newDistance);

      if (Math.abs(newArmAngle.minus(armAngle).getDegrees()) <= 0.0005) {
        shotTime = newShotTime;
        armAngle = newArmAngle;
        distance = newDistance;
        break;
      }

      shotTime = newShotTime;
      distance = newDistance;
      armAngle = newArmAngle;
    }

    SmartDashboard.putNumber("Auto Shoot/Iterations", iterations);

    swerve
        .getField()
        .getObject("Moving Goal")
        .setPose(new Pose2d(virtualGoalLocation, new Rotation2d()));

    arm.setAutoShootPosition(armAngle);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", armAngle.getDegrees());

    // Shooter speed
    ShooterState shooterState = AutoShootConstants.autoShootSpeedMap.get(distance);

    shooter.setShooterState(shooterState);

    // Calculate robot angle and drive speeds (copied from TeleopSwerve command)
    double forwardMetersPerSecond =
        -forwardSpeed.getAsDouble() * maxTranslationalSpeed.getAsDouble();
    double strafeMetersPerSecond = strafeSpeed.getAsDouble() * maxTranslationalSpeed.getAsDouble();

    forwardMetersPerSecond = forwardRateLimiter.calculate(forwardMetersPerSecond);
    strafeMetersPerSecond = strafeRateLimiter.calculate(strafeMetersPerSecond);

    if (Math.abs(forwardMetersPerSecond) < Units.inchesToMeters(0.5)) {
      forwardMetersPerSecond = 0.0;
      forwardRateLimiter.reset(0.0);
    }

    if (Math.abs(strafeMetersPerSecond) < Units.inchesToMeters(0.5)) {
      strafeMetersPerSecond = 0.0;
      strafeRateLimiter.reset(0.0);
    }

    Rotation2d desiredAngle =
        robotTranslation.minus(virtualGoalLocation).getAngle().plus(desiredAngleOffset);

    double angularSpeed =
        turnToAngleController.calculate(
            robotPose.getRotation().getRadians(), desiredAngle.getRadians());

    angularSpeed = MathUtil.clamp(angularSpeed, -1.0, 1.0);

    swerve.driveFieldOriented(
        forwardMetersPerSecond,
        strafeMetersPerSecond,
        angularSpeed * SwerveConstants.turnToAngleMaxVelocity,
        true,
        true,
        false);

    Rotation2d armAngleError = armAngle.minus(arm.getPosition());
    Rotation2d driveAngleError = robotPose.getRotation().minus(desiredAngle);

    SmartDashboard.putNumber("Auto Shoot/Drive Angle Error", driveAngleError.getDegrees());

    boolean facingSpeaker = AutoShootMathUtil.isFacingSpeaker(robotPose, virtualGoalLocation);

    SmartDashboard.putBoolean("Auto Shoot/Facing Speaker", facingSpeaker);

    if (setpointDebouncer.calculate(
            Math.abs(armAngleError.getDegrees()) <= armAngleToleranceMap.get(speakerDistance))
        && shooterDebouncer.calculate(shooter.nearSetpoint())
        && facingSpeaker) {
      intake.feedToShooter();

      if (!simShotNote && RobotBase.isSimulation()) {
        NoteVisualizer.shoot().schedule();
      }
      simShotNote = true;
    }

    previouSpeeds = fieldSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotor();
    shooter.stopMotor();

    forwardRateLimiter.reset(0.0);
    strafeRateLimiter.reset(0.0);
    turnToAngleController.reset();

    swerve.getField().getObject("Moving Goal").setPoses();
    swerve.setIgnoreArducam(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
