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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import java.util.function.DoubleSupplier;

public class ShootWhileMoving extends Command {
  private final Arm arm;
  private final Intake intake;
  private final Shooter shooter;
  private final Swerve swerve;

  private DoubleSupplier forwardSpeed;
  private DoubleSupplier strafeSpeed;

  private double maxTranslationalSpeed;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);

  private PIDController turnToAngleController = new PIDController(1.0, 0, SwerveConstants.headingD);

  private Pose2d speakerPose;

  private ChassisSpeeds previouSpeeds = new ChassisSpeeds();

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double setpointDebounceTime = 0.30;
  private final double feedTime = 0.250;

  private Debouncer setpointDebouncer = new Debouncer(setpointDebounceTime);

  private boolean simShotNote = false;

  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(
      DoubleSupplier forwardSpeed,
      DoubleSupplier strafeSpeed,
      double maxTranslationalSpeed,
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

    simShotNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotPose = swerve.getPose().getTranslation();
    ChassisSpeeds fieldSpeeds = swerve.getFieldRelativeSpeeds();

    ChassisSpeeds fieldAcceleration = fieldSpeeds.minus(previouSpeeds).div(0.020);

    double fieldAccelX = accelXFilter.calculate(fieldAcceleration.vxMetersPerSecond);
    double fieldAccelY = accelYFilter.calculate(fieldAcceleration.vyMetersPerSecond);

    SmartDashboard.putNumber("Auto Shoot/Acceleration X", fieldAccelX);
    SmartDashboard.putNumber("Auto Shoot/Acceleration Y", fieldAccelY);

    double distance = robotPose.minus(speakerPose.getTranslation()).getNorm();

    double shotTime = ArmConstants.autoShootTimeInterpolation.get(distance);

    Translation2d virtualGoalLocation = new Translation2d();

    for (int i = 0; i < 5; i++) {
      double virtualGoalX =
          speakerPose.getX() - shotTime * (fieldSpeeds.vxMetersPerSecond + fieldAccelX * feedTime);
      double virtualGoalY =
          speakerPose.getY() - shotTime * (fieldSpeeds.vyMetersPerSecond + fieldAccelY * feedTime);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double newDistance = robotPose.minus(virtualGoalLocation).getNorm();
      double newShotTime = ArmConstants.autoShootTimeInterpolation.get(newDistance);

      if (Math.abs(shotTime - newShotTime) <= 0.05) {
        shotTime = newShotTime;
        distance = newDistance;
        break;
      }

      shotTime = newShotTime;
      distance = newDistance;
    }

    swerve
        .getField()
        .getObject("Moving Goal")
        .setPose(new Pose2d(virtualGoalLocation, new Rotation2d()));

    // Calculate arm angle
    Rotation2d armAngle =
        Rotation2d.fromRadians(ArmConstants.autoShootAngleInterpolation.get(distance));
    arm.setDesiredPosition(armAngle);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", armAngle.getDegrees());

    // Shooter speed
    double motorRPM = ArmConstants.autoShootRPMInterpolation.get(distance);

    shooter.setMotorSpeed(motorRPM);

    // Calculate robot angle and drive speeds (copied from TeleopSwerve command)
    double forwardMetersPerSecond = -forwardSpeed.getAsDouble() * maxTranslationalSpeed;
    double strafeMetersPerSecond = strafeSpeed.getAsDouble() * maxTranslationalSpeed;

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
        robotPose.minus(virtualGoalLocation).getAngle().plus(Rotation2d.fromRadians(Math.PI));

    if (AllianceUtil.isRedAlliance()) {
      desiredAngle = desiredAngle.plus(Rotation2d.fromRadians(Math.PI));
    }

    Rotation2d robotAngle = swerve.getHeading();

    double angularSpeed =
        turnToAngleController.calculate(robotAngle.getRadians(), desiredAngle.getRadians());

    angularSpeed = MathUtil.clamp(angularSpeed, -0.75, 0.75);

    swerve.driveFieldOriented(
        forwardMetersPerSecond,
        strafeMetersPerSecond,
        angularSpeed * SwerveConstants.turnToAngleMaxVelocity,
        true,
        true,
        false);

    Rotation2d armAngleError = armAngle.minus(arm.getPosition());
    Rotation2d driveAngleError = robotAngle.minus(desiredAngle);
    double shooterError = motorRPM - shooter.getBottomVelocity();

    SmartDashboard.putNumber("Auto Shoot/Drive Angle Error", driveAngleError.getDegrees());

    if (setpointDebouncer.calculate(
            Math.abs(armAngleError.getRadians()) < ArmConstants.autoShootAngleTolerance
                && shooterError < ShooterConstants.velocityTolerance)
        && Math.abs(driveAngleError.getDegrees()) <= 7.00) {
      intake.feedToShooter();

      if (!simShotNote) {
        NoteVisualizer.shoot().beforeStarting(Commands.waitSeconds(0.250)).schedule();

        simShotNote = true;
      }
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
