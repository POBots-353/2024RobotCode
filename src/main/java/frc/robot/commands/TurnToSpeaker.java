// Copyright (c) FIRST and other WPILib contributors.
// Charlie is the best driver in the club, much better than me - Syon
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class TurnToSpeaker extends Command {
  private Swerve swerve;

  private long lastDetectionTime = 0;

  private Rotation2d desiredRotation = Rotation2d.fromDegrees(0.0);

  private PIDController turnToSpeakerController =
      new PIDController(0.95, 0, SwerveConstants.headingD);

  private DoubleSupplier forwardSpeed;
  private DoubleSupplier strafeSpeed;

  private DoubleSupplier maxTranslationalSpeed;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);

  /** Creates a new TurnToNote. */
  public TurnToSpeaker(
      DoubleSupplier forwardSpeed,
      DoubleSupplier strafeSpeed,
      DoubleSupplier maxTranslationalSpeed,
      Swerve swerve) {
    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    this.maxTranslationalSpeed = maxTranslationalSpeed;
    this.swerve = swerve;

    turnToSpeakerController.setTolerance(Units.degreesToRadians(1));
    turnToSpeakerController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void updateDesiredRotation() {
    if (!LimelightHelpers.getTV(VisionConstants.limelightName)) {
      return;
    }

    long detectionTime =
        LimelightHelpers.getLimelightNTTableEntry(VisionConstants.limelightName, "tx")
            .getLastChange();

    if (lastDetectionTime == detectionTime) {
      return;
    }
    lastDetectionTime = detectionTime;

    double rotation = LimelightHelpers.getTX(VisionConstants.limelightName);

    double timestamp =
        (lastDetectionTime / 1.0e6)
            - (LimelightHelpers.getLatency_Pipeline(VisionConstants.limelightName)) / 1000.0;

    Optional<Rotation2d> rotationAtTime = swerve.getRotationAtTime(timestamp);

    if (rotationAtTime.isEmpty()) {
      rotationAtTime = Optional.of(swerve.getHeading());
    }

    desiredRotation = rotationAtTime.get().minus(Rotation2d.fromDegrees(rotation));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateDesiredRotation();
    double turningSpeed =
        turnToSpeakerController.calculate(
            swerve.getPose().getRotation().getRadians(), desiredRotation.getRadians());

    turningSpeed = MathUtil.clamp(turningSpeed, -1.00, 1.00);

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

    swerve.driveFieldOriented(
        forwardMetersPerSecond,
        strafeMetersPerSecond,
        turningSpeed * SwerveConstants.turnToAngleMaxVelocity,
        true,
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forwardRateLimiter.reset(0.0);
    strafeRateLimiter.reset(0.0);
    turnToSpeakerController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
