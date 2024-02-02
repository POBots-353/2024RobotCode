// Copyright (c) FIRST and other WPILib contributors.
// Charlie is the best driver in the club, much better than me - Syon
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Detector;
import java.util.function.DoubleSupplier;

public class TurnToNote extends Command {
  private Swerve swerve = new Swerve();

  private double lastDetectionTime = 0.0;

  private Rotation2d desiredRotation = Rotation2d.fromDegrees(0.0);

  private PIDController turnToNoteController =
      new PIDController(SwerveConstants.headingP, 0, SwerveConstants.headingD);

  private DoubleSupplier forwardSpeed;
  private DoubleSupplier strafeSpeed;

  private SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  private SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);

  /** Creates a new TurnToNote. */
  public TurnToNote(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, Swerve swerve) {
    this.forwardSpeed = forwardSpeed;
    this.strafeSpeed = strafeSpeed;
    this.swerve = swerve;

    turnToNoteController.setTolerance(Units.degreesToRadians(1));
    turnToNoteController.enableContinuousInput(lastDetectionTime, lastDetectionTime);

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

    LimelightHelpers.Results results =
        LimelightHelpers.getLatestResults(VisionConstants.limelightName).targetingResults;
    if (results.timestamp_LIMELIGHT_publish == lastDetectionTime) {
      return;
    }
    lastDetectionTime = results.timestamp_LIMELIGHT_publish;
    LimelightTarget_Detector[] limelightDetector = results.targets_Detector;

    if (limelightDetector.length == 0) {
      return;
    }

    double rotation = limelightDetector[0].tx;

    desiredRotation = swerve.getHeading().minus(Rotation2d.fromDegrees(rotation));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateDesiredRotation();
    double turningSpeed =
        turnToNoteController.calculate(
            swerve.getHeading().getRadians(), desiredRotation.getRadians());

    double forwardMetersPerSecond =
        -forwardSpeed.getAsDouble() * SwerveConstants.maxTranslationalSpeed;
    double strafeMetersPerSecond =
        strafeSpeed.getAsDouble() * SwerveConstants.maxTranslationalSpeed;

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
    turnToNoteController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
