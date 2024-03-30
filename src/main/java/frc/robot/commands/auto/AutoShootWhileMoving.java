// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;
import frc.robot.util.ShooterState;

public class AutoShootWhileMoving extends Command {
  private final Arm arm;
  private final Shooter shooter;
  private final Swerve swerve;

  private Pose2d speakerPose;

  private ChassisSpeeds previouSpeeds = new ChassisSpeeds();

  private LinearFilter accelXFilter = LinearFilter.movingAverage(2);
  private LinearFilter accelYFilter = LinearFilter.movingAverage(2);

  private final double feedTime = 0.0353;

  /** Creates a new ShootWhileMoving. */
  public AutoShootWhileMoving(Arm arm, Shooter shooter, Swerve swerve) {
    this.arm = arm;
    this.shooter = shooter;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speakerPose = AllianceUtil.getSpeakerPose();

    previouSpeeds = swerve.getFieldRelativeSpeeds();

    arm.setProfileSetpoint(arm.getCurrentState());

    accelXFilter.reset();
    accelYFilter.reset();
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

    double distance = swerve.getSpeakerDistance();

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

      double newDistance = robotPose.getDistance(virtualGoalLocation);
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

    previouSpeeds = fieldSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.getField().getObject("Moving Goal").setPoses();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
