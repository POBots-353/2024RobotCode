// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;

public class AutoShoot extends Command {
  private final Arm arm;
  private final Intake intake;
  private final Shooter shooter;
  private final Swerve swerve;

  private Pose2d speakerPose;
  private Debouncer setpointDebouncer = new Debouncer(ArmConstants.debounceTime);

  /** Creates a new AutoShoot. */
  public AutoShoot(Arm arm, Intake intake, Shooter shooter, Swerve swerve) {
    this.arm = arm;
    this.intake = intake;
    this.shooter = shooter;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (AllianceUtil.isRedAlliance()) {
      speakerPose = FieldConstants.speakerRedAlliance;
    } else {
      speakerPose = FieldConstants.speakerBlueAlliance;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = speakerPose.minus(swerve.getPose()).getTranslation().getNorm();

    double angle = ArmConstants.autoShootInterpolation.get(distance);
    Rotation2d desiredAngle = Rotation2d.fromRadians(angle);
    arm.setDesiredPosition(Rotation2d.fromRadians(angle));
    shooter.setMotorSpeed(ShooterConstants.shooterVelocity);

    Rotation2d armAngleError = desiredAngle.minus(arm.getPosition());
    double shooterError = Math.abs(shooter.getVelocity() - ShooterConstants.shooterVelocity);

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError.getRadians()) < ArmConstants.angleTolerance
            && shooterError < ShooterConstants.velocityTolerance)) {
      intake.feedToShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotor();
    shooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
