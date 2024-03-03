// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShoot extends Command {
  private final Arm arm;
  private final Intake intake;
  private final Shooter shooter;
  private final Swerve swerve;

  private Debouncer setpointDebouncer = new Debouncer(0.30);

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = swerve.getSpeakerDistance();

    double angle = AutoShootConstants.autoShootAngleInterpolation.get(distance);
    Rotation2d desiredAngle = Rotation2d.fromRadians(angle);
    arm.setDesiredPosition(desiredAngle);

    double motorRPM = AutoShootConstants.autoShootRPMInterpolation.get(distance);
    shooter.setMotorSpeed(motorRPM);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", desiredAngle.getDegrees());

    Rotation2d armAngleError = desiredAngle.minus(arm.getPosition());
    double shooterError = motorRPM - shooter.getBottomVelocity();

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError.getRadians()) < ArmConstants.autoShootAngleTolerance
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
