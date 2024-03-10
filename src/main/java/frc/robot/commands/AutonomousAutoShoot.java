// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShooterState;

public class AutonomousAutoShoot extends Command {
  private final Arm arm;
  private final Intake intake;
  private final Shooter shooter;
  private final Swerve swerve;

  private Debouncer setpointDebouncer = new Debouncer(0.30);
  private Debouncer feedingDebouncer = new Debouncer(0.75);

  private boolean fedNote = false;

  /** Creates a new AutoShoot. */
  public AutonomousAutoShoot(Arm arm, Intake intake, Shooter shooter, Swerve swerve) {
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
    fedNote = false;

    setpointDebouncer.calculate(false);
    feedingDebouncer.calculate(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = swerve.getSpeakerDistance();

    double angle = AutoShootConstants.autoShootAngleInterpolation.get(distance);
    Rotation2d desiredAngle = Rotation2d.fromRadians(angle);
    arm.setDesiredPosition(desiredAngle);

    ShooterState state = AutoShootConstants.autoShootSpeeds.get(distance);

    shooter.setShooterState(state);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", desiredAngle.getDegrees());

    Rotation2d armAngleError = desiredAngle.minus(arm.getPosition());

    if (setpointDebouncer.calculate(
        Math.abs(armAngleError.getRadians()) < ArmConstants.autoShootAngleTolerance
            && shooter.nearSetpoint())) {
      intake.feedToShooter();
      fedNote = true;
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
    return feedingDebouncer.calculate(fedNote);
  }
}
