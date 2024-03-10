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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ShooterState;

public class AutonomousAutoShoot extends Command {
  private final Arm arm;
  private final Shooter shooter;
  private final Swerve swerve;

  private Debouncer setpointDebouncer = new Debouncer(0.30);

  private Rotation2d desiredAngle = new Rotation2d();
  private ShooterState desiredState = new ShooterState();

  /** Creates a new AutoShoot. */
  public AutonomousAutoShoot(Arm arm, Shooter shooter, Swerve swerve) {
    this.arm = arm;
    this.shooter = shooter;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpointDebouncer.calculate(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = swerve.getSpeakerDistance();

    desiredAngle = AutoShootConstants.autoShootAngleMap.get(distance);
    arm.setDesiredPosition(desiredAngle);

    desiredState = AutoShootConstants.autoShootSpeedMap.get(distance);

    shooter.setShooterState(desiredState);

    SmartDashboard.putNumber("Auto Shoot/Desired Angle", desiredAngle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d armAngleError = desiredAngle.minus(arm.getPosition());

    return setpointDebouncer.calculate(
        Math.abs(armAngleError.getRadians()) < ArmConstants.autoShootAngleTolerance
            && shooter.nearSetpoint());
  }
}
