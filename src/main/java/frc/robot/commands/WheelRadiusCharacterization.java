// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class WheelRadiusCharacterization extends Command {
  private final Swerve swerve;

  private final double characterizationSpeed = Units.degreesToRadians(7.5);
  private final SlewRateLimiter angularRateLimiter =
      new SlewRateLimiter(Units.degreesToRadians(60.0));

  private double lastGyroYaw = 0.0;
  private double accumYaw = 0.0;

  private double[] startWheelPositions;

  private double currentWheelRadius = 0.0;

  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(Swerve swerve) {
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();
    lastGyroYaw = swerve.getYawRadians();
    accumYaw = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angularSpeed = angularRateLimiter.calculate(characterizationSpeed);

    swerve.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, angularSpeed), false, true, true);

    double currentYaw = swerve.getYawRadians();

    accumYaw += MathUtil.angleModulus(currentYaw - lastGyroYaw);
    lastGyroYaw = currentYaw;

    double averagePosition = 0.0;
    double[] wheelPositions = swerve.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averagePosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    averagePosition /= 4.0;

    currentWheelRadius = (accumYaw * SwerveConstants.driveBaseRadius) / averagePosition;

    SmartDashboard.putNumber("Measured Wheel Radius", Units.metersToInches(currentWheelRadius));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angularRateLimiter.reset(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
