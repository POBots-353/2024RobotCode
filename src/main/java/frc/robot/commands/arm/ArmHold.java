// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmHold extends Command {
  private final Arm arm;

  private Rotation2d holdPosition;
  private TrapezoidProfile.State holdState;

  /** Creates a new ArmHold. */
  public ArmHold(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdPosition = arm.getPosition();
    holdState = new TrapezoidProfile.State(holdPosition.getRadians(), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(arm.getPosition().getRadians() - holdState.position)
        < ArmConstants.angleTolerance) {
      arm.setSpeed(0.0);
    } else {
      arm.setDesiredPosition(holdPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
