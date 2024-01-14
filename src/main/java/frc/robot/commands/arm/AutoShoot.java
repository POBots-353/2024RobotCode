// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;

public class AutoShoot extends Command {
  private final Arm arm;
  private final Swerve swerve;

  private Pose2d speakerPose;

  /** Creates a new AutoShoot. */
  public AutoShoot(Arm arm, Swerve swerve) {
    this.arm = arm;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
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
    arm.setPosition(Rotation2d.fromRadians(angle));
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
