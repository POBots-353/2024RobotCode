// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoClimb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbLeft extends SequentialCommandGroup {
  /** Creates a new AutoClimbLeft. */
  Swerve swerve;
  Climber climber;
  
  public AutoClimbLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double[] coordinates = swerve.getLeftChainPose();
    double xCoordinates = coordinates[0];
    double yCoordinates = coordinates[1];
    Rotation2d angleHeading = Rotation2d.fromDegrees(coordinates[2]);

    addCommands(
      Commands.runOnce(
        () -> climber.setClimberUp()),
      
      AutoBuilder.pathfindToPose(
          new Pose2d(
              xCoordinates,
              yCoordinates,
              angleHeading),
          new PathConstraints(
              SwerveConstants.maxTranslationalSpeed,
              SwerveConstants.maxTranslationalAcceleration,
              Units.degreesToRadians(180.0), 180.0)));
    
      Commands.runOnce(
        () -> climber.setClimberDown());
  }
}
