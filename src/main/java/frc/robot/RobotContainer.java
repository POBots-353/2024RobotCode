// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LogUtil;
import frc.robot.util.PersistentSendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Swerve swerve = new Swerve();

  PathPlannerPath pathDeux = PathPlannerPath.fromPathFile("PathDeux");

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.driverControllerPort);

  private PowerDistribution powerDistribution = new PowerDistribution();

  private PersistentSendableChooser<String> batteryChooser =
      new PersistentSendableChooser<>("Battery Number");
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoChooser();
    configureBatteryChooser();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Power Distribution Panel", powerDistribution);

    PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");


    LogUtil.recordMetadata("Battery Number", batteryChooser.getSelectedName());
    LogUtil.recordMetadata("Battery Nickname", batteryChooser.getSelected());

    swerve.setDefaultCommand(
        new TeleopSwerve(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            driverController::getRightY,
            () -> driverController.getHID().getLeftBumper(),
            SwerveConstants.maxTranslationalSpeed,
            SwerveConstants.maxAngularSpeed,
            swerve));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController
        .back()
        .and(driverController.start())
        .onTrue(Commands.runOnce(swerve::zeroYaw).ignoringDisable(true));

    driverController.x().whileTrue(swerve.run(swerve::lockModules));

    driverController
        .leftTrigger()
        .and(driverController.rightTrigger())
        .whileTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(6.0, 3.2, Rotation2d.fromDegrees(0.0)),
                new PathConstraints(3.0, 3.0, Units.degreesToRadians(180.0), 
                180.0)));

    driverController
        .leftTrigger()
        .and(driverController.x())
        .whileTrue(
            AutoBuilder.pathfindThenFollowPath(
              pathDeux, 
              new PathConstraints(3.0, 3.0, 
              Units.degreesToRadians(180.0), 180.0)));
  }

  private void configureAutoChooser() {
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("[SysID] Swerve Quasistatic Forward", swerve.quasistaticForward());
    autoChooser.addOption("[SysID] Swerve Quasistatic Backward", swerve.quasistaticBackward());
    autoChooser.addOption("[SysID] Swerve Dynamic Forward", swerve.dynamicForward());
    autoChooser.addOption("[SysID] Swerve Dynamic Backward", swerve.dynamicBackward());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBatteryChooser() {
    batteryChooser.addOption("2019.5 #2", "Al");
    batteryChooser.addOption("2019.5 #3", "Daniel");
    batteryChooser.addOption("2020 #1", "2020 #1");
    batteryChooser.addOption("2020 #2", "2020 #2");
    batteryChooser.addOption("2021 #1", "Fred");
    batteryChooser.addOption("2024 #1", "2024 #1");
    batteryChooser.addOption("2024 #2", "2024 #2");

    SmartDashboard.putData("Battery Chooser", batteryChooser);
  }

  public void updateSwerveOdometry() {
    swerve.updateOdometry();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
