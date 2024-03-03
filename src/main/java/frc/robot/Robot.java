// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.FaultLogger;
import frc.robot.util.LogUtil;
import java.util.HashMap;
import java.util.Map;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Notifier odometryNotifier;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();

    LogUtil.recordMetadata("Java Vendor", System.getProperty("java.vendor"));
    LogUtil.recordMetadata("Java Version", System.getProperty("java.version"));
    LogUtil.recordMetadata("WPILib Version", WPILibVersion.Version);

    LogUtil.recordMetadata(
        "REVLib Version",
        CANSparkBase.kAPIMajorVersion
            + "."
            + CANSparkBase.kAPIMinorVersion
            + "."
            + CANSparkBase.kAPIBuildVersion);
    LogUtil.recordMetadata("Runtime Type", getRuntimeType().toString());

    // Git and build information
    LogUtil.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    LogUtil.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
    LogUtil.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    LogUtil.recordMetadata("Git Date", BuildConstants.GIT_DATE);
    LogUtil.recordMetadata("Git Revision", BuildConstants.GIT_REVISION);

    if (RobotBase.isReal()) {
      DriverStation.startDataLog(DataLogManager.getLog());

      Map<Integer, String> motorNameMap = new HashMap<>();

      motorNameMap.put(FrontLeftModule.driveID, "Front Left Drive");
      motorNameMap.put(FrontLeftModule.turnID, "Front Left Turn");

      motorNameMap.put(FrontRightModule.driveID, "Front Right Drive");
      motorNameMap.put(FrontRightModule.turnID, "Front Right Turn");

      motorNameMap.put(BackLeftModule.driveID, "Back Left Drive");
      motorNameMap.put(BackLeftModule.turnID, "Back Left Turn");

      motorNameMap.put(BackRightModule.driveID, "Back Right Drive");
      motorNameMap.put(BackRightModule.turnID, "Back Right Turn");

      motorNameMap.put(ArmConstants.mainMotorID, "Arm Main");
      motorNameMap.put(ArmConstants.followerID, "Arm Follower");

      motorNameMap.put(ShooterConstants.topShooterID, "Shooter Top");
      motorNameMap.put(ShooterConstants.bottomShooterID, "Shooter Bottom");

      motorNameMap.put(IntakeConstants.intakeMotorID, "Intake");

      motorNameMap.put(ClimberConstants.leftMotorID, "Climber Left");
      motorNameMap.put(ClimberConstants.rightMotorID, "Climber Right");

      URCL.start(motorNameMap);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    odometryNotifier = new Notifier(m_robotContainer::updateSwerveOdometry);
    odometryNotifier.setName("OdometryThread");
    odometryNotifier.startPeriodic(1.0 / SwerveConstants.odometryUpdateFrequency);

    Monologue.setupMonologue(m_robotContainer, "/Monologue", false, true);

    addPeriodic(Monologue::updateAll, kDefaultPeriod);
    addPeriodic(FaultLogger::update, 1);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    double startTime = Timer.getFPGATimestamp();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber(
        "CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    double codeRuntime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
    SmartDashboard.putNumber("Code Runtime (ms)", codeRuntime);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void driverStationConnected() {
    Monologue.setFileOnly(DriverStation.isFMSAttached());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
