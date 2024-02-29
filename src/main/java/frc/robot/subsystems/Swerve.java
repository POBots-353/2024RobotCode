// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.controllers.VirtualJoystick;
import frc.lib.controllers.VirtualXboxController;
import frc.lib.subsystem.VirtualSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.StationCoordinateConstants.CenterChainPoses;
import frc.robot.Constants.StationCoordinateConstants.LeftChainPoses;
import frc.robot.Constants.StationCoordinateConstants.RightChainPoses;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.ArducamConstants;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.util.AStarPathfinder;
import frc.robot.util.AllianceUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import monologue.Annotations.Log;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Swerve extends VirtualSubsystem implements Logged {
  public static record PoseEstimate(Pose3d estimatedPose, double timestamp, Vector<N3> standardDevs)
      implements Comparable<PoseEstimate> {
    @Override
    public int compareTo(PoseEstimate other) {
      if (timestamp > other.timestamp) {
        return 1;
      } else if (timestamp < other.timestamp) {
        return -1;
      }
      return 0;
    }
  }

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveConstants.wheelLocations);

  private SwerveModule frontLeftModule =
      new SwerveModule(
          "Front Left",
          FrontLeftModule.driveID,
          FrontLeftModule.turnID,
          FrontLeftModule.encoderID,
          FrontLeftModule.angleOffset);

  private SwerveModule frontRightModule =
      new SwerveModule(
          "Front Right",
          FrontRightModule.driveID,
          FrontRightModule.turnID,
          FrontRightModule.encoderID,
          FrontRightModule.angleOffset);

  private SwerveModule backLeftModule =
      new SwerveModule(
          "Back Left",
          BackLeftModule.driveID,
          BackLeftModule.turnID,
          BackLeftModule.encoderID,
          BackLeftModule.angleOffset);

  private SwerveModule backRightModule =
      new SwerveModule(
          "Back Right",
          BackRightModule.driveID,
          BackRightModule.turnID,
          BackRightModule.encoderID,
          BackRightModule.angleOffset);

  private AHRS navx = new AHRS(SPI.Port.kMXP, (byte) SwerveConstants.odometryUpdateFrequency);

  @Log.NT(key = "Angle Offset")
  private Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);

  public static final Lock odometryLock = new ReentrantLock();
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveOdometry simOdometry;

  private SwerveDriveWheelPositions previousWheelPositions =
      new SwerveDriveWheelPositions(getModulePositions());
  private Rotation2d previousAngle = Rotation2d.fromDegrees(0.0);

  private PhotonCamera arducam = new PhotonCamera(VisionConstants.arducamName);
  private PhotonPoseEstimator arducamPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamTransform);

  private PhotonCameraSim arducamSim;
  private VisionSystemSim visionSim;

  private List<Pose3d> detectedTargets = new ArrayList<>();
  private List<Pose3d> rejectedPoses = new ArrayList<>();
  @Log.NT private double limelightLastDetectedTime = 0.0;

  private List<PoseEstimate> poseEstimates = new ArrayList<>();

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(4.0), null, null),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                frontLeftModule.setCharacterizationVolts(volts.in(Volts));
                frontRightModule.setCharacterizationVolts(volts.in(Volts));

                backLeftModule.setCharacterizationVolts(volts.in(Volts));
                backRightModule.setCharacterizationVolts(volts.in(Volts));
              },
              t -> {
                t.motor("Front Left")
                    .linearVelocity(MetersPerSecond.of(frontLeftModule.getVelocity()))
                    .linearPosition(Meters.of(frontLeftModule.getPosition()))
                    .voltage(Volts.of(frontLeftModule.getVoltage()));

                t.motor("Front Right")
                    .linearVelocity(MetersPerSecond.of(frontRightModule.getVelocity()))
                    .linearPosition(Meters.of(frontRightModule.getPosition()))
                    .voltage(Volts.of(frontRightModule.getVoltage()));

                t.motor("Back Left")
                    .linearVelocity(MetersPerSecond.of(backLeftModule.getVelocity()))
                    .linearPosition(Meters.of(backLeftModule.getPosition()))
                    .voltage(Volts.of(backLeftModule.getVoltage()));

                t.motor("Back Right")
                    .linearVelocity(MetersPerSecond.of(backRightModule.getVelocity()))
                    .linearPosition(Meters.of(backRightModule.getPosition()))
                    .voltage(Volts.of(backLeftModule.getVoltage()));
              },
              this));

  private Field2d field = new Field2d();

  private final double prematchDriveDelay = 1.0;
  private final double prematchTranslationalTolerance = 0.1;

  private double simYaw = 0.0;

  /** Creates a new Swerve. */
  public Swerve() {
    PhotonCamera.setVersionCheckEnabled(false);

    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());
    arducamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    if (RobotBase.isSimulation()) {
      simOdometry = new SwerveDriveOdometry(kinematics, getHeading(), getModulePositions());
      visionSim = new VisionSystemSim("main");

      visionSim.addAprilTags(FieldConstants.aprilTagLayout);

      SimCameraProperties cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(100.0));
      cameraProperties.setCalibError(0.25, 0.15);
      cameraProperties.setFPS(28);
      cameraProperties.setAvgLatencyMs(36);
      cameraProperties.setLatencyStdDevMs(15);

      arducamSim = new PhotonCameraSim(arducam, cameraProperties);
      visionSim.addCamera(arducamSim, VisionConstants.arducamTransform);

      arducamSim.enableRawStream(false);
      arducamSim.enableProcessedStream(false);
    }

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            AutoConstants.translationConstants,
            AutoConstants.rotationConstants,
            SwerveConstants.maxModuleSpeed,
            SwerveConstants.driveBaseRadius,
            new ReplanningConfig(false, true)),
        AllianceUtil::isRedAlliance,
        this);

    Pathfinding.setPathfinder(new AStarPathfinder(this::getPose));

    PathPlannerLogging.setLogActivePathCallback(
        path -> {
          field.getObject("trajectory").setPoses(path);
          if (path.size() == 0) {
            field.getObject("Target Pose").setPoses(path);
            lockModules();
          }
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> {
          field.getObject("Target Pose").setPose(pose);
        });

    SmartDashboard.putData("Swerve/Field", field);

    SmartDashboard.putData(
        "Swerve/NavX Accelerometer",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("3AxisAccelerometer");
            builder.addDoubleProperty("X", navx::getWorldLinearAccelX, null);
            builder.addDoubleProperty("Y", navx::getWorldLinearAccelY, null);
            builder.addDoubleProperty("Z", navx::getWorldLinearAccelZ, null);
          }
        });

    SmartDashboard.putData("Swerve/Built-in Accelerometer", new BuiltInAccelerometer());

    SmartDashboard.putData(
        "Swerve/Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> frontLeftModule.getAngle().getDegrees(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> frontLeftModule.getVelocity(), null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> frontRightModule.getAngle().getDegrees(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> frontRightModule.getVelocity(), null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> backLeftModule.getAngle().getDegrees(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> backLeftModule.getVelocity(), null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> backRightModule.getAngle().getDegrees(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> backRightModule.getVelocity(), null);

            builder.addDoubleProperty("Robot Angle", () -> getHeading().getDegrees(), null);
          }
        });

    SmartDashboard.putData(
        "Swerve/Gyro",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
          }
        });

    DataLogManager.log("NavX Firmware: " + navx.getFirmwareVersion());

    Commands.sequence(Commands.waitSeconds(2.0), runOnce(this::resetModulesToAbsolute))
        .ignoringDisable(true)
        .schedule();
  }

  public void resetModulesToAbsolute() {
    frontLeftModule.resetToAbsolute();
    frontRightModule.resetToAbsolute();
    backLeftModule.resetToAbsolute();
    backRightModule.resetToAbsolute();
  }

  @Log.NT
  public boolean arducamConnected() {
    return arducam.isConnected();
  }

  public ChassisSpeeds getFudgeFactoredSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    if (isOpenLoop) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          new Rotation2d(
              getChassisSpeeds().omegaRadiansPerSecond * SwerveConstants.skewOpenLoopFudgeFactor));
    } else {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds,
          new Rotation2d(speeds.omegaRadiansPerSecond * SwerveConstants.skewClosedLoopFudgeFactor));
    }
  }

  /**
   * Drives the robot relative to the field
   *
   * @param forward The forward velocity of the robot. Positive is going away from your alliance
   *     wall
   * @param strafe The sideways velocity of the robot. Positive is going to the right when you are
   *     standing behind the alliance wall
   * @param turn The angular velocity of the robot (CCW is +)
   */
  public void driveFieldOriented(double forward, double strafe, double turn) {
    driveFieldOriented(forward, strafe, turn, true, false, false);
  }

  /**
   * Drives the robot relative to the field
   *
   * @param forward The forward velocity of the robot. Positive is going away from your alliance
   *     wall
   * @param strafe The sideways velocity of the robot. Positive is going to the right when you are
   *     standing behind the alliance wall
   * @param turn The angular velocity of the robot (CCW is +)
   * @param fudgeFactor Weather or not to adjust the translation of the robot relative to the
   *     turning speed
   * @param isOpenLoop Weather the drive motors should be open loop
   */
  public void driveFieldOriented(
      double forward,
      double strafe,
      double turn,
      boolean fudgeFactor,
      boolean isOpenLoop,
      boolean allowTurnInPlace) {
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(forward, -strafe, turn, getHeading());
    setChassisSpeeds(chassisSpeeds, fudgeFactor, isOpenLoop, allowTurnInPlace);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setChassisSpeeds(speeds, true, false, false);
  }

  public void setChassisSpeeds(
      ChassisSpeeds speeds, boolean fudgeFactor, boolean isOpenLoop, boolean allowTurnInPlace) {
    speeds = ChassisSpeeds.discretize(speeds, 0.020);

    if (fudgeFactor && RobotBase.isReal()) {
      speeds = getFudgeFactoredSpeeds(speeds, isOpenLoop);
    }

    setModuleStates(kinematics.toSwerveModuleStates(speeds), isOpenLoop, allowTurnInPlace);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false, false);
  }

  public void setModuleStates(
      SwerveModuleState[] states, boolean isOpenLoop, boolean allowTurnInPlace) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxModuleSpeed);

    frontLeftModule.setState(states[0], isOpenLoop, allowTurnInPlace);
    frontRightModule.setState(states[1], isOpenLoop, allowTurnInPlace);
    backLeftModule.setState(states[2], isOpenLoop, allowTurnInPlace);
    backRightModule.setState(states[3], isOpenLoop, allowTurnInPlace);
  }

  public void lockModules() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        },
        false,
        true);
  }

  public void zeroYaw() {
    Pose2d originalOdometryPosition = getPose();

    if (SwerveConstants.zeroWithIntakeForward) {
      setHeading(Rotation2d.fromDegrees(180.0));
    } else {
      setHeading(Rotation2d.fromDegrees(0.0));
    }

    Rotation2d orientationOffset =
        SwerveConstants.zeroWithIntakeForward
            ? Rotation2d.fromDegrees(180.0)
            : Rotation2d.fromDegrees(0.0);

    odometryLock.lock();
    poseEstimator.resetPosition(
        getHeading(),
        getModulePositions(),
        new Pose2d(
            originalOdometryPosition.getTranslation(),
            AllianceUtil.getZeroRotation().plus(orientationOffset)));
    odometryLock.unlock();

    if (RobotBase.isSimulation()) {
      simOdometry.resetPosition(
          getHeading(),
          getModulePositions(),
          new Pose2d(
              originalOdometryPosition.getTranslation(),
              AllianceUtil.getZeroRotation().plus(orientationOffset)));
    }
  }

  public Rotation2d getRawHeading() {
    if (RobotBase.isReal()) {
      return navx.getRotation2d();
    } else {
      return Rotation2d.fromRadians(simYaw);
    }
  }

  public void setHeading(Rotation2d rotation) {
    odometryLock.lock();
    angleOffset = getRawHeading().minus(rotation);
    odometryLock.unlock();
  }

  @Log.NT(key = "Heading")
  public Rotation2d getHeading() {
    return getRawHeading().minus(angleOffset);
  }

  @Log.NT(key = "Rotation3d")
  public Rotation3d getHeading3d() {
    return navx.getRotation3d();
  }

  @Log.NT(key = "Chassis Speeds")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @Log.NT(key = "Field Relative Speeds")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getChassisSpeeds(), getHeading().plus(AllianceUtil.getZeroRotation()));
  }

  @Log.NT(key = "Estimated Pose")
  public Pose2d getPose() {
    odometryLock.lock();
    Pose2d pose = poseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    return pose;
  }

  @Log.NT(key = "Arducam Pose")
  public Pose3d getArducamPose() {
    return new Pose3d(getPose()).plus(VisionConstants.arducamTransform);
  }

  @Log.NT(key = "Limelight Pose")
  public Pose3d getLimelightPose() {
    return new Pose3d(getPose()).plus(VisionConstants.limelightTransform);
  }

  public Field2d getField() {
    return field;
  }

  public void resetPose(Pose2d pose) {
    setHeading(pose.getRotation().plus(AllianceUtil.getZeroRotation()));

    odometryLock.lock();
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    odometryLock.unlock();

    if (RobotBase.isSimulation()) {
      simOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }
  }

  @Log.NT(key = "Module Positions")
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getModulePosition(),
      frontRightModule.getModulePosition(),
      backLeftModule.getModulePosition(),
      backRightModule.getModulePosition(),
    };
  }

  @Log.NT(key = "Module States")
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getModuleState(),
      frontRightModule.getModuleState(),
      backLeftModule.getModuleState(),
      backRightModule.getModuleState(),
    };
  }

  @Log.NT(key = "Desired States")
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getDesiredState(),
      frontRightModule.getDesiredState(),
      backLeftModule.getDesiredState(),
      backRightModule.getDesiredState(),
    };
  }

  private boolean odometryUpdateValid(SwerveDriveWheelPositions positions, Rotation2d heading) {
    Twist2d twist = kinematics.toTwist2d(previousWheelPositions, positions);
    twist.dtheta = heading.minus(previousAngle).getRadians();

    odometryLock.lock();
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    Pose2d newPose = currentPose.exp(twist);

    previousAngle = heading;
    previousWheelPositions = positions;

    Translation2d distance = newPose.minus(currentPose).getTranslation();

    // It's literally impossible for the robot to move 1 meter in 20 milliseconds
    if (Math.abs(distance.getX()) > 1.0 || Math.abs(distance.getY()) > 1.0) {
      return false;
    }

    return true;
  }

  public void updateOdometry() {
    odometryLock.lock();
    Rotation2d heading = getHeading();
    SwerveDriveWheelPositions positions = new SwerveDriveWheelPositions(getModulePositions());

    if (!frontLeftModule.motorsValid()
        || !frontRightModule.motorsValid()
        || !backLeftModule.motorsValid()
        || !backRightModule.motorsValid()) {
      odometryLock.unlock();
      return;
    }
    odometryLock.unlock();

    if (!odometryUpdateValid(positions, heading)) {
      return;
    }

    odometryLock.lock();
    poseEstimator.update(heading, positions);
    odometryLock.unlock();
  }

  private Vector<N3> getLLStandardDeviations(
      Pose3d visionPose, Pose3d targetPose, int detectedTargets) {
    double distance = targetPose.getTranslation().toTranslation2d().getNorm();
    if (detectedTargets > 1) {
      return VecBuilder.fill(
          Units.inchesToMeters(2.25),
          Units.inchesToMeters(2.25),
          Units.degreesToRadians(1000000000.0));
    } else {
      double xyStandardDev = LimelightConstants.xyPolynomialRegression.predict(distance);
      double thetaStandardDev = LimelightConstants.thetaPolynomialRegression.predict(distance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, 10000000000.0 * thetaStandardDev * 4);
    }
  }

  private Vector<N3> getArducamStandardDeviations(
      Pose3d visionPose, Pose3d targetPose, int detectedTargets) {
    double distance = targetPose.getTranslation().toTranslation2d().getNorm();
    if (detectedTargets < 1) {
      return VecBuilder.fill(
          Units.inchesToMeters(6.5),
          Units.inchesToMeters(6.5),
          Units.degreesToRadians(10000000000000.0));
    } else {
      double xyStandardDev = ArducamConstants.xyPolynomialRegression.predict(distance);
      double thetaStandardDev = ArducamConstants.thetaPolynomialRegression.predict(distance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, 1000000000000.0 * thetaStandardDev * 4);
    }
  }

  private boolean isValidPose(
      Pose3d visionPose, Pose3d targetPose, int detectedTargets, double timestampSeconds) {
    double distance = targetPose.getTranslation().getNorm();

    if (distance > 6.5) {
      return false;
    }

    if (DriverStation.isAutonomous()) {
      if (detectedTargets < 2) {
        return false;
      }

      ChassisSpeeds currentSpeeds = getChassisSpeeds();

      double velocityTolerance = Units.inchesToMeters(1.5);

      if (Math.abs(currentSpeeds.vxMetersPerSecond) > velocityTolerance
          || Math.abs(currentSpeeds.vyMetersPerSecond) > velocityTolerance
          || Math.abs(currentSpeeds.omegaRadiansPerSecond) > Units.degreesToRadians(1.0)) {
        return false;
      }
    }

    if (distance > 3.5 && detectedTargets < 2) {
      return false;
    }

    // If it thinks the robot is out of field bounds
    if (visionPose.getX() < 0.0
        || visionPose.getX() > FieldConstants.aprilTagLayout.getFieldLength()
        || visionPose.getY() < 0.0
        || visionPose.getY() > FieldConstants.aprilTagLayout.getFieldWidth()
        || visionPose.getZ()
            < -0.15 // To account for minor inaccuracies in the LL location on the robot
        || visionPose.getZ() > 1.6) {
      return false;
    }

    double dt = Timer.getFPGATimestamp() - timestampSeconds;
    Rotation2d angleAtTime =
        getPose()
            .getRotation()
            .minus(new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond * dt));

    Rotation2d angleDifference = angleAtTime.minus(visionPose.getRotation().toRotation2d());

    double angleTolerance =
        DriverStation.isAutonomous() ? 3.5 : (detectedTargets >= 2) ? 20.0 : 10.0;

    // If the angle is too different from our gyro angle at the time of the image
    if (Math.abs(angleDifference.getDegrees()) > angleTolerance) {
      return false;
    }

    return true;
  }

  private void updateLimelightPoses(String limelightName) {
    if (!LimelightHelpers.getTV(limelightName)) {
      return;
    }
    LimelightHelpers.Results results =
        LimelightHelpers.getLatestResults(limelightName).targetingResults;
    LimelightHelpers.PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (!results.valid) {
      return;
    }

    // Old data
    if (limelightLastDetectedTime == results.timestamp_LIMELIGHT_publish) {
      return;
    }
    limelightLastDetectedTime = results.timestamp_LIMELIGHT_publish;

    LimelightTarget_Fiducial[] detectedTags = results.targets_Fiducials;

    if (detectedTags.length == 0) {
      return;
    }

    LimelightTarget_Fiducial closestTag = detectedTags[0];
    Pose3d closestTagPose = closestTag.getCameraPose_TargetSpace();
    Pose3d visionPose = results.getBotPose3d_wpiBlue();

    double timestamp = poseEstimate.timestampSeconds;

    if (!isValidPose(visionPose, closestTagPose, detectedTags.length, timestamp)) {
      rejectedPoses.add(visionPose);
      return;
    }

    Vector<N3> standardDevs =
        getLLStandardDeviations(visionPose, closestTagPose, detectedTags.length);

    poseEstimates.add(new PoseEstimate(visionPose, timestamp, standardDevs));

    for (LimelightTarget_Fiducial target : detectedTags) {
      int tagID = (int) target.fiducialID;

      Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(tagID);

      if (tagPose.isPresent()) {
        detectedTargets.add(tagPose.get());
      }
    }
  }

  private void updatePhotonVisionPoses() {
    PhotonPipelineResult result = arducam.getLatestResult();

    if (!result.hasTargets()) {
      return;
    }

    odometryLock.lock();
    arducamPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
    odometryLock.unlock();

    Optional<EstimatedRobotPose> optionalVisionPose = arducamPoseEstimator.update(result);
    if (optionalVisionPose.isEmpty()) {
      return;
    }

    EstimatedRobotPose visionPose = optionalVisionPose.get();

    Pose3d closestTargetPose =
        new Pose3d(
            result.getBestTarget().getBestCameraToTarget().getTranslation(),
            result.getBestTarget().getBestCameraToTarget().getRotation());

    if (!isValidPose(
        visionPose.estimatedPose,
        closestTargetPose,
        visionPose.targetsUsed.size(),
        visionPose.timestampSeconds)) {
      rejectedPoses.add(visionPose.estimatedPose);
      return;
    }

    Vector<N3> standardDevs =
        getArducamStandardDeviations(
            visionPose.estimatedPose, closestTargetPose, visionPose.targetsUsed.size());

    poseEstimates.add(
        new PoseEstimate(visionPose.estimatedPose, visionPose.timestampSeconds, standardDevs));

    for (PhotonTrackedTarget target : visionPose.targetsUsed) {
      int aprilTagID = target.getFiducialId();

      Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(aprilTagID);
      if (tagPose.isEmpty()) {
        continue;
      }

      detectedTargets.add(tagPose.get());
    }
  }

  public Pose2d getNearestChain(List<Pose2d> chainPoses) {
    return getPose().nearest(chainPoses);
  }

  public Pose2d getCenterChainPose() {
    return getNearestChain(CenterChainPoses.poses);
  }

  public Pose2d getRightChainPose() {
    return getNearestChain(RightChainPoses.poses);
  }

  public Pose2d getLeftChainPose() {
    return getNearestChain(LeftChainPoses.poses);
  }

  public void updateVisionPoseEstimates() {
    poseEstimates.clear();
    detectedTargets.clear();
    rejectedPoses.clear();

    updateLimelightPoses(VisionConstants.limelightName);
    updatePhotonVisionPoses();

    Collections.sort(poseEstimates);

    odometryLock.lock();
    for (PoseEstimate poseEstimate : poseEstimates) {
      poseEstimator.addVisionMeasurement(
          poseEstimate.estimatedPose().toPose2d(),
          poseEstimate.timestamp(),
          poseEstimate.standardDevs());
    }
    odometryLock.unlock();

    log("Detected Tags", detectedTargets.toArray(Pose3d[]::new));
    log("Rejected Poses", rejectedPoses.toArray(Pose3d[]::new));

    field
        .getObject("Detected Targets")
        .setPoses(detectedTargets.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
    field
        .getObject("Rejected Poses")
        .setPoses(rejectedPoses.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.periodic();
    frontRightModule.periodic();
    backLeftModule.periodic();
    backRightModule.periodic();

    updateVisionPoseEstimates();

    odometryLock.lock();
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    odometryLock.unlock();

    SmartDashboard.putNumber(
        "Distance to Speaker",
        getPose().minus(AllianceUtil.getSpeakerPose()).getTranslation().getNorm());
  }

  @Override
  public void simulationPeriodic() {
    frontLeftModule.simulationPeriodic();
    frontRightModule.simulationPeriodic();
    backLeftModule.simulationPeriodic();
    backRightModule.simulationPeriodic();

    simYaw += getChassisSpeeds().omegaRadiansPerSecond * 0.020;

    simOdometry.update(getHeading(), getModulePositions());
    visionSim.update(simOdometry.getPoseMeters());

    if (DriverStation.isDisabled()) {
      setChassisSpeeds(new ChassisSpeeds());
    }
  }

  @Override
  public Command getPrematchCheckCommand(
      VirtualXboxController controller, VirtualJoystick joystick) {
    return Commands.sequence(
        // Make sure gyro is connected
        Commands.runOnce(
            () -> {
              if (!navx.isConnected()) {
                addError("NavX is not connected");
              } else {
                addInfo("NavX is connected");
              }
            }),
        // Test gyro zeroing
        Commands.runOnce(
            () -> {
              controller.setStartButton(true);
              controller.setBackButton(true);
            }),
        Commands.waitSeconds(0.2),
        Commands.runOnce(
            () -> {
              if (Math.abs(getHeading().getDegrees()) > 0.05) {
                addError("Gyro failed to zero");
              } else {
                addInfo("Gyro zero successful");
              }

              controller.clearVirtualButtons();
            }),
        // Test all modules
        Commands.sequence(
            frontLeftModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            frontRightModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            backLeftModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError),
            backRightModule.getPrematchCommand(this::addInfo, this::addWarning, this::addError)),
        // Test forward speed
        Commands.runOnce(
            () -> {
              controller.setLeftY(-1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().vxMetersPerSecond < Units.feetToMeters(10.0)) {
                addError("Forward speed too slow");
              } else if (Math.abs(getChassisSpeeds().vyMetersPerSecond)
                  > prematchTranslationalTolerance) {
                addError("Strafe speed too high");
              } else {
                addInfo("Forward drive successful");
              }

              controller.clearVirtualAxes();
            }),
        // Test slowing down to 0 m/s
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (Math.abs(getChassisSpeeds().vxMetersPerSecond) > Units.feetToMeters(0.1)
                  || Math.abs(getChassisSpeeds().vyMetersPerSecond) > Units.feetToMeters(0.1)) {
                addError("Robot moving too fast");
              } else {
                addInfo("Slow down successful");
              }

              controller.clearVirtualAxes();
            }),
        // Test backward speed
        Commands.runOnce(
            () -> {
              controller.setLeftY(1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().vxMetersPerSecond > Units.feetToMeters(-10.0)) {
                addError("Backward speed too slow");
              } else if (Math.abs(getChassisSpeeds().vyMetersPerSecond)
                  > prematchTranslationalTolerance) {
                addError("Strafe speed too high");
              } else {
                addInfo("Backward drive successful");
              }

              controller.clearVirtualAxes();
            }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test left speed
        Commands.runOnce(
            () -> {
              controller.setLeftX(-1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().vyMetersPerSecond < Units.feetToMeters(10.0)) {
                addError("Left speed too slow");
              } else if (Math.abs(getChassisSpeeds().vxMetersPerSecond)
                  > prematchTranslationalTolerance) {
                addError("Forward/Backward speed too high");
              } else {
                addInfo("Left drive sucessful");
              }

              controller.clearVirtualAxes();
            }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test right speed
        Commands.runOnce(
            () -> {
              controller.setLeftX(1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().vyMetersPerSecond > Units.feetToMeters(-10.0)) {
                addError("Right speed too slow");
              } else if (Math.abs(getChassisSpeeds().vxMetersPerSecond)
                  > prematchTranslationalTolerance) {
                addError("Forward/Backward speed too high");
              } else {
                addInfo("Right drive successful");
              }

              controller.clearVirtualAxes();
            }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test angular CW speed
        Commands.runOnce(
            () -> {
              controller.setRightX(1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().omegaRadiansPerSecond > Units.degreesToRadians(-160.0)) {
                addError("Clockwise rotation too slow");
              } else {
                addInfo("Clockwise rotation successful");
              }

              controller.clearVirtualAxes();
            }),
        Commands.waitSeconds(prematchDriveDelay),
        // Test angular CCW speed
        Commands.runOnce(
            () -> {
              controller.setRightX(-1.0);
            }),
        Commands.waitSeconds(prematchDriveDelay),
        Commands.runOnce(
            () -> {
              if (getChassisSpeeds().omegaRadiansPerSecond < Units.degreesToRadians(160.0)) {
                addError("Counter Clockwise rotation too slow");
              } else {
                addInfo("Counter Clockwise rotation successful");
              }

              controller.clearVirtualAxes();
            }));
  }

  public Command quasistaticForward() {
    return Commands.sequence(
            runOnce(
                () -> {
                  frontLeftModule.setCharacterizationVolts(0.0);
                  frontRightModule.setCharacterizationVolts(0.0);
                  backLeftModule.setCharacterizationVolts(0.0);
                  backRightModule.setCharacterizationVolts(0.0);
                }),
            Commands.waitSeconds(0.50),
            sysIdRoutine.quasistatic(Direction.kForward))
        .finallyDo(
            () -> {
              frontLeftModule.stopCharacterizing();
              frontRightModule.stopCharacterizing();
              backLeftModule.stopCharacterizing();
              backRightModule.stopCharacterizing();
            });
  }

  public Command quasistaticBackward() {
    return Commands.sequence(
            runOnce(
                () -> {
                  frontLeftModule.setCharacterizationVolts(0.0);
                  frontRightModule.setCharacterizationVolts(0.0);
                  backLeftModule.setCharacterizationVolts(0.0);
                  backRightModule.setCharacterizationVolts(0.0);
                }),
            Commands.waitSeconds(0.50),
            sysIdRoutine.quasistatic(Direction.kReverse))
        .finallyDo(
            () -> {
              frontLeftModule.stopCharacterizing();
              frontRightModule.stopCharacterizing();
              backLeftModule.stopCharacterizing();
              backRightModule.stopCharacterizing();
            });
  }

  public Command dynamicForward() {
    return Commands.sequence(
            runOnce(
                () -> {
                  frontLeftModule.setCharacterizationVolts(0.0);
                  frontRightModule.setCharacterizationVolts(0.0);
                  backLeftModule.setCharacterizationVolts(0.0);
                  backRightModule.setCharacterizationVolts(0.0);
                }),
            Commands.waitSeconds(0.50),
            sysIdRoutine.dynamic(Direction.kForward))
        .finallyDo(
            () -> {
              frontLeftModule.stopCharacterizing();
              frontRightModule.stopCharacterizing();
              backLeftModule.stopCharacterizing();
              backRightModule.stopCharacterizing();
            });
  }

  public Command dynamicBackward() {
    return Commands.sequence(
            runOnce(
                () -> {
                  frontLeftModule.setCharacterizationVolts(0.0);
                  frontRightModule.setCharacterizationVolts(0.0);
                  backLeftModule.setCharacterizationVolts(0.0);
                  backRightModule.setCharacterizationVolts(0.0);
                }),
            Commands.waitSeconds(0.50),
            sysIdRoutine.dynamic(Direction.kReverse))
        .finallyDo(
            () -> {
              frontLeftModule.stopCharacterizing();
              frontRightModule.stopCharacterizing();
              backLeftModule.stopCharacterizing();
              backRightModule.stopCharacterizing();
            });
  }
}
