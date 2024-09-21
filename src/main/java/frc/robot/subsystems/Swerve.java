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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
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
import frc.robot.util.FaultLogger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
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
  // New
  private Pose2d zeroPosition = new Pose2d(0, 0, new Rotation2d());
  public final double MIN_X = -7.5;
  public final double MIN_Y = -5.0;
  public final double MAX_X = 7.5;
  public final double MAX_Y = 5.0;

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

  public static record OdometryRecord(
      SwerveDriveWheelPositions positions, Rotation2d heading, double timestamp, boolean valid) {}

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

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  @Log.NT(key = "Angle Offset")
  private Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);

  public static final ReadWriteLock odometryLock = new ReentrantReadWriteLock();

  private Queue<OdometryRecord> odometryRecordQueue = new ArrayBlockingQueue<>(20);

  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveOdometry simOdometry;

  private TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
      TimeInterpolatableBuffer.createBuffer(1.5);

  private SwerveDriveWheelPositions previousWheelPositions =
      new SwerveDriveWheelPositions(getModulePositions());
  private Rotation2d previousAngle = Rotation2d.fromDegrees(0.0);

  private int odometryUpdateCount = 0;
  private int odometryRejectCount = 0;

  private PhotonCamera arducam = new PhotonCamera(VisionConstants.arducamName);
  private PhotonPoseEstimator arducamPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.arducamTransform);

  private PhotonCamera limelight = new PhotonCamera(VisionConstants.limelightName);
  private PhotonPoseEstimator limelightPoseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.aprilTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.limelightTransform);

  private PhotonPipelineResult latestArducamResult;
  private PhotonPipelineResult latestLimelightResult;

  private Optional<PhotonTrackedTarget> centerSpeakerTarget = Optional.empty();

  // Temporary fix for inaccurate poses while auto shooting
  private boolean ignoreArducam = false;

  private PhotonCameraSim arducamSim;
  private PhotonCameraSim limelightSim;
  private VisionSystemSim visionSim;

  private List<Pose3d> detectedTargets = new ArrayList<>();
  private List<Pose3d> rejectedPoses = new ArrayList<>();
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

  private Notifier odometryThread;

  /** Creates a new Swerve. */
  public Swerve() {
    DataLogManager.log("[Swerve] Initializing");

    PhotonCamera.setVersionCheckEnabled(false);

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRawHeading(),
            getModulePositions(),
            new Pose2d(0.0, 0.0, getRawHeading()));
    arducamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    limelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    odometryThread = new Notifier(this::updateOdometry);
    odometryThread.setName("OdometryThread");
    odometryThread.startPeriodic(1.0 / SwerveConstants.odometryUpdateFrequency);

    if (RobotBase.isSimulation()) {
      simOdometry = new SwerveDriveOdometry(kinematics, getRawHeading(), getModulePositions());
      visionSim = new VisionSystemSim("main");

      visionSim.addAprilTags(FieldConstants.aprilTagLayout);

      SimCameraProperties arducamProperties = new SimCameraProperties();
      arducamProperties.setCalibration(800, 600, Rotation2d.fromDegrees(68.97));
      arducamProperties.setCalibError(0.21, 0.10);
      arducamProperties.setFPS(28);
      arducamProperties.setAvgLatencyMs(36);
      arducamProperties.setLatencyStdDevMs(15);
      arducamProperties.setExposureTimeMs(45);

      arducamSim = new PhotonCameraSim(arducam, arducamProperties);
      visionSim.addCamera(arducamSim, VisionConstants.arducamTransform);

      arducamSim.enableRawStream(false);
      arducamSim.enableProcessedStream(false);

      SimCameraProperties limelightProperties = new SimCameraProperties();
      limelightProperties.setCalibration(960, 720, Rotation2d.fromDegrees(75.67));
      limelightProperties.setCalibError(0.21, 0.10);
      limelightProperties.setFPS(30);
      limelightProperties.setAvgLatencyMs(36);
      limelightProperties.setLatencyStdDevMs(15);
      limelightProperties.setExposureTimeMs(45);

      limelightSim = new PhotonCameraSim(limelight, limelightProperties);
      visionSim.addCamera(limelightSim, VisionConstants.limelightTransform);

      limelightSim.enableRawStream(false);
      limelightSim.enableProcessedStream(false);
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
            setChassisSpeeds(new ChassisSpeeds());
          }
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> field.getObject("Target Pose").setPose(pose));

    SmartDashboard.putData("Swerve/Field", field);

    SmartDashboard.putData(
        "Swerve/NavX Accelerometer",
        builder -> {
          builder.setSmartDashboardType("3AxisAccelerometer");
          builder.addDoubleProperty("X", navx::getWorldLinearAccelX, null);
          builder.addDoubleProperty("Y", navx::getWorldLinearAccelY, null);
          builder.addDoubleProperty("Z", navx::getWorldLinearAccelZ, null);
        });

    SmartDashboard.putData("Swerve/Built-in Accelerometer", new BuiltInAccelerometer());

    SmartDashboard.putData(
        "Swerve/Swerve Drive",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> frontLeftModule.getAngle().getRadians(), null);
          builder.addDoubleProperty("Front Left Velocity", frontLeftModule::getVelocity, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> frontRightModule.getAngle().getRadians(), null);
          builder.addDoubleProperty("Front Right Velocity", frontRightModule::getVelocity, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> backLeftModule.getAngle().getRadians(), null);
          builder.addDoubleProperty("Back Left Velocity", backLeftModule::getVelocity, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> backRightModule.getAngle().getRadians(), null);
          builder.addDoubleProperty("Back Right Velocity", backRightModule::getVelocity, null);

          builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
        });

    SmartDashboard.putData(
        "Swerve/Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
        });

    Commands.sequence(Commands.waitSeconds(2.0), runOnce(this::resetModulesToAbsolute))
        .ignoringDisable(true)
        .schedule();

    FaultLogger.register(navx);
    FaultLogger.register(arducam);
    FaultLogger.register(limelight);

    DataLogManager.log("[Swerve] Initialization Complete");
  }

  public void resetModulesToAbsolute() {
    DataLogManager.log("[Swerve] Resetting modules to absolute");
    Pose2d originalPose = getPose();

    odometryLock.writeLock().lock();
    frontLeftModule.resetToAbsolute();
    frontRightModule.resetToAbsolute();
    backLeftModule.resetToAbsolute();
    backRightModule.resetToAbsolute();

    poseEstimator.resetPosition(getRawHeading(), getModulePositions(), originalPose);
    odometryLock.writeLock().unlock();
  }

  @Log.NT
  public boolean arducamConnected() {
    return arducam.isConnected();
  }

  @Log.NT
  public boolean limelightConnected() {
    return limelight.isConnected();
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

    poseEstimator.resetPosition(
        getRawHeading(),
        getModulePositions(),
        new Pose2d(
            originalOdometryPosition.getTranslation(),
            AllianceUtil.getZeroRotation().plus(orientationOffset)));

    if (RobotBase.isSimulation()) {
      simOdometry.resetPosition(
          getRawHeading(),
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
    angleOffset = getRawHeading().minus(rotation);
  }

  @Log.NT(key = "Heading")
  public Rotation2d getHeading() {
    return getRawHeading().minus(angleOffset);
  }

  @Log.NT(key = "Rotation3d")
  public Rotation3d getHeading3d() {
    return navx.getRotation3d();
  }

  public double getYawRadians() {
    return Units.degreesToRadians(navx.getYaw());
  }

  public Optional<Rotation2d> getRotationAtTime(double time) {
    Optional<Rotation2d> rotationAtTime = rotationBuffer.getSample(time);
    return rotationAtTime;
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
    return poseEstimator.getEstimatedPosition();
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

    rotationBuffer.clear();
    poseEstimator.resetPosition(getRawHeading(), getModulePositions(), pose);

    if (RobotBase.isSimulation()) {
      simOdometry.resetPosition(getRawHeading(), getModulePositions(), pose);
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

  // New
  Pose2d currentPose = getPose();
  double xCoordinate = currentPose.getX();
  double yCoordinate = currentPose.getY();

  public boolean isNearPracticeOutOfBounds() {
    if (xCoordinate >= MAX_X - 1) {
      return true;
    } else if (xCoordinate <= MIN_X + 1) {
      return true;
    } else if (yCoordinate >= MAX_Y - 1) {
      return true;
    } else if (yCoordinate <= MIN_Y + 1) {
      return true;
    }
    return false;
  }

  private boolean odometryUpdateValid(Twist2d delta) {
    Pose2d poseExponential = new Pose2d().exp(delta);

    Translation2d distance = poseExponential.getTranslation();

    // It's literally impossible for the robot to move 1 meter in 20 milliseconds
    if (Math.abs(distance.getX()) > 1.0 || Math.abs(distance.getY()) > 1.0) {
      return false;
    }

    return true;
  }

  private void updateOdometry() {
    double timestamp = Timer.getFPGATimestamp();

    Rotation2d heading = getRawHeading();
    SwerveDriveWheelPositions positions = new SwerveDriveWheelPositions(getModulePositions());

    Twist2d twist = kinematics.toTwist2d(previousWheelPositions, positions);
    twist.dtheta = heading.minus(previousAngle).getRadians();

    previousAngle = heading;
    previousWheelPositions = positions;

    boolean valid =
        frontLeftModule.motorsValid()
            && frontRightModule.motorsValid()
            && backLeftModule.motorsValid()
            && backRightModule.motorsValid();

    valid = valid && odometryUpdateValid(twist);

    odometryLock.writeLock().lock();
    odometryRecordQueue.offer(new OdometryRecord(positions, heading, timestamp, valid));
    odometryLock.writeLock().unlock();
  }

  private Vector<N3> getLLStandardDeviations(
      EstimatedRobotPose robotPose, double averageDistance, int tagCount) {
    double stdDevScale = 1 + (averageDistance * averageDistance / 30);

    // If we're semi-close to it, use polynomial regression or a super low stdandard dev
    if (tagCount > 1) {
      Vector<N3> standardDeviation =
          VecBuilder.fill(Units.inchesToMeters(2.5), Units.inchesToMeters(2.5), Double.MAX_VALUE);

      return standardDeviation.times(stdDevScale);
    } else {
      double xyStandardDev = LimelightConstants.xyPolynomialRegression.predict(averageDistance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, Double.MAX_VALUE);
    }
  }

  private Vector<N3> getArducamStandardDeviations(
      EstimatedRobotPose robotPose, double averageDistance, int tagCount) {
    double stdDevScale = 1 + (averageDistance * averageDistance / 30);
    if (tagCount > 1) {
      Vector<N3> standardDeviation =
          VecBuilder.fill(Units.inchesToMeters(4.5), Units.inchesToMeters(4.5), Double.MAX_VALUE);

      return standardDeviation.times(stdDevScale);
    } else {
      double xyStandardDev = ArducamConstants.xyPolynomialRegression.predict(averageDistance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, Double.MAX_VALUE);
    }
  }

  private boolean isValidPose(
      Pose3d visionPose, double averageDistance, int detectedTargets, double timestampSeconds) {
    if (averageDistance > 6.5) {
      return false;
    }

    if (DriverStation.isAutonomous()) {
      return false;
      // if (detectedTargets < 2) {
      //   return false;
      // }

      // ChassisSpeeds currentSpeeds = getChassisSpeeds();

      // double velocityTolerance = Units.inchesToMeters(1.5);

      // if (Math.abs(currentSpeeds.vxMetersPerSecond) > velocityTolerance
      //     || Math.abs(currentSpeeds.vyMetersPerSecond) > velocityTolerance
      //     || Math.abs(currentSpeeds.omegaRadiansPerSecond) > Units.degreesToRadians(1.0)) {
      //   return false;
      // }
    }

    if (averageDistance > 4.0 && detectedTargets < 2) {
      return false;
    }

    // If it thinks the robot is out of field bounds
    if (visionPose.getX() < 0.0
        || visionPose.getX() > FieldConstants.aprilTagLayout.getFieldLength()
        || visionPose.getY() < 0.0
        || visionPose.getY() > FieldConstants.aprilTagLayout.getFieldWidth()
        || visionPose.getZ()
            < -0.5 // To account for minor inaccuracies in the camera location on the robot
        || visionPose.getZ() > 1.6) {
      return false;
    }

    Optional<Rotation2d> angleAtTime = getRotationAtTime(timestampSeconds);
    if (angleAtTime.isEmpty()) {
      angleAtTime = Optional.of(getPose().getRotation());
    }

    Rotation2d angleDifference = angleAtTime.get().minus(visionPose.getRotation().toRotation2d());

    double angleTolerance =
        DriverStation.isAutonomous() ? 8.0 : (detectedTargets >= 2) ? 25.0 : 15.0;

    // If the angle is too different from our gyro angle at the time of the image
    if (Math.abs(angleDifference.getDegrees()) > angleTolerance) {
      return false;
    }

    return true;
  }

  private void updateLimelightPoses() {
    if (!latestLimelightResult.hasTargets()) {
      centerSpeakerTarget = Optional.empty();
      return;
    }

    limelightPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());

    Optional<EstimatedRobotPose> optionalVisionPose =
        limelightPoseEstimator.update(latestLimelightResult);
    if (optionalVisionPose.isEmpty()) {
      return;
    }

    EstimatedRobotPose visionPose = optionalVisionPose.get();

    double totalDistance = 0.0;
    int tagCount = 0;

    boolean seenCenterTag = false;
    int centerSpeakerID = AllianceUtil.getCenterSpeakerID();
    for (PhotonTrackedTarget target : visionPose.targetsUsed) {
      tagCount++;
      totalDistance += target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();

      if (target.getFiducialId() == centerSpeakerID) {
        centerSpeakerTarget = Optional.of(target);
        seenCenterTag = true;
      }
    }
    if (!seenCenterTag) {
      centerSpeakerTarget = Optional.empty();
    }

    double averageDistance = totalDistance / tagCount;

    if (!isValidPose(
        visionPose.estimatedPose,
        averageDistance,
        visionPose.targetsUsed.size(),
        visionPose.timestampSeconds)) {
      rejectedPoses.add(visionPose.estimatedPose);
      return;
    }

    Vector<N3> standardDevs =
        getLLStandardDeviations(visionPose, averageDistance, visionPose.targetsUsed.size());

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

  private void updateArducamPoses() {
    if (!latestArducamResult.hasTargets()) {
      return;
    }

    if (ignoreArducam) {
      return;
    }

    arducamPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());

    Optional<EstimatedRobotPose> optionalVisionPose =
        arducamPoseEstimator.update(latestArducamResult);
    if (optionalVisionPose.isEmpty()) {
      return;
    }

    EstimatedRobotPose visionPose = optionalVisionPose.get();

    double totalDistance = 0.0;
    int tagCount = 0;
    for (PhotonTrackedTarget target : visionPose.targetsUsed) {
      tagCount++;
      totalDistance += target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    double averageDistance = totalDistance / tagCount;

    if (!isValidPose(
        visionPose.estimatedPose,
        averageDistance,
        visionPose.targetsUsed.size(),
        visionPose.timestampSeconds)) {
      rejectedPoses.add(visionPose.estimatedPose);
      return;
    }

    Vector<N3> standardDevs =
        getArducamStandardDeviations(visionPose, averageDistance, visionPose.targetsUsed.size());

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
    if (AllianceUtil.isRedAlliance()) {
      return getNearestChain(CenterChainPoses.redSidePoses);
    } else {
      return getNearestChain(CenterChainPoses.blueSidePoses);
    }
  }

  public Pose2d getRightChainPose() {
    if (AllianceUtil.isRedAlliance()) {
      return getNearestChain(RightChainPoses.redSidePoses);
    } else {
      return getNearestChain(RightChainPoses.blueSidePoses);
    }
  }

  public Pose2d getLeftChainPose() {
    if (AllianceUtil.isRedAlliance()) {
      return getNearestChain(LeftChainPoses.redSidePoses);
    } else {
      return getNearestChain(LeftChainPoses.blueSidePoses);
    }
  }

  public void updateVisionPoseEstimates() {
    poseEstimates.clear();
    detectedTargets.clear();
    rejectedPoses.clear();

    updateLimelightPoses();
    updateArducamPoses();

    Collections.sort(poseEstimates);

    for (PoseEstimate poseEstimate : poseEstimates) {
      poseEstimator.addVisionMeasurement(
          poseEstimate.estimatedPose().toPose2d(),
          poseEstimate.timestamp(),
          poseEstimate.standardDevs());
    }

    log("Detected Tags", detectedTargets.toArray(Pose3d[]::new));
    log("Rejected Poses", rejectedPoses.toArray(Pose3d[]::new));

    field
        .getObject("Detected Targets")
        .setPoses(detectedTargets.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
    field
        .getObject("Rejected Poses")
        .setPoses(rejectedPoses.stream().map(p -> p.toPose2d()).toArray(Pose2d[]::new));
  }

  public void setIgnoreArducam(boolean ignore) {
    ignoreArducam = ignore;
  }

  public double getSpeakerDistance() {
    if (centerSpeakerTarget.isPresent()) {
      Transform3d transform = centerSpeakerTarget.get().getBestCameraToTarget();

      Pose3d pose =
          new Pose3d()
              .transformBy(transform.inverse())
              .transformBy(VisionConstants.limelightTransform.inverse());

      return pose.getTranslation().toTranslation2d().getNorm();
    } else {
      return getPose().minus(AllianceUtil.getSpeakerPose()).getTranslation().getNorm();
    }
  }

  public PhotonPipelineResult getLimelightResults() {
    return latestLimelightResult;
  }

  public PhotonPipelineResult getArducamResults() {
    return latestArducamResult;
  }

  public Optional<PhotonTrackedTarget> getCenterSpeakerTarget() {
    return centerSpeakerTarget;
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return new double[] {
      frontLeftModule.getPositionRadians(),
      frontRightModule.getPositionRadians(),
      backLeftModule.getPositionRadians(),
      backRightModule.getPositionRadians()
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.periodic();
    frontRightModule.periodic();
    backLeftModule.periodic();
    backRightModule.periodic();

    SmartDashboard.putNumber("Odometry Queue Size", odometryRecordQueue.size());

    odometryLock.writeLock().lock();
    while (!odometryRecordQueue.isEmpty()) {
      odometryUpdateCount++;
      OdometryRecord odometryRecord = odometryRecordQueue.poll();
      if (!odometryRecord.valid()) {
        odometryRejectCount++;
        continue;
      }
      Pose2d newPose =
          poseEstimator.updateWithTime(
              odometryRecord.timestamp(), odometryRecord.heading(), odometryRecord.positions());
      rotationBuffer.addSample(odometryRecord.timestamp(), newPose.getRotation());
      if (RobotBase.isSimulation()) {
        simOdometry.update(odometryRecord.heading(), odometryRecord.positions());
      }
    }
    odometryLock.writeLock().unlock();

    latestArducamResult = arducam.getLatestResult();
    latestLimelightResult = limelight.getLatestResult();

    updateVisionPoseEstimates();

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Distance to Speaker", getSpeakerDistance());

    log("odometryUpdateCount", odometryUpdateCount);
    log("odometryRejectCount", odometryRejectCount);
    log(
        "Odometry Update %",
        ((double) (odometryUpdateCount - odometryRejectCount) / odometryUpdateCount) * 100.0);
  }

  // New
  public void setZeroPosition() {
    poseEstimator.resetPosition(getRawHeading(), getModulePositions(), zeroPosition);
  }

  @Override
  public void simulationPeriodic() {
    frontLeftModule.simulationPeriodic();
    frontRightModule.simulationPeriodic();
    backLeftModule.simulationPeriodic();
    backRightModule.simulationPeriodic();

    simYaw += getChassisSpeeds().omegaRadiansPerSecond * 0.020;

    // simOdometry.update(getRawHeading(), getModulePositions());
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
              Rotation2d rotationOffset =
                  (SwerveConstants.zeroWithIntakeForward)
                      ? Rotation2d.fromDegrees(180.0)
                      : Rotation2d.fromDegrees(0.0);
              if (Math.abs(getHeading().plus(rotationOffset).getDegrees()) > 0.05) {
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
              double forwardSpeed = getChassisSpeeds().vxMetersPerSecond;
              double forwardSpeedSign = Math.signum(forwardSpeed);
              double desiredSign = (SwerveConstants.zeroWithIntakeForward) ? -1.0 : 1.0;

              if (Math.abs(forwardSpeed) < Units.feetToMeters(10.0)
                  && forwardSpeedSign == desiredSign) {
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
        Commands.waitSeconds(1.35),
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
              double backwardSpeed = getChassisSpeeds().vxMetersPerSecond;
              double backwardSpeedSign = Math.signum(backwardSpeed);
              double desiredSign = (SwerveConstants.zeroWithIntakeForward) ? 1.0 : -1.0;

              if (Math.abs(backwardSpeed) < Units.feetToMeters(10.0)
                  && backwardSpeedSign == desiredSign) {
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
              double strafeSpeed = getChassisSpeeds().vyMetersPerSecond;
              double strafeSign = Math.signum(strafeSpeed);
              double desiredSign = (SwerveConstants.zeroWithIntakeForward) ? 1.0 : -1.0;

              if (Math.abs(strafeSpeed) < Units.feetToMeters(10.0) && strafeSign == desiredSign) {
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
              double strafeSpeed = getChassisSpeeds().vyMetersPerSecond;
              double strafeSign = Math.signum(strafeSpeed);
              double desiredSign = (SwerveConstants.zeroWithIntakeForward) ? -1.0 : 1.0;

              if (Math.abs(strafeSpeed) > Units.feetToMeters(10.0) && strafeSign == desiredSign) {
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
