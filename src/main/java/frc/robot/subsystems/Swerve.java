// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.util.AllianceUtil;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Swerve extends SubsystemBase {
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

  private AHRS navx =
      new AHRS(
          Port.kUSB, SerialDataType.kProcessedData, (byte) SwerveConstants.odometryUpdateFrequency);
  private Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);

  private SwerveDrivePoseEstimator poseEstimator;

  private double limelightLastDetectedTime = 0.0;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                frontLeftModule.setCharacterizationVolts(volts.in(Volts));
                backLeftModule.setCharacterizationVolts(volts.in(Volts));

                frontRightModule.setCharacterizationVolts(volts.in(Volts));
                backRightModule.setCharacterizationVolts(volts.in(Volts));
              },
              null,
              this));

  private Field2d field = new Field2d();

  /** Creates a new Swerve. */
  public Swerve() {
    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, getHeading(), getModulePositions(), new Pose2d());

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

    PathPlannerLogging.setLogActivePathCallback(
        path -> {
          field.getObject("trajectory").setPoses(path);
          if (path.size() == 0) {
            field.getObject("Target Pose").setPoses(path);
            setChassisSpeeds(new ChassisSpeeds(), false, false, false);
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

    DataLogManager.log("NavX Firmware: " + navx.getFirmwareVersion());

    Timer.delay(1.00);

    frontLeftModule.resetToAbsolute();
    frontRightModule.resetToAbsolute();
    backLeftModule.resetToAbsolute();
    backRightModule.resetToAbsolute();
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

    if (fudgeFactor) {
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
    Pose2d originalOdometryPosition = poseEstimator.getEstimatedPosition();

    setHeading(Rotation2d.fromDegrees(0.0));

    poseEstimator.resetPosition(
        getHeading(),
        getModulePositions(),
        new Pose2d(originalOdometryPosition.getTranslation(), AllianceUtil.getZeroRotation()));
  }

  public void setHeading(Rotation2d rotation) {
    angleOffset = navx.getRotation2d().minus(rotation);
  }

  public Rotation2d getHeading() {
    return navx.getRotation2d().minus(angleOffset);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    setHeading(pose.getRotation());

    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getModulePosition(),
      frontRightModule.getModulePosition(),
      backLeftModule.getModulePosition(),
      backRightModule.getModulePosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getModuleState(),
      frontRightModule.getModuleState(),
      backLeftModule.getModuleState(),
      backRightModule.getModuleState()
    };
  }

  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getDesiredState(),
      frontRightModule.getDesiredState(),
      backLeftModule.getDesiredState(),
      backRightModule.getDesiredState()
    };
  }

  public void updateOdometry() {
    poseEstimator.update(getHeading(), getModulePositions());
  }

  private Vector<N3> getLLStandardDeviations(
      Pose3d visionPose, Pose3d targetPose, int detectedTargets) {
    double distance = targetPose.getTranslation().toTranslation2d().getNorm();
    if (detectedTargets > 1) {
      return VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10.0));
    } else {
      double xyStandardDev = LimelightConstants.xyPolynomialRegression.predict(distance);
      double thetaStandardDev = LimelightConstants.thetaPolynomialRegression.predict(distance);

      return VecBuilder.fill(xyStandardDev, xyStandardDev, thetaStandardDev * 4);
    }
  }

  private boolean isValidPose(Pose3d visionPose, Pose3d targetPose, int detectedTargets) {
    if (targetPose.getTranslation().getNorm() > 3.0) {
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

    Rotation2d angleDifference = getHeading().minus(visionPose.getRotation().toRotation2d());

    // If the angle is too different from our gyro angle
    if (Math.abs(MathUtil.inputModulus(angleDifference.getDegrees(), -180.0, 180.0)) > 20.0) {
      return false;
    }

    return true;
  }

  private void updateLimelightPoses(List<Pose2d> poses) {
    LimelightHelpers.Results results =
        LimelightHelpers.getLatestResults(VisionConstants.limelightName).targetingResults;

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

    if (!isValidPose(visionPose, closestTagPose, detectedTags.length)) {
      return;
    }

    Vector<N3> standardDevs =
        getLLStandardDeviations(visionPose, closestTagPose, detectedTags.length);
    poseEstimator.addVisionMeasurement(
        visionPose.toPose2d(), limelightLastDetectedTime, standardDevs);

    for (LimelightTarget_Fiducial target : detectedTags) {
      int tagID = (int) target.fiducialID;

      Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(tagID);

      if (tagPose.isPresent()) {
        poses.add(tagPose.get().toPose2d());
      }
    }
  }

  public void updateVisionPoseEstimates() {
    List<Pose2d> detectedTargets = new ArrayList<>();

    updateLimelightPoses(detectedTargets);

    field.getObject("Detected Targets").setPoses(detectedTargets);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    frontLeftModule.periodic();
    frontRightModule.periodic();
    backLeftModule.periodic();
    backRightModule.periodic();

    updateVisionPoseEstimates();

    field.setRobotPose(poseEstimator.getEstimatedPosition());
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
