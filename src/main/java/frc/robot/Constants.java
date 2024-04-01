// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.LinearInterpolation;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.ShooterState;
import java.awt.geom.Point2D;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    public static final int intakeNoteButton = 13;

    public static final int shootButton = 12;

    public static final int armToPickup = 5;
    public static final int armToSubwoofer = 6;
    public static final int armToSource = 7;
    public static final int armToAmp = 8;
    public static final int armAutoShoot = 14;

    public static final int armManualUp = 10;
    public static final int armManualDown = 9;

    public static final int armPreciseManualAdjustment = 2;

    public static final int startingConfiguration = 16;

    public static final int climberButton = 1;
    public static final int leftClimberButton = 3;
    public static final int rightClimberButton = 4;

    public static final int manualShootButton = 12;
    public static final int manualFeedButton = 11;
    public static final int manualOuttakeButton = 15;

    public static final int ledWarningButton = 2;
  }

  public static final class FieldConstants {
    public static final double fieldLength = 16.541;
    public static final double fieldWidth = 8.211;

    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d speakerBlueAlliance =
        new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0.0));
    public static final Pose2d speakerRedAlliance =
        new Pose2d(16.54, 5.5, Rotation2d.fromDegrees(180.0));

    public static final double speakerWidth = Units.inchesToMeters(41.375);

    public static final int blueSpeakerCenterID = 7;
    public static final int redSpeakerCenterID = 4;
  }

  public static final class StationCoordinateConstants {
    public static final class CenterChainPoses {
      public static final Pose2d bottomRightChainRedStage =
          new Pose2d(new Translation2d(12.41, 2.87), Rotation2d.fromDegrees(-59.30));
      public static final Pose2d topRightChainRedStage =
          new Pose2d(new Translation2d(12.39, 5.32), Rotation2d.fromDegrees(60));
      public static final Pose2d leftChainRedStage =
          new Pose2d(new Translation2d(10.23, 4.06), Rotation2d.fromDegrees(-180));
      public static final Pose2d rightChainBlueStage =
          new Pose2d(new Translation2d(6.32, 3.99), Rotation2d.fromDegrees(-2.01));
      public static final Pose2d topLeftChainBlueStage =
          new Pose2d(new Translation2d(4.14, 5.29), Rotation2d.fromDegrees(120));
      public static final Pose2d bottomLeftChainBlueStage =
          new Pose2d(new Translation2d(4.06, 2.89), Rotation2d.fromDegrees(-123.52));

      public static final List<Pose2d> redSidePoses =
          List.of(bottomRightChainRedStage, topRightChainRedStage, leftChainRedStage);

      public static final List<Pose2d> blueSidePoses =
          List.of(rightChainBlueStage, topLeftChainBlueStage, bottomLeftChainBlueStage);
    }

    public static final class LeftChainPoses {
      public static final Pose2d bottomRightChainRedStage =
          new Pose2d(new Translation2d(11.89, 2.59), Rotation2d.fromDegrees(-59.30));
      public static final Pose2d topRightChainRedStage =
          new Pose2d(new Translation2d(13.07, 5), Rotation2d.fromDegrees(60));
      public static final Pose2d leftChainRedStage =
          new Pose2d(new Translation2d(10.27, 4.69), Rotation2d.fromDegrees(-180));
      public static final Pose2d rightChainBlueStage =
          new Pose2d(new Translation2d(6.31, 3.57), Rotation2d.fromDegrees(-2.01));
      public static final Pose2d topLeftChainBlueStage =
          new Pose2d(new Translation2d(4.72, 5.70), Rotation2d.fromDegrees(120));
      public static final Pose2d bottomLeftChainBlueStage =
          new Pose2d(new Translation2d(3.61, 3.16), Rotation2d.fromDegrees(-123.52));

      public static final List<Pose2d> redSidePoses =
          List.of(bottomRightChainRedStage, topRightChainRedStage, leftChainRedStage);

      public static final List<Pose2d> blueSidePoses =
          List.of(rightChainBlueStage, topLeftChainBlueStage, bottomLeftChainBlueStage);
    }

    public static final class RightChainPoses {
      public static final Pose2d bottomRightChainRedStage =
          new Pose2d(new Translation2d(12.94, 3.18), Rotation2d.fromDegrees(-59.30));
      public static final Pose2d topRightChainRedStage =
          new Pose2d(new Translation2d(11.98, 5.6), Rotation2d.fromDegrees(60));
      public static final Pose2d leftChainRedStage =
          new Pose2d(new Translation2d(10.27, 3.41), Rotation2d.fromDegrees(-180));
      public static final Pose2d rightChainBlueStage =
          new Pose2d(new Translation2d(6.31, 4.63), Rotation2d.fromDegrees(-2.01));
      public static final Pose2d topLeftChainBlueStage =
          new Pose2d(new Translation2d(3.60, 5.05), Rotation2d.fromDegrees(120));
      public static final Pose2d bottomLeftChainBlueStage =
          new Pose2d(new Translation2d(4.69, 2.60), Rotation2d.fromDegrees(-123.52));

      public static final List<Pose2d> redSidePoses =
          List.of(bottomRightChainRedStage, topRightChainRedStage, leftChainRedStage);

      public static final List<Pose2d> blueSidePoses =
          List.of(rightChainBlueStage, topLeftChainBlueStage, bottomLeftChainBlueStage);
    }
  }

  public static final class VisionConstants {
    public static final String limelightName = "limelight";
    public static final String arducamName = "Arducam_OV9281";

    public static final Transform3d arducamTransform =
        new Transform3d(
            Units.inchesToMeters(-6.5),
            Units.inchesToMeters(-13.25),
            Units.inchesToMeters(8.50),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(180.0)));

    public static final Transform3d limelightTransform =
        new Transform3d(
            Units.inchesToMeters(5.53),
            0.0,
            Units.inchesToMeters(10.35),
            new Rotation3d(Units.degreesToRadians(180.0), Units.degreesToRadians(-35.0), 0.0));

    public static final class ArducamConstants {
      public static final double[] distances =
          new double[] {
            0.50, 1.00, 1.50, 2.00, 4.95, 5.5,
          };
      public static final double[] xyStandardDeviations =
          new double[] {
            0.019, // 0.50
            0.025, // 1.00
            0.050, // 1.50
            0.108, // 2.00
            0.130, // 4.95 used to be 0.037
            1.145, // 5.5 might be a stddev of 0.85-ish
          };
      public static final double[] thetaStandardDeviations =
          new double[] {
            0.115, // 0.50
            0.149, // 1.00
            0.190, // 1.50
            0.250, // 2.00
            0.353, // 4.95
            1.85, // 5.5
          };

      public static PolynomialRegression xyPolynomialRegression =
          new PolynomialRegression(distances, xyStandardDeviations, 3);

      public static PolynomialRegression thetaPolynomialRegression =
          new PolynomialRegression(distances, thetaStandardDeviations, 3);
    }

    public static final class LimelightConstants {
      public static final double[] distances = new double[] {0.50, 1.00, 1.50, 2.00, 2.50, 5.00};
      public static final double[] xyStandardDeviations =
          new double[] {
            0.014, // 0.50
            0.020, // 1.00
            0.070, // 1.50
            0.120, // 2.00
            0.160, // 2.5
            0.853, // 5.0
          };
      public static final double[] thetaStandardDeviations =
          new double[] {
            0.115, // 0.50
            0.149, // 1.00
            0.190, // 1.50
            0.250, // 2.00
            0.350, // 2.5
            0.500, // 5.0
          };

      public static PolynomialRegression xyPolynomialRegression =
          new PolynomialRegression(distances, xyStandardDeviations, 3);

      public static PolynomialRegression thetaPolynomialRegression =
          new PolynomialRegression(distances, thetaStandardDeviations, 3);
    }
  }

  public static final class ArmConstants {
    public static final int mainMotorID = 9;
    public static final int followerID = 10;

    public static final boolean mainMotorInverted = true;
    public static final boolean absoluteEncoderInverted = true;

    public static final Rotation2d absoluteOffset = Rotation2d.fromDegrees(265.89294);

    public static final double armKg = 0.14823;
    public static final double armKs = 0.21422;
    public static final double armKv = 7.6911;
    public static final double armKa = 0.01358;

    public static final double armKp = 2.55;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;

    public static final double holdKp = 1.50;
    public static final double holdKi = 0.0;
    public static final double holdKd = 0.0;

    public static final int currentLimit = 50;

    public static final double armGearRatio =
        (12.0 / 60.0) * (18.0 / 68.0) * (18.0 / 72.0) * (12.0 / 64.0);
    public static final double armPositionConversionFactor = 2 * Math.PI * armGearRatio;
    public static final double armVelocityConversionFactor = armPositionConversionFactor / 60.0;

    // Assuming 90% gearbox efficiency
    public static final double maxVelocity =
        Units.rotationsToRadians(5800 * armGearRatio) * 0.90 / 60;
    public static final double maxAcceleration = Units.degreesToRadians(480.0);

    public static final double manualSpeed = 0.60;
    public static final double preciseManualSpeed = 0.20;
    public static final double angleTolerance = Units.degreesToRadians(0.15);
    public static final double autonomousAngleTolerance = Units.degreesToRadians(0.35);
    public static final double autoShootAngleTolerance = Units.degreesToRadians(0.65);
    public static final double replanningError = Units.degreesToRadians(55.0);
    public static final double debounceTime = 0.50;
    public static final double movementDebounceTime = 0.353;
    public static final double autoMovementDebounceTime = 0.40;
    public static final double holdDebounceTime = 1.5;

    public static final TrapezoidProfile.Constraints profileConstraints =
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    public static final Rotation2d pickupAngle = Rotation2d.fromDegrees(-2.4);
    public static final Rotation2d ampAngle = Rotation2d.fromDegrees(95.0);
    public static final Rotation2d subwooferAngle = Rotation2d.fromDegrees(7.5);
    public static final Rotation2d podiumAngle = Rotation2d.fromDegrees(22.03);
    public static final Rotation2d startingConfigAngle = Rotation2d.fromDegrees(72.02189922348654);
    public static final Rotation2d sourceAngle = Rotation2d.fromDegrees(62.85939169721119);

    public static final Rotation2d autoSubwooferAngle = Rotation2d.fromDegrees(7.5 - 2.0 - 1.0);
    public static final Rotation2d autoAmpPodiumAngle = Rotation2d.fromDegrees(34.785046585654335);
    public static final Rotation2d autoFarAmpPodiumAngle =
        Rotation2d.fromDegrees(35.3 - 1.0 - 1.0 - 0.25 - 0.5); // 35.3 - 1.0 - 1.0 - 0.25 + 0.5 32.3
    public static final Rotation2d autoRCAmpPodiumAngle = Rotation2d.fromDegrees(38.5);
    public static final Rotation2d autoAmp4PieceFinalAngle =
        Rotation2d.fromDegrees(26.007488521000912 - 0.5); // 26.007488521000912 + 0.5 26.7
    public static final Rotation2d autoSourcePodiumAngle = Rotation2d.fromDegrees(36.3);
    public static final Rotation2d autoCloseShootAngle = Rotation2d.fromDegrees(22.5);
    public static final Rotation2d autoWingShotAngle = Rotation2d.fromDegrees(27.00 - 1.0);
    public static final Rotation2d autoCenterWingShotAngle = Rotation2d.fromDegrees(23.50 - 1.0);
    public static final Rotation2d autoAmpWingAngle = Rotation2d.fromDegrees(30.0);
    public static final Rotation2d behindWing1Angle = Rotation2d.fromDegrees(28.117);
    public static final Rotation2d behindWing2Angle = Rotation2d.fromDegrees(29.49);
    public static final Rotation2d fivePieceAutoFinale = Rotation2d.fromDegrees(20.38);

    public static final double ampSpeedAngle = Units.degreesToRadians(85.0);
    public static final double subwooferSpeedAngle = Units.degreesToRadians(12.0);

    public static final double reverseMovementLimitAngle = Units.degreesToRadians(-5.2);
    public static final double forwardMovementLimitAngle = Units.degreesToRadians(103.53);

    public static final double armPivotZ = Units.inchesToMeters(10.3);
    public static final double armPivotX = Units.inchesToMeters(-7.5);
    public static final double armLength = Units.inchesToMeters(21.75);
    public static final double armPivotToShooter = Units.inchesToMeters(16.0);
    public static final double armMass = Units.lbsToKilograms(30.0);
  }

  public static class IntakeConstants {
    public static final int intakeMotorID = 13;

    public static final double intakeSpeed = 0.90;
    public static final int intakeCurrentLimit = 40; // Amps
    public static final int shutoffCurrentLimit = 55;

    public static final int beamBreakID = 9;
  }

  public static class ShooterConstants {
    public static final int bottomShooterID = 12;
    public static final int topShooterID = 11;

    public static final double shooterVelocity = 4000.0;
    public static final double subwooferVelocity = 2500.0;
    public static final double ampVelocity = 750.0;

    public static final ShooterState defaultState = new ShooterState(4500.0, 2800.0);
    public static final ShooterState defaultSameSpeed = new ShooterState(4000.0);
    public static final ShooterState subwooferState = new ShooterState(2500.0);
    public static final ShooterState ampState = new ShooterState(750.0);
    public static final ShooterState idleState = new ShooterState(2000.0);

    public static final double topShooterKs = 0.04452;
    public static final double topShooterKv = 0.0021696;
    public static final double topShooterKa = 0.00073383;

    public static final double bottomShooterKs = 0.12555;
    public static final double bottomShooterKv = 0.0021493;
    public static final double bottomShooterKa = 0.00076645;

    public static final double shooterKp = 0.0012;

    public static final double velocityTolerance = 50.0; // 100.0

    public static final double voltageCompensation = 12.0;

    public static final int shooterCurrentLimit = 40;

    public static final double gearing = 1 / 0.75;
    public static final double circumference = Math.PI * Units.inchesToMeters(3.8);
  }

  public static final class AutoShootConstants {
    // (distance, angle)
    // This isn't being used, but the data could still be important at some point
    public static final Point2D[] autoShootArmAngles =
        new Point2D.Double[] {
          new Point2D.Double(1.12, Units.degreesToRadians(8.0 - 1.5)),
          new Point2D.Double(1.25, Units.degreesToRadians(8.0 - 1.5)),
          new Point2D.Double(1.60, Units.degreesToRadians(8.0 - 1.5)),
          new Point2D.Double(2.09, Units.degreesToRadians(20.732014330623144 - 2.0 - 1.5)),
          // new Point2D.Double(2.29, Units.degreesToRadians(26.03)),
          new Point2D.Double(2.50, Units.degreesToRadians(24.935558632659113 - 2.0 - 1.5)),
          new Point2D.Double(3.0, Units.degreesToRadians(28.465810207071904 - 2.0 - 1.5)),
          new Point2D.Double(3.53, Units.degreesToRadians(31.604855866583044 - 2.0 - 1.5)),
          // new Point2D.Double(2.91, Units.degreesToRadians(26.9)),
          // new Point2D.Double(3.26, Units.degreesToRadians(31.0)),
          // new Point2D.Double(Units.inchesToMeters(38.0), Units.degreesToRadians(8.5)),
          // new Point2D.Double(1.83, Units.degreesToRadians(13.0)),
          // new Point2D.Double(2.03, Units.degreesToRadians(16.14)),
          // new Point2D.Double(2.42, Units.degreesToRadians(22.8)),
          // new Point2D.Double(2.45, Units.degreesToRadians(23.5)),
          // new Point2D.Double(2.65, Units.degreesToRadians(23.1)),
          // new Point2D.Double(2.94, Units.degreesToRadians(27.3)),
          // new Point2D.Double(3.02, Units.degreesToRadians(27.4)),
          // new Point2D.Double(3.06, Units.degreesToRadians(28.1)),
          // new Point2D.Double(3.09, Units.degreesToRadians(29.5)),
          // new Point2D.Double(3.7, Units.degreesToRadians(31.2)),
        };

    public static final InterpolatingTreeMap<Double, Rotation2d> autoShootAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    static {
      autoShootAngleMap.put(1.12, Rotation2d.fromDegrees(8.0 - 1.5));
      autoShootAngleMap.put(1.25, Rotation2d.fromDegrees(8.0 - 1.5));
      autoShootAngleMap.put(1.60, Rotation2d.fromDegrees(8.0 - 1.5));
      autoShootAngleMap.put(2.09, Rotation2d.fromDegrees(20.732014330623144 - 2.0 - 1.5));
      autoShootAngleMap.put(2.50, Rotation2d.fromDegrees(24.935558632659113 - 2.0 - 1.5));
      autoShootAngleMap.put(3.0, Rotation2d.fromDegrees(28.465810207071904 - 2.0 - 1.5));
      autoShootAngleMap.put(
          3.53, Rotation2d.fromDegrees(27.374591864513985)); // 31.604855866583044 - 2.0 - 1.5
      autoShootAngleMap.put(3.98, Rotation2d.fromDegrees(29.137217802484155));
      autoShootAngleMap.put(4.1522590735236655, Rotation2d.fromDegrees(29.15150655822783));
      autoShootAngleMap.put(4.98, Rotation2d.fromDegrees(30.921454458988247));
    }

    public static final InterpolatingTreeMap<Double, ShooterState> autoShootSpeedMap =
        new InterpolatingTreeMap<Double, ShooterState>(
            InverseInterpolator.forDouble(), ShooterState::interpolate);

    static {
      autoShootSpeedMap.put(1.10, new ShooterState(2800.0, 2800.0));
      autoShootSpeedMap.put(1.5, new ShooterState(2800.0, 2800.0));
      autoShootSpeedMap.put(1.5 + 0.00000001, new ShooterState(4500.0, 3200.0));
      autoShootSpeedMap.put(2.91 - 0.00000001, new ShooterState(4500.0, 3200.0));
      autoShootSpeedMap.put(2.91, new ShooterState(4500.0, 3200.0));
      autoShootSpeedMap.put(3.98 - 0.00000001, new ShooterState(4500.0, 3200.0));
      autoShootSpeedMap.put(3.98, new ShooterState(4500.0, 3000.0));
    }

    // (distance, time)
    public static final Point2D[] autoShootTimes =
        new Point2D.Double[] {
          new Point2D.Double(1.7, 0.19),
          new Point2D.Double(2.70, 0.23),
          new Point2D.Double(3.35, 0.29),
        };

    public static final LinearInterpolation autoShootTimeInterpolation =
        new LinearInterpolation(autoShootTimes);
  }

  public static final class ClimberConstants {
    public static final int leftMotorID = 14;
    public static final int rightMotorID = 15;

    public static final int climberCurrentLimit = 60;

    public static final double climberMotorSpeed = 0.70;
  }

  public static final class LEDConstants {
    public static final int ledPort = 0;
    public static final int bufferLength = 16;

    public static final Color transparent = new Color(0, 0, 0);
    public static final Color rslColor = new Color(255, 25, 0);
  }

  public static final class SwerveConstants {
    public static final double TRACK_WIDTH = Units.inchesToMeters(24.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.0);

    public static final Translation2d frontLeft =
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d frontRight =
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d backLeft =
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d backRight =
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final Translation2d[] wheelLocations = {
      frontLeft, frontRight, backLeft, backRight
    };

    public static final double driveBaseRadius = frontLeft.getNorm();

    public static final double maxTranslationalSpeed = Units.feetToMeters(14.5);
    public static final double maxAngularSpeed = Units.degreesToRadians(180);
    public static final double slowMotionMaxTranslationalSpeed = Units.feetToMeters(3.53);
    public static final double turboMaxTranslationalSpeed = Units.feetToMeters(14.5);

    public static final double maxTranslationalAcceleration = Units.feetToMeters(18.0);
    public static final double maxAngularAcceleration = Units.feetToMeters(270.0);

    public static final double wheelRadius = Units.inchesToMeters(3.86) / 2;
    public static final double wheelCircumference = wheelRadius * 2 * Math.PI;
    public static final double driveGearRatio = 1 / 6.12;
    public static final double driveCouplingRatio = 50.0 / 14.0;
    public static final double turnGearRatio = 1 / (150.0 / 7.0);

    public static final double drivePositionConversion = wheelCircumference * driveGearRatio;
    public static final double driveVelocityConversion = drivePositionConversion / 60;

    public static final double turnPositionConversion = 2 * Math.PI * turnGearRatio;

    public static final boolean driveMotorInverted = false;
    public static final boolean turnMotorInverted = true;
    public static final boolean canCoderInverted = false;

    public static final boolean useCosineCompensation = true;

    public static final double skewOpenLoopFudgeFactor = 0.100;
    public static final double skewClosedLoopFudgeFactor = 0.010;

    public static final double driveP = 0.08;

    public static final double driveKs = 0.22542;
    public static final double driveKv = 2.4829;
    public static final double driveKa = 0.0 * 0.22997;

    public static final double turnP = 0.85;
    public static final double turnD = 0;

    public static final double headingP = 0.55;
    public static final double headingD = 0;

    public static final double turnToAngleMaxVelocity = Units.degreesToRadians(270.0);

    public static final double openLoopRamp = 0.15;
    public static final double closedLoopRamp = 0.03;

    public static final double voltageCompensation = 12.0;

    public static final int driveCurrentLimit = 45;
    public static final int turnCurrentLimit = 25;

    public static final double maxDriveTemperature = 50.0;
    public static final double maxTurnTemperature = 50.0;

    public static final double maxModuleSpeed = Units.feetToMeters(14.5);

    public static final int odometryUpdateFrequency = 100; // 100 Hz

    public static final boolean zeroWithIntakeForward = true;
  }

  public static final class AutoConstants {
    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);

    public static final double pathfindingMaxVelocity = Units.feetToMeters(13.0);
    public static final double pathfindingMaxAcceleration = Units.feetToMeters(20.0);
    public static final double pathfindingMaxAngularVelocity = Units.degreesToRadians(180.0);
    public static final double pathfindingMaxAngularAcceleration = Units.degreesToRadians(180.0);
  }

  public static class FrontLeftModule {
    public static final int driveID = 3;
    public static final int turnID = 7;
    public static final int encoderID = 11;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(86.66015625);
  }

  public static class FrontRightModule {
    public static final int driveID = 1;
    public static final int turnID = 5;
    public static final int encoderID = 9;
    public static final Rotation2d angleOffset =
        Rotation2d.fromDegrees(-127.88085937499999); // -133.06640625
  }

  public static class BackLeftModule {
    public static final int driveID = 2;
    public static final int turnID = 6;
    public static final int encoderID = 10;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-57.216796875);
  }

  public static final class BackRightModule {
    public static final int driveID = 4;
    public static final int turnID = 8;
    public static final int encoderID = 12;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-78.134765625);
  }
}
