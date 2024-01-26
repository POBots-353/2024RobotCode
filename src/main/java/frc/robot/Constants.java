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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.LinearInterpolation;
import frc.robot.util.PolynomialRegression;
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
    public static final int armShootSubwoofer = 7;
    public static final int armShootPodium = 8;
    public static final int armToAmp = 6;
    public static final int armAutoShoot = 14;

    public static final int armManualUp = 10;
    public static final int armManualDown = 9;

    public static final int climberButton = 1;

    public static final int manualShootButton = 12;
    public static final int manualFeedButton = 11;

    public static final int ledWarningButton = 2;
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d speakerBlueAlliance =
        new Pose2d(0.25, 5.5, Rotation2d.fromDegrees(180.0));
    public static final Pose2d speakerRedAlliance =
        new Pose2d(16.3, 5.5, Rotation2d.fromDegrees(0.0));

    public static Pose2d driverStationBlueAlliance = new Pose2d();
    public static Pose2d driverStationRedAlliance = new Pose2d();
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

      public static final List<Pose2d> poses =
          List.of(
              bottomRightChainRedStage,
              topRightChainRedStage,
              leftChainRedStage,
              rightChainBlueStage,
              topLeftChainBlueStage,
              bottomLeftChainBlueStage);
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

      public static final List<Pose2d> poses =
          List.of(
              bottomRightChainRedStage,
              topRightChainRedStage,
              leftChainRedStage,
              rightChainBlueStage,
              topLeftChainBlueStage,
              bottomLeftChainBlueStage);
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

      public static final List<Pose2d> poses =
          List.of(
              bottomRightChainRedStage,
              topRightChainRedStage,
              leftChainRedStage,
              rightChainBlueStage,
              topLeftChainBlueStage,
              bottomLeftChainBlueStage);
    }
  }

  public static final class VisionConstants {
    public static final String limelightName = "limelight";
    public static final String arducamName = "Arducam_OV9281";

    public static final Transform3d arducamPose =
        new Transform3d(
            Units.inchesToMeters(12.5),
            0.0,
            Units.inchesToMeters(3.53),
            new Rotation3d(0.0, 0.0, 0.0));

    public static final class ArducamConstants {
      public static final double[] distances =
          new double[] {
            0.50, 1.00, 1.50, 2.00, 4.95, 5.5,
          };
      public static final double[] xyStandardDeviations =
          new double[] {
            0.014, // 0.50
            0.020, // 1.00
            0.150, // 1.50
            0.200, // 2.00
            0.037, // 4.95
            1.15, // 5.5 might be a stddev of 0.85-ish
          };
      public static final double[] thetaStandardDeviations =
          new double[] {
            0.115, // 0.50
            0.149, // 1.00
            0.190, // 1.50
            0.250, // 2.00
            1.85, // 5.5
          };

      public static PolynomialRegression xyPolynomialRegression =
          new PolynomialRegression(distances, xyStandardDeviations, 3);

      public static PolynomialRegression thetaPolynomialRegression =
          new PolynomialRegression(distances, thetaStandardDeviations, 3);
    }

    public static final class LimelightConstants {
      public static final double[] distances =
          new double[] {
            0.50, 1.00, 1.50, 2.00,
          };
      public static final double[] xyStandardDeviations =
          new double[] {
            0.014, // 0.50
            0.020, // 1.00
            0.150, // 1.50
            0.200, // 2.00
          };
      public static final double[] thetaStandardDeviations =
          new double[] {
            0.115, // 0.50
            0.149, // 1.00
            0.190, // 1.50
            0.250 // 2.00
          };

      public static PolynomialRegression xyPolynomialRegression =
          new PolynomialRegression(distances, xyStandardDeviations, 3);

      public static PolynomialRegression thetaPolynomialRegression =
          new PolynomialRegression(distances, thetaStandardDeviations, 3);
    }
  }

  public static final class ArmConstants {
    public static final int mainMotorID = 10;
    public static final int followerID = 11;

    public static final boolean mainMotorInverted = true;
    public static final boolean absoluteEncoderInverted = true;

    public static final Rotation2d absoluteOffset = Rotation2d.fromDegrees(0.0);

    public static final double armKg = 0.353;
    public static final double armKs = 0.353;
    public static final double armKv = 0.353;
    public static final double armKa = 0.353;

    public static final double armKp = 0.1;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;

    public static final double armGearRatio = 1.0 / 400.0;
    public static final double armPositionConversionFactor = 2 * Math.PI * armGearRatio;
    public static final double armVelocityConversionFactor = armPositionConversionFactor / 60.0;

    public static final double maxVelocity = Units.degreesToRadians(180.0);
    public static final double maxAcceleration = Units.degreesToRadians(180.0);

    public static final double manualSpeed = 0.60;

    public static final TrapezoidProfile.Constraints profileConstraints =
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    public static final Rotation2d pickupAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d autoAmpPodiumAngle = Rotation2d.fromDegrees(60.0);
    public static final Rotation2d autoSourcePodiumAngle = Rotation2d.fromDegrees(60.0);
    public static final Rotation2d ampAngle = Rotation2d.fromDegrees(45.0);
    public static final Rotation2d subwooferAngle = Rotation2d.fromDegrees(15.0);
    public static final Rotation2d podiumAngle = Rotation2d.fromDegrees(60.0);

    // (distance, angle)
    // Stubbed with fake values for now
    public static final Point2D[] autoShootArmAngles =
        new Point2D.Double[] {
          new Point2D.Double(0.25, Units.degreesToRadians(3.53)),
          new Point2D.Double(0.5, Units.degreesToRadians(3.53)),
        };

    public static final LinearInterpolation autoShootInterpolation =
        new LinearInterpolation(autoShootArmAngles);

    // (distance, time)
    public static final Point2D[] autoShootTimes =
        new Point2D.Double[] {
          new Point2D.Double(0.25, 0.5), new Point2D.Double(0.25, 0.5),
        };

    public static final LinearInterpolation autoShootTimeInterpolation =
        new LinearInterpolation(autoShootTimes);
  }

  public static class IntakeConstants {
    public static final int intakeMotorOneID = 5;
    public static final double intakeMotorSpeed = 0.80;
  }

  public static class ShooterConstants {
    public static final double shooterVelocity = 0;
    public static final double shooterKs = 0.353;
    public static final double shooterKv = 0.353;
    public static final double shooterKa = 0.353;
    public static final double shooterP = 0.353;
    public static final int shooterMainId = 0;
    public static final int shooterFollowerId = 0;
  }

  public static final class ClimberConstants {
    public static final int mainMotorID = 15;
    public static final int followerMotorID = 16;

    public static final double climberMotorSpeed = 0.75;
  }

  public static final class LEDConstants {
    public static final int ledPort = 0;
    public static final int bufferLength = 100;

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

    public static final double maxTranslationalSpeed = Units.feetToMeters(12.5);
    public static final double maxAngularSpeed = Units.degreesToRadians(180);
    public static final double slowMotionMaxTranslationalSpeed = Units.feetToMeters(5.0);
    public static final double turboMaxTranslationalSpeed = Units.feetToMeters(14.5);

    public static final double maxTranslationalAcceleration = Units.feetToMeters(25.0);
    public static final double maxAngularAcceleration = Units.feetToMeters(270.0);

    public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    public static final double driveGearRatio = 1 / 6.12;
    public static final double turnGearRatio = 1 / (150.0 / 7.0);

    public static final double drivePositionConversion = wheelCircumference * driveGearRatio;
    public static final double driveVelocityConversion = drivePositionConversion / 60;

    public static final double turnPositionConversion = 2 * Math.PI * turnGearRatio;

    public static final boolean driveMotorInverted = false;
    public static final boolean turnMotorInverted = true;
    public static final boolean canCoderInverted = false;

    public static final double skewOpenLoopFudgeFactor = 0.100;
    public static final double skewClosedLoopFudgeFactor = 0.060;

    public static final double driveP = 0.15;

    public static final double driveKs = 0.16022;
    public static final double driveKv = 2.3501;
    public static final double driveKa = 0.37865;

    public static final double turnP = 0.85;
    public static final double turnD = 0;

    public static final double headingP = 0.55;
    public static final double headingD = 0;

    public static final double turnToAngleMaxVelocity = Units.degreesToRadians(270.0);

    public static final double openLoopRamp = 0.15;
    public static final double closedLoopRamp = 0.0;

    public static final double voltageCompensation = 12.0;

    public static final int driveCurrentLimit = 60;
    public static final int turnCurrentLimit = 20;

    public static final double maxDriveTemperature = 50.0;
    public static final double maxTurnTemperature = 50.0;

    public static final double maxModuleSpeed = Units.feetToMeters(14.5);

    public static final int odometryUpdateFrequency = 100; // 100 Hz
  }

  public static final class AutoConstants {
    public static final PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(4.5, 0.0, 0.0);
  }

  public static final class FrontLeftModule {
    public static final int driveID = 4;
    public static final int turnID = 8;
    public static final int encoderID = 12;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(102.568359375);
  }

  public static class FrontRightModule {
    public static final int driveID = 2;
    public static final int turnID = 6;
    public static final int encoderID = 10;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(128.759765625);
  }

  public static class BackLeftModule {
    public static final int driveID = 1;
    public static final int turnID = 5;
    public static final int encoderID = 9;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(52.734375);
  }

  public static class BackRightModule {
    public static final int driveID = 3;
    public static final int turnID = 7;
    public static final int encoderID = 11;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-93.427734375);
  }
}
