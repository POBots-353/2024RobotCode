// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
  }

  public static final class SwerveConstants {
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.5);

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

    public static final double maxTranslationalSpeed = Units.feetToMeters(12.5);
    public static final double maxAngularSpeed = Units.degreesToRadians(180);

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

    public static final double chassisSkewFudgeFactor = 0.120;

    public static final double driveP = 0.15;

    public static final double driveKs = 0.11196;
    public static final double driveKv = 2.3939;
    public static final double driveKa = 0.15385;

    public static final double turnP = 0.65;
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

    public static final int odometryUpdateFrequency = 100; // 100 Hz (every 10 ms)
  }

  public static final class FrontLeftModule {
    public static final int driveID = 4;
    public static final int turnID = 8;
    public static final int encoderID = 12;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(103.271484375);
  }

  public static class FrontRightModule {
    public static final int driveID = 2;
    public static final int turnID = 6;
    public static final int encoderID = 10;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.693359375);
  }

  public static class BackLeftModule {
    public static final int driveID = 1;
    public static final int turnID = 5;
    public static final int encoderID = 9;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(53.173828125);
  }

  public static class BackRightModule {
    public static final int driveID = 3;
    public static final int turnID = 7;
    public static final int encoderID = 11;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(266.923828125);
  }
}
