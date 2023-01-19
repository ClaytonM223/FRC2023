// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public final class USB{
      public static final int DRIVER_CONTROLLER = 0;
      public static final int OPERATOR_CONTROLLER = 1;
    }

    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = .10;
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2RadPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
      }

    public final class sensors{
      public static final int GYRO_ID = 13;
    }

    public static final class DriveConstants {

      public static final double kTrackWidth = Units.inchesToMeters(25.25);
      //distance between right and left wheel 
      public static final double kWheelBase = Units.inchesToMeters(25.25);
      //distance front to back
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final int kFrontLeftDriveMotorPort = 5;
      public static final int kFrontLeftTurningMotorPort = 4;

      public static final int kBackLeftDriveMotorPort = 3;
      public static final int kBackLeftTurningMotorPort = 2;
       
      public static final int kFrontRightDriveMotorPort = 6;
      public static final int kFrontRightTurningMotorPort = 7;

      public static final int kBackRightDriveMotorPort = 1;
      public static final int kBackRightTurningMotorPort = 8;

      public static final boolean kFrontRightTurningEncoderReversed = false;
      public static final boolean kBackRightTurningEncoderReversed = false;
      public static final boolean kFrontLeftTurningEncoderReversed = false;
      public static final boolean kBackLeftTurningEncoderReversed = false;

      public static final boolean kFrontRightDriveEncoderReversed = false;
      public static final boolean kBackRightDriveEncoderReversed = false;
      public static final boolean kFrontLeftDriveEncoderReversed = true;
      public static final boolean kBackLeftDriveEncoderReversed = true;

      public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
      public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
      public static final int kBackRightDriveAbsoluteEncoderPort = 12;
      public static final int kBackLeftDriveAbsoluteEncoderPort = 11;

      public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
 
      public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
      public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;

      public static final double kFrontRightDriveEncoderOffsetRad = 0.912;
      public static final double kFrontLeftDriveEncoderOffsetRad = 4.316;
      public static final double kBackRightDriveEncoderOffsetRad = 2.828;
      public static final double kBackLeftDriveEncoderOffsetRad = 2.230;

      public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 8;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; 
 }

    public static final class AutoConstants{
      public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
      public static final double kPXController = 1.5;
      public static final double kPYController = 1.5;
      public static final double kPTController = 3;

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
              new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants{
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.05;
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
