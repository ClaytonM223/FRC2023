// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        public static final double kTurningMotorGearRatio = 1 / 7;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2RadPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
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

      public static final boolean kFrontRightTurningEncoderReversed = true;
      public static final boolean kBackRightTurningEncoderReversed = true;
      public static final boolean kFrontLeftTurningEncoderReversed = true;
      public static final boolean kBackLeftTurningEncoderReversed = true;

      public static final boolean kFrontRightDriveEncoderReversed = false;
      public static final boolean kBackRightDriveEncoderReversed = false;
      public static final boolean kFrontLeftDriveEncoderReversed = true;
      public static final boolean kBackLeftDriveEncoderReversed = true;

      public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
      public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
      public static final int kBackRightDriveAbsoluteEncoderPort = 12;
      public static final int kBackLeftDriveAbsoluteEncoderPort = 11;



    }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
