// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * 
 * <p>
 * TODO update these values when the new robot is built
 */
public final class Constants {
  public static final class SwerveConstants {
    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kRearLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 5;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftTurningEncoderPort = 0;
    public static final int kRearLeftTurningEncoderPort = 3;
    public static final int kFrontRightTurningEncoderPort = 1;
    public static final int kRearRightTurningEncoderPort = 2;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final double kFrontLeftTurningEncoderOffset = -0.81;
    public static final double kRearLeftTurningEncoderOffset = 0.93;
    public static final double kFrontRightTurningEncoderOffset = -1.19;
    public static final double kRearRightTurningEncoderOffset = -2.55;

    /** Distance between centers of right and left wheels on robot. */
    public static final double kTrackWidth = 0.5;

    /** Distance between front and back wheels on robot. */
    public static final double kWheelBase = 0.67;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 3.6;

    public static final double kMaxAngularSpeedRadiansPerSecond = 8.76;

    /** Time in seconds for the robot to stop turning from max speed. */
    public static final double kTurningStopTime = 0.2;
  }

  public static final class ModuleConstants {
    public static final double kWheelCircumferenceMeters = 0.3;

    /** Gear ratio between the motor and the wheel. */
    public static final double kDrivingGearRatio = 8.33;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 9;

    public static final double kShooterSpeedTicksPerDecisecond = 12000;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kControllerDeadband = 0.11;
  }
}
