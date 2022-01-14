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

        public static final int kFrontLeftEncoderPort = 0;
        public static final int kRearLeftEncoderPort = 3;
        public static final int kFrontRightEncoderPort = 1;
        public static final int kRearRightEncoderPort = 2;

        /** Distance between right and left wheels on robot. */
        public static final double kTrackWidth = 0.5;

        /** Distance between centers of front and back wheels on robot. */
        public static final double kWheelBase = 0.67;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kMaxSpeedMetersPerSecond = 3.0;

        public static final class AbsoluteEncoderConstants {
            // Swerve drive encoder offsets (taken from OffSeasonIR2021)
            public static final double frontLeftOffset = 2.75 - (Math.PI / 5);
            public static final double frontRightOffset = -6.091199;
            public static final double rearLeftOffset = 2.573;
            public static final double rearRightOffset = 3.9;
        }

        public static final double kMaxAngularSpeedRadiansPerSecond = 8.76;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }
}
