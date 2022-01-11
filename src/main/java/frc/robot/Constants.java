// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class MotorConstants {
        public static final int portNumber = 2;
    }

    public static final class XboxControllerConstants {
        public static final int idNumber = 1;
    }

    public static final class SwerveConstants {
        public static final double MAX_DRIVE_SPEED_MPS = 3.0;
        public static final double MAX_TURNING_SPEED_RADIANS_PER_SECOND = 8.76;

        /** Distance between centers of front and back wheels on robot. */
        public static final double WHEEL_BASE = 0.67;

        /** Distance between right and left wheels on robot. */
        public static final double TRACK_WIDTH = 0.5;

        public static final class AbsoluteEncoderConstants {
            public static final int frontLeftEncoder = 0;
            public static final int frontRightEncoder = 1;
            public static final int rearLeftEncoder = 3;
            public static final int rearRightEncoder = 2;

            // Swerve drive encoder offsets (taken from OffSeasonIR2021)
            public static final double frontLeftOffset = 2.75 - (Math.PI / 5);
            public static final double frontRightOffset = -6.091199;
            public static final double rearLeftOffset = 2.573;
            public static final double rearRightOffset = 3.9;
        }

        public static final class MotorConstants {
            public static final int frontLeftTurningMotor = 1;
            public static final int frontLeftDriveMotor = 8;

            public static final int frontRightTurningMotor = 5;
            public static final int frontRightDriveMotor = 4;

            public static final int rearLeftTurningMotor = 3;
            public static final int rearLeftDriveMotor = 2;

            public static final int rearRightTurningMotor = 7;
            public static final int rearRightDriveMotor = 6;
        }
    }
}
