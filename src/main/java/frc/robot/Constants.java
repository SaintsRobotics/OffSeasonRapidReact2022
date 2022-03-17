// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.MUX.Port;

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
		public static final int kFrontLeftDriveMotorPort = 9;
		public static final int kRearLeftDriveMotorPort = 12;
		public static final int kFrontRightDriveMotorPort = 5;
		public static final int kRearRightDriveMotorPort = 2;

		public static final int kFrontLeftTurningMotorPort = 7;
		public static final int kRearLeftTurningMotorPort = 11;
		public static final int kFrontRightTurningMotorPort = 4;
		public static final int kRearRightTurningMotorPort = 16;

		public static final int kFrontLeftTurningEncoderPort = 19;
		public static final int kRearLeftTurningEncoderPort = 20;
		public static final int kFrontRightTurningEncoderPort = 18;
		public static final int kRearRightTurningEncoderPort = 17;

		public static final double kFrontLeftTurningEncoderOffset = 356;
		public static final double kRearLeftTurningEncoderOffset = 122;
		public static final double kFrontRightTurningEncoderOffset = 256;
		public static final double kRearRightTurningEncoderOffset = 328;

		public static final boolean kFrontLeftDriveMotorReversed = false;
		public static final boolean kRearLeftDriveMotorReversed = false;
		public static final boolean kFrontRightDriveMotorReversed = true;
		public static final boolean kRearRightDriveMotorReversed = true;
		/** Distance between centers of right and left wheels on robot. */
		public static final double kTrackWidth = 0.57;

		/** Distance between front and back wheels on robot. */
		public static final double kWheelBase = 0.6;

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final double kMaxSpeedMetersPerSecond = 3.66;

		// TODO update value with new robot
		public static final double kMaxAngularSpeedRadiansPerSecond = 8.76;

		/**
		 * Time in seconds for the robot to stop turning from max speed.
		 * TODO update value with new robot.
		 */
		public static final double kTurningStopTime = 0.2;
	}

	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = 0.1;

		/** Gear ratio between the motor and the wheel. */
		public static final double kDrivingGearRatio = 8.14;
	}

	public static final class ShooterConstants {
		public static final int kBottomFlywheelPort = 9;
		public static final int kTopFlywheelPort = 23;

		public static final int kArmPort = 13;
		public static final int kIntakeWheelsPort = 8;
		public static final int kLeftFeederPort = 10;
		public static final int kRightFeederPort = 6;
		public static final int kTopFeederPort = 14;

		public static final Port kQueueColorSensorPort = Port.kTwo;
		public static final Port kShooterColorSensorPort = Port.kThree;

		public static final double kBottomShooterSpeedRPM = 2200;
		public static final double kTopShooterSpeedRPM = 5000; // TODO: THIS RPM NEEDS UPDATING

		public static final int kLowerArmAngle = 50;
		public static final int kUpperArmAngle = -50;
		public static final double kIntakeSpeed = 0.7;
		public static final double kTopFeederSpeedSlow = 0.1;
		public static final double kTopFeederSpeedFast = 1;
		public static final double kSideFeederSpeed = 0.6;
		public static final boolean kIntakeReversed = true;
		public static final boolean kLeftFeederReversed = true;
		public static final boolean kRightFeederReversed = false;

		public static final int kRedThreshold = 300;
		public static final int kBlueThreshold = 300;
		public static final double kBottomShooterP = 0.00045;
		public static final double kTopShooterP = 0.000005;
	}

	/** Constants for the climber. */
	public static final class ClimberConstants {
		public static final int kLeftArmPort = 15;
		public static final int kRightArmPort = 3;

		public static final int kLeftServoPort = 1;
		public static final int kRightServoPort = 0;

		public static final boolean kLeftArmReversed = true;
		public static final boolean kRightArmReversed = false;

		public static final double kLeftServoLockedPosition = 1;
		public static final double kLeftServoUnlockedPosition = 0.5;

		public static final double kRightServoLockedPosition = 0.5;
		public static final double kRightServoUnlockedPosition = 1;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kControllerDeadband = 0.11;

		public static final boolean kTelemetry = false;
	}
}
