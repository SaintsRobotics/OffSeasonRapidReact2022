package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.subsystems.SwerveModule;

/** File for storing the hardware of the robot. */
public class HardwareMap {

        /** File for storing the hardware of the drivetrain. */
        public class SwerveDrivetrainHardware {
                public SwerveModule frontLeft = new SwerveModule(swerveModuleHardware,
                                swerveModuleHardware.frontLeftDriveMotor, swerveModuleHardware.frontLeftTurningMotor,
                                Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2,
                                swerveModuleHardware.frontLeftAbsoluteEncoder);
                public SwerveModule frontRight = new SwerveModule(swerveModuleHardware,
                                swerveModuleHardware.frontRightDriveMotor, swerveModuleHardware.frontRightTurningMotor,
                                Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2,
                                swerveModuleHardware.frontRightAbsoluteEncoder);
                public SwerveModule rearLeft = new SwerveModule(swerveModuleHardware,
                                swerveModuleHardware.rearLeftDriveMotor, swerveModuleHardware.rearLeftTurningMotor,
                                -Constants.SwerveConstants.WHEEL_BASE / 2, Constants.SwerveConstants.TRACK_WIDTH / 2,
                                swerveModuleHardware.rearLeftAbsoluteEncoder);
                public SwerveModule rearRight = new SwerveModule(swerveModuleHardware,
                                swerveModuleHardware.rearRightDriveMotor, swerveModuleHardware.rearRightTurningMotor,
                                -Constants.SwerveConstants.WHEEL_BASE / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2,
                                swerveModuleHardware.rearRightAbsoluteEncoder);
        }

        /** File for storing the hardware of the swerve module. */
        public class SwerveModuleHardware {
                public CANSparkMax frontLeftTurningMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.frontLeftTurningMotor, MotorType.kBrushless);
                public CANSparkMax frontLeftDriveMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.frontLeftDriveMotor, MotorType.kBrushless);
                public AbsoluteEncoder frontLeftAbsoluteEncoder = new AbsoluteEncoder(
                                Constants.SwerveConstants.AbsoluteEncoderConstants.frontLeftEncoder,
                                Constants.SwerveConstants.AbsoluteEncoderConstants.frontLeftOffset);

                public CANSparkMax frontRightTurningMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.frontRightTurningMotor, MotorType.kBrushless);
                public CANSparkMax frontRightDriveMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.frontRightDriveMotor, MotorType.kBrushless);
                public AbsoluteEncoder frontRightAbsoluteEncoder = new AbsoluteEncoder(
                                Constants.SwerveConstants.AbsoluteEncoderConstants.frontRightEncoder,
                                Constants.SwerveConstants.AbsoluteEncoderConstants.frontRightOffset);

                public CANSparkMax rearLeftTurningMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.rearLeftTurningMotor, MotorType.kBrushless);
                public CANSparkMax rearLeftDriveMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.rearLeftDriveMotor, MotorType.kBrushless);
                public AbsoluteEncoder rearLeftAbsoluteEncoder = new AbsoluteEncoder(
                                Constants.SwerveConstants.AbsoluteEncoderConstants.rearLeftEncoder,
                                Constants.SwerveConstants.AbsoluteEncoderConstants.rearLeftOffset);

                public CANSparkMax rearRightTurningMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.rearRightTurningMotor, MotorType.kBrushless);
                public CANSparkMax rearRightDriveMotor = new CANSparkMax(
                                Constants.SwerveConstants.MotorConstants.rearRightDriveMotor, MotorType.kBrushless);
                public AbsoluteEncoder rearRightAbsoluteEncoder = new AbsoluteEncoder(
                                Constants.SwerveConstants.AbsoluteEncoderConstants.rearRightEncoder,
                                Constants.SwerveConstants.AbsoluteEncoderConstants.rearRightOffset);
        }

        public SwerveModuleHardware swerveModuleHardware;
        public SwerveDrivetrainHardware swerveDrivetrainHardware;

        /** Creates a new {@link HardwareMap} */
        public HardwareMap() {
                swerveModuleHardware = new SwerveModuleHardware();
                swerveDrivetrainHardware = new SwerveDrivetrainHardware();
        }
}