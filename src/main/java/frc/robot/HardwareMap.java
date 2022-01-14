package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

/** File for storing the hardware of the robot. */
public class HardwareMap {
  /** File for storing the hardware of the drivetrain. */
  public class SwerveDrivetrainHardware {
    public SwerveModule frontLeft = new SwerveModule(swerveModuleHardware,
        swerveModuleHardware.frontLeftDriveMotor, swerveModuleHardware.frontLeftTurningMotor,
        swerveModuleHardware.frontLeftAbsoluteEncoder);
    public SwerveModule frontRight = new SwerveModule(swerveModuleHardware,
        swerveModuleHardware.frontRightDriveMotor, swerveModuleHardware.frontRightTurningMotor,
        swerveModuleHardware.frontRightAbsoluteEncoder);
    public SwerveModule rearLeft = new SwerveModule(swerveModuleHardware,
        swerveModuleHardware.rearLeftDriveMotor, swerveModuleHardware.rearLeftTurningMotor,
        swerveModuleHardware.rearLeftAbsoluteEncoder);
    public SwerveModule rearRight = new SwerveModule(swerveModuleHardware,
        swerveModuleHardware.rearRightDriveMotor, swerveModuleHardware.rearRightTurningMotor,
        swerveModuleHardware.rearRightAbsoluteEncoder);
  }

  /** File for storing the hardware of the swerve module. */
  public class SwerveModuleHardware {
    public CANSparkMax frontLeftTurningMotor = new CANSparkMax(
        SwerveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax frontLeftDriveMotor = new CANSparkMax(
        SwerveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder frontLeftAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kFrontLeftEncoderPort,
        SwerveConstants.AbsoluteEncoderConstants.frontLeftOffset);

    public CANSparkMax frontRightTurningMotor = new CANSparkMax(
        SwerveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax frontRightDriveMotor = new CANSparkMax(
        SwerveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder frontRightAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kFrontRightEncoderPort,
        SwerveConstants.AbsoluteEncoderConstants.frontRightOffset);

    public CANSparkMax rearLeftTurningMotor = new CANSparkMax(
        SwerveConstants.kRearLeftTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax rearLeftDriveMotor = new CANSparkMax(
        SwerveConstants.kRearLeftDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder rearLeftAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kRearLeftEncoderPort,
        SwerveConstants.AbsoluteEncoderConstants.rearLeftOffset);

    public CANSparkMax rearRightTurningMotor = new CANSparkMax(
        SwerveConstants.kRearRightTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax rearRightDriveMotor = new CANSparkMax(
        SwerveConstants.kRearRightDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder rearRightAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kRearRightEncoderPort,
        SwerveConstants.AbsoluteEncoderConstants.rearRightOffset);
  }

  public SwerveModuleHardware swerveModuleHardware;
  public SwerveDrivetrainHardware swerveDrivetrainHardware;

  /** Creates a new {@link HardwareMap} */
  public HardwareMap() {
    swerveModuleHardware = new SwerveModuleHardware();
    swerveDrivetrainHardware = new SwerveDrivetrainHardware();
  }
}