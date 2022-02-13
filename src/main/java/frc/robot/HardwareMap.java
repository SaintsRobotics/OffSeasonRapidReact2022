package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

/** File for storing the hardware of the robot. */
public class HardwareMap {
  /** File for storing the hardware of the drivetrain. */
  public class SwerveDrivetrainHardware {
    public SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.kFrontLeftDriveMotorPort,
        SwerveConstants.kFrontLeftTurningMotorPort,
        SwerveConstants.kFrontLeftTurningEncoderPort,
        SwerveConstants.kFrontLeftTurningEncoderReversed,
        SwerveConstants.kFrontLeftTurningEncoderOffset);
    public SwerveModule rearLeft = new SwerveModule(
        SwerveConstants.kRearLeftDriveMotorPort,
        SwerveConstants.kRearLeftTurningMotorPort,
        SwerveConstants.kRearLeftTurningEncoderPort,
        SwerveConstants.kRearLeftTurningEncoderReversed,
        SwerveConstants.kRearLeftTurningEncoderOffset);
    public SwerveModule frontRight = new SwerveModule(
        SwerveConstants.kFrontRightDriveMotorPort,
        SwerveConstants.kFrontRightTurningMotorPort,
        SwerveConstants.kFrontRightTurningEncoderPort,
        SwerveConstants.kFrontRightTurningEncoderReversed,
        SwerveConstants.kFrontRightTurningEncoderOffset);
    public SwerveModule rearRight = new SwerveModule(
        SwerveConstants.kRearRightDriveMotorPort,
        SwerveConstants.kRearRightTurningMotorPort,
        SwerveConstants.kRearRightTurningEncoderPort,
        SwerveConstants.kRearRightTurningEncoderReversed,
        SwerveConstants.kRearRightTurningEncoderOffset);

    public AHRS gyro = new AHRS();
  }

  public SwerveDrivetrainHardware swerveDrivetrainHardware;

  /** Creates a new {@link HardwareMap}. */
  public HardwareMap() {
    swerveDrivetrainHardware = new SwerveDrivetrainHardware();
  }
}
