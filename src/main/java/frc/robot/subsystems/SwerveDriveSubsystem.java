// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;

/** Controls the drivetrain of the robot using swerve. */
public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveDrivetrainHardware m_swerveDrivetrainHardware;
  private SwerveModule m_frontLeft;
  private SwerveModule m_rearLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_rearRight;
  private AHRS m_gyro;
  private SwerveDriveOdometry m_odometry;

  /**
   * Creates a new {@link SwerveDriveSubsystem}.
   * 
   * @param hardware the hardware for the {@link SwerveDriveSubsystem}
   */
  public SwerveDriveSubsystem(SwerveDrivetrainHardware hardware) {
    m_frontLeft = hardware.frontLeft;
    m_rearLeft = hardware.rearLeft;
    m_frontRight = hardware.frontRight;
    m_rearRight = hardware.rearRight;
    
    m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = SwerveConstants.kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(m_gyro.getAngle()), m_swerveDrivetrainHardware.frontLeft.getState(),
      m_swerveDrivetrainHardware.frontRight.getState(), m_swerveDrivetrainHardware.rearLeft.getState(),
      m_swerveDrivetrainHardware.rearRight.getState());
    SmartDashboard.putNumber("FrontLeft", m_frontLeft.getRadians());
    SmartDashboard.putNumber("FrontRight", m_frontRight.getRadians());
    SmartDashboard.putNumber("RearLeft", m_rearLeft.getRadians());
    SmartDashboard.putNumber("RearRight", m_rearRight.getRadians());
    SmartDashboard.putNumber("OdometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("OdometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("FL alsdfj;lkasdjf", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FL alsdfj;angle gangla nleg", m_frontLeft.getState().angle.getDegrees());
  }
}
