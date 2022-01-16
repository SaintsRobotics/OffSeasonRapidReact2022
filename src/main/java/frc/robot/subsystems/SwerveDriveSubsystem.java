// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;

/** Controls the drivetrain of the robot using swerve. */
public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeft;
  private SwerveModule m_rearLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_rearRight;

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
    m_frontLeft.setState(swerveModuleStates[0]);
    m_frontRight.setState(swerveModuleStates[1]);
    m_rearLeft.setState(swerveModuleStates[2]);
    m_rearRight.setState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PID error FL", m_frontLeft.getPIDError());
    SmartDashboard.putNumber("PID error RL", m_rearLeft.getPIDError());
    SmartDashboard.putNumber("PID error FR", m_frontRight.getPIDError());
    SmartDashboard.putNumber("PID error RR", m_rearRight.getPIDError());
    SmartDashboard.putNumber("FrontLeft", m_frontLeft.getRadians());
    SmartDashboard.putNumber("FrontRight", m_frontRight.getRadians());
    SmartDashboard.putNumber("RearLeft", m_rearLeft.getRadians());
    SmartDashboard.putNumber("RearRight", m_rearRight.getRadians());
  }
}
