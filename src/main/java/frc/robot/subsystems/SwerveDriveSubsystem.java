// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;

/** Controls the drivetrain of the robot using swerve. */
public class SwerveDriveSubsystem extends SubsystemBase {
  private SwerveModule m_frontLeftWheel;
  private SwerveModule m_rearLeftWheel;
  private SwerveModule m_frontRightWheel;
  private SwerveModule m_rearRightWheel;

  private ChassisSpeeds m_chassisSpeeds;

  /**
   * Creates a new {@link SwerveDriveSubsystem}.
   * 
   * @param hardware the hardware for the {@link SwerveDriveSubsystem}
   */
  public SwerveDriveSubsystem(SwerveDrivetrainHardware hardware) {
    m_frontLeftWheel = hardware.frontLeft;
    m_rearLeftWheel = hardware.rearLeft;
    m_frontRightWheel = hardware.frontRight;
    m_rearRightWheel = hardware.rearRight;
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
    m_chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    SwerveModuleState[] swerveModuleArray = SwerveConstants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    m_frontLeftWheel.setState(swerveModuleArray[0]);
    m_frontRightWheel.setState(swerveModuleArray[1]);
    m_rearLeftWheel.setState(swerveModuleArray[2]);
    m_rearRightWheel.setState(swerveModuleArray[3]);

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("state angle " + i, swerveModuleArray[i].angle.getDegrees());
    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Wheel", m_frontLeftWheel.getRadians());
    SmartDashboard.putNumber("Front Right Wheel", m_frontRightWheel.getRadians());
    SmartDashboard.putNumber("Rear Left Wheel", m_rearLeftWheel.getRadians());
    SmartDashboard.putNumber("Rear Right Wheel", m_rearRightWheel.getRadians());
  }
}
