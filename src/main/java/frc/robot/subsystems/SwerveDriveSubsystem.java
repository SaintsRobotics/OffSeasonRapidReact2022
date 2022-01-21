// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;

/** Controls the drivetrain of the robot using swerve. */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearRight;

  private final AHRS m_gyro;
  private final SwerveDriveOdometry m_odometry;

  private boolean m_isTurning;
  private PIDController m_headPidController;
  private double m_rotSpeed;
  private double m_ySpeed;
  private double m_xSpeed;

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

    m_gyro = hardware.gyro;

    m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {

    //Odometry 
    m_odometry.update(m_gyro.getRotation2d(), m_frontLeft.getState(),
        m_frontRight.getState(), m_rearLeft.getState(),
        m_rearRight.getState());
    SmartDashboard.putNumber("FrontLeft", m_frontLeft.getRadians());
    SmartDashboard.putNumber("FrontRight", m_frontRight.getRadians());
    SmartDashboard.putNumber("RearLeft", m_rearLeft.getRadians());
    SmartDashboard.putNumber("RearRight", m_rearRight.getRadians());
    SmartDashboard.putNumber("OdometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("OdometryRot", m_odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("FL speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FL angle", m_frontLeft.getState().angle.getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info. (and heading correction)
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_xSpeed=xSpeed;
    m_ySpeed=ySpeed;
    m_rotSpeed=rot;
/*    if (m_isTurning) { // if should be turning
      m_headPidController.setSetpoint(Math.toRadians(m_gyro.getAngle()));
      SmartDashboard.putString("heading correction", "setting setpoint");
    }
    if (!m_isTurning && (m_xSpeed != 0 || m_ySpeed != 0)) { // if translating only (want heading correction)
      SmartDashboard.putString("heading correction", "correcting heading");
      m_rotSpeed = m_headPidController.calculate(Utils.normalizeAngle(Math.toRadians(m_gyro.getAngle()), 2 * Math.PI));
    }
    else { 
      SmartDashboard.putString("heading correction", "not correcting heading, not translating");
    } */
    if (m_xSpeed == 0 && m_ySpeed == 0 && m_rotSpeed == 0) {
      m_frontLeft.setDesiredState();
      m_frontRight.setDesiredState();
      m_rearLeft.setDesiredState();
      m_rearRight.setDesiredState();
    } else {
      SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics
          .toSwerveModuleStates(fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rotSpeed, m_gyro.getRotation2d())
              : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rotSpeed));

      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
}
