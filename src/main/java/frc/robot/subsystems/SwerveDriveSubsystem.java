// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  // TODO tune pid
  private final PIDController m_headingCorrectionPID = new PIDController(1.5, 0, 0);
  private final Timer m_headingCorrectionTimer;

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
    m_gyro.calibrate();

    m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, m_gyro.getRotation2d());
    m_gyro.reset();
    m_odometry.resetPosition(new Pose2d(), m_gyro.getRotation2d());

    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
    m_headingCorrectionPID.setSetpoint(MathUtil.angleModulus(m_gyro.getRotation2d().getRadians()));
    m_headingCorrectionTimer = new Timer();
    m_headingCorrectionTimer.start();
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_frontLeft.getState(), m_rearLeft.getState(), m_frontRight.getState(),
        m_rearRight.getState());

    SmartDashboard.putNumber("Module Angle Front Left", m_frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Module Angle Rear Left", m_rearLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("Module Angle Front Right", m_frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("Module Angle Rear Right", m_rearRight.getState().angle.getDegrees());

    SmartDashboard.putNumber("Module Speed Front Left", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Module Speed Rear Left", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Module Speed Front Right", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Module Speed Rear Right", m_rearRight.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("Odometry X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Rot", m_odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("Gyro Angle", MathUtil.inputModulus(m_gyro.getAngle(), 0, 360));

    SmartDashboard.putNumber("Heading Correction Setpoint", Math.toDegrees(m_headingCorrectionPID.getSetpoint()));
    SmartDashboard.putNumber("Heading Correction Timer", m_headingCorrectionTimer.get());
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
   * Method to drive the robot.
   *
   * @param xSpeed        Speed of the robot in the x direction in meters per
   *                      second (forward). Positive is forward.
   * @param ySpeed        Speed of the robot in the y direction in meters per
   *                      second (sideways). Positive is left.
   * @param rot           Angular rate of the robot in radians per second.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field. Positive is counterclockwise.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double rotation = rot;

    // resets the timer when the robot is turning, used to measure the time since the robot has stopped turning
    if (rot != 0) {
      m_headingCorrectionTimer.reset();
    }

    // corrects the heading of the robot to prevent it from drifting
    double currentAngle = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());

    if ((xSpeed == 0 && ySpeed == 0) || m_headingCorrectionTimer.get() < SwerveConstants.kTurningStopTime) {
      m_headingCorrectionPID.setSetpoint(currentAngle);
      SmartDashboard.putString("Heading Correction", "Setting Setpoint");
    } else {
      rotation = m_headingCorrectionPID.calculate(currentAngle);
      SmartDashboard.putString("Heading Correction", "Correcting Heading");
    }

    // this check prevents the wheels from resetting to straight when the robot
    // stops moving
    if (xSpeed == 0 && ySpeed == 0 && rotation == 0) {
      m_frontLeft.setDesiredState();
      m_rearLeft.setDesiredState();
      m_frontRight.setDesiredState();
      m_rearRight.setDesiredState();
    } else {
      SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeed, ySpeed, rotation));

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_rearLeft.setDesiredState(swerveModuleStates[1]);
      m_frontRight.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    SmartDashboard.putNumber("Desired X", xSpeed);
    SmartDashboard.putNumber("Desired Y", ySpeed);
    SmartDashboard.putNumber("Desired Rot", Math.toDegrees(rotation));
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
}
