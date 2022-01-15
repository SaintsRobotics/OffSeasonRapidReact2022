// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AbsoluteEncoder;
import frc.robot.HardwareMap.SwerveModuleHardware;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private AbsoluteEncoder m_absoluteEncoder;
  private CANSparkMax m_driveMotor;
  private CANSparkMax m_turningMotor;
  private PIDController m_pidController;
  private double PIDCalculate;

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param hardware     the hardware for the swerve module
   * @param driveMotor   motor that drives the wheel
   * @param turningMotor motor that changes the angle of the wheel
   * @param absEncoder   absolute encoder for the swerve module
   */
  public SwerveModule(SwerveModuleHardware hardware, CANSparkMax driveMotor, CANSparkMax turningMotor,
      AbsoluteEncoder absEncoder) {
    m_absoluteEncoder = absEncoder;
    m_pidController = new PIDController(0.3, 0, 0);
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setState(SwerveModuleState desiredState) {
    m_turningMotor
        .set(m_pidController.calculate(m_absoluteEncoder.getRotation2d().getRadians(), desiredState.angle.getRadians()));
    m_driveMotor.set(desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("pid measurement", m_absoluteEncoder.getRotation2d().getRadians());
    SmartDashboard.putNumber("pid setpoint", m_absoluteEncoder.getRotation2d().getRadians());

  }

  /**
   * Returns the current radian value from the encoder.
   * 
   * @return the current radian value from the encoder
   */
  public double getRadians() {
    return m_absoluteEncoder.getRotation2d().getRadians();
  }

  public double getPIDCalculate() {
    return PIDCalculate;
  }
}
