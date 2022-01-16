// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AbsoluteEncoder;
import frc.robot.HardwareMap.SwerveModuleHardware;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AbsoluteEncoder m_turningEncoder;
  
  private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param hardware     the hardware for the swerve module
   * @param driveMotor   motor that drives the wheel
   * @param turningMotor motor that changes the angle of the wheel
   * @param turningEncoder   absolute encoder for the swerve module
   */
  public SwerveModule(SwerveModuleHardware hardware, CANSparkMax driveMotor, CANSparkMax turningMotor,
      AbsoluteEncoder turningEncoder) {
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    m_turningEncoder = turningEncoder;

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_turningEncoder.getRotation2d());

    final double driveOutput = state.speedMetersPerSecond;
    final var turnOutput = m_turningPIDController.calculate(m_turningEncoder.getRotation2d().getRadians(),
        state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /**
   * Returns the current radian value from the encoder.
   * 
   * @return the current radian value from the encoder
   */
  public double getRadians() {
    return m_turningEncoder.getRotation2d().getRadians();
  }

  public double getPIDError() {
    return m_turningPIDController.getPositionError();
  }
}
