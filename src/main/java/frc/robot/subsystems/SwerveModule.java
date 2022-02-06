// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param driveMotor     motor that drives the wheel
   * @param turningMotor   motor that changes the angle of the wheel
   * @param turningEncoder absolute encoder for the swerve module
   */
  public SwerveModule(CANSparkMax driveMotor, CANSparkMax turningMotor, AbsoluteEncoder turningEncoder) {
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    m_turningEncoder = turningEncoder;

    m_driveMotor.getEncoder().setVelocityConversionFactor(ModuleConstants.kWheelCircumferenceMeters
        / 60 / ModuleConstants.kDrivingGearRatio);
        
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity(), m_turningEncoder.get());
  }

  /**
   * Stops the module from driving and turning. Use this so the wheels don't reset
   * to straight.
   */
  public void setDesiredState() {
    m_turningMotor.set(0);
    m_driveMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_turningEncoder.get());

    final double driveOutput = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get().getRadians(),
        state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }
}
