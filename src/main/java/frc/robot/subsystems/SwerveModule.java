// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
	private final CANSparkMax m_driveMotor;
	private final CANSparkMax m_turningMotor;

	private final AnalogEncoder m_turningEncoder;
	private final AnalogEncoderSim m_turningEncoderSim;

	private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

	private SwerveModuleState m_desiredState = new SwerveModuleState();

	/**
	 * Creates a new {@link SwerveModule}.
	 * 
	 * @param driveMotorChannel     ID for the drive motor.
	 * @param turningMotorChannel   ID for the turning motor.
	 * @param turningEncoderChannel ID for the turning encoder.
	 * @param turningEncoderOffset  Offset of the turning encoder.
	 */
	public SwerveModule(
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderChannel,
			double turningEncoderOffset) {
		m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		m_turningEncoder = new AnalogEncoder(turningEncoderChannel);
		m_turningEncoderSim = new AnalogEncoderSim(m_turningEncoder);
		m_turningEncoder.setPositionOffset(turningEncoderOffset);

		// We want the encoder value to increase as the wheel is turned
		// counter-clockwise so we need to negate the distance per rotation
		m_turningEncoder.setDistancePerRotation(-2 * Math.PI);

		m_driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kWheelCircumferenceMeters / 60 / ModuleConstants.kDrivingGearRatio);

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
		return new SwerveModuleState(
				Robot.isReal() ? m_driveMotor.getEncoder().getVelocity() : m_desiredState.speedMetersPerSecond,
				new Rotation2d(m_turningEncoder.get()));
	}

	/**
	 * Returns the absolute angle of the module. Use this to set the offset of the
	 * modules.
	 * 
	 * @return Absolute angle of the module from 0-1.
	 */
	public double getAbsoluteAngle() {
		return m_turningEncoder.getAbsolutePosition();
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
		m_desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

		final double driveOutput = m_desiredState.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
		final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(),
				m_desiredState.angle.getRadians());

		m_driveMotor.set(driveOutput);
		m_turningMotor.set(turnOutput);
		m_turningEncoderSim.setTurns(m_desiredState.angle.getRadians());
	}

	public void setIdle() {
		m_driveMotor.setIdleMode(IdleMode.kCoast);
	}

	public void setBrake() {
		m_driveMotor.setIdleMode(IdleMode.kBrake);
	}
}
