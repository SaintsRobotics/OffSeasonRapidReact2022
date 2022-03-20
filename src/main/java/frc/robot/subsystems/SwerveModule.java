// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
	private final CANSparkMax m_driveMotor;
	private final CANSparkMax m_turningMotor;

	private final CANCoder m_turningEncoder;

	private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

	// Defaults to zeroed state.
	private SwerveModuleState m_desiredState = new SwerveModuleState();

	/**
	 * Creates a new {@link SwerveModule}.
	 * 
	 * @param driveMotorChannel     ID for the drive motor.
	 * @param turningMotorChannel   ID for the turning motor.
	 * @param turningEncoderChannel ID for the turning encoder.
	 * @param driveMotorReversed    Whether the drive motor is reversed.
	 * @param turningEncoderOffset  Offset of the turning encoder.
	 */
	public SwerveModule(
			int driveMotorChannel,
			int turningMotorChannel,
			int turningEncoderChannel,
			Boolean driveMotorReversed,
			double turningEncoderOffset) {
		m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
		m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
		m_turningEncoder = new CANCoder(turningEncoderChannel);

		// converts default units to meters per second
		m_driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kWheelDiameterMeters * Math.PI / 60 / ModuleConstants.kDrivingGearRatio);
		m_driveMotor.setIdleMode(IdleMode.kBrake);
		m_driveMotor.setInverted(driveMotorReversed);

		m_turningMotor.setIdleMode(IdleMode.kBrake);

		// converts default units to radians
		m_turningEncoder.configFeedbackCoefficient(Math.toRadians(0.087890625), "radians", SensorTimeBase.PerSecond);
		m_turningEncoder.configMagnetOffset(-turningEncoderOffset);

		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Returns the desired state if the robot is simulated.
		return Robot.isReal() ? new SwerveModuleState(
				m_driveMotor.getEncoder().getVelocity(),
				new Rotation2d(m_turningEncoder.getAbsolutePosition()))
				: m_desiredState;
	}

	/**
	 * Returns the absolute angle of the module. Use this to set the offset of the
	 * modules.
	 * 
	 * @return Absolute angle of the module from 0 to 360.
	 */
	public double getAbsoluteAngle() {
		return MathUtil.inputModulus(
				Math.toDegrees(m_turningEncoder.getAbsolutePosition()) - m_turningEncoder.configGetMagnetOffset(), 0,
				360);
	}

	/**
	 * Returns the turning motor.
	 * 
	 * @return The turning motor.
	 */
	public CANSparkMax getTurningMotor() {
		return m_turningMotor;
	}

	/**
	 * Returns the drive motor.
	 * 
	 * @return The drive motor.
	 */
	public CANSparkMax getDriveMotor() {
		return m_driveMotor;
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
		m_desiredState = SwerveModuleState.optimize(desiredState,
				new Rotation2d(m_turningEncoder.getAbsolutePosition()));

		final double driveOutput = m_desiredState.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
		final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(),
				m_desiredState.angle.getRadians());

		m_driveMotor.set(driveOutput);
		m_turningMotor.set(turnOutput);
	}

	public void setIdle() {
		m_driveMotor.setIdleMode(IdleMode.kCoast);
	}

	public void setBrake() {
		m_driveMotor.setIdleMode(IdleMode.kBrake);
	}
}
