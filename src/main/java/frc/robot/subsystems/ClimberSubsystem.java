// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;

/** Subsystem that controls the climber. */
public class ClimberSubsystem extends SubsystemBase {
	private final CANSparkMax m_leftClimber = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);
	private final CANSparkMax m_rightClimber = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);

	private final Servo m_leftServo = new Servo(ClimberConstants.kLeftServoPort);
	private final Servo m_rightServo = new Servo(ClimberConstants.kRightServoPort);

	private double m_speed;

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftArmReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightArmReversed);
	}

	@Override
	public void periodic() {
		final double leftServoPosition = m_leftServo.get();
		final double rightServoPosition = m_rightServo.get();

		// Unlocks the servos before raising the arm.
		if (m_speed > 0) {
			m_leftServo.set(ClimberConstants.kLeftServoUnlockedPosition);
			m_rightServo.set(ClimberConstants.kRightServoUnlockedPosition);

			// Checks that both servos are released before running the motor by checking if
			// the position errors of the servos are within acceptable bounds of the
			// unlocked position.
			final boolean leftServoUnlocked = MathUtil.applyDeadband(
					leftServoPosition - ClimberConstants.kLeftServoUnlockedPosition,
					ClimberConstants.kServoDeadband) == 0;
			final boolean rightServoUnlocked = MathUtil.applyDeadband(
					rightServoPosition - ClimberConstants.kRightServoUnlockedPosition,
					ClimberConstants.kServoDeadband) == 0;

			m_leftClimber.set(leftServoUnlocked && rightServoUnlocked ? m_speed : 0);
			m_rightClimber.set(leftServoUnlocked && rightServoUnlocked ? m_speed : 0);
		} else {
			m_leftServo.set(ClimberConstants.kLeftServoLockedPosition);
			m_rightServo.set(ClimberConstants.kRightServoLockedPosition);

			m_leftClimber.set(m_speed);
			m_rightClimber.set(m_speed);
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Speed Desired", m_speed);
			SmartDashboard.putNumber("Climber Speed Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Speed Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", leftServoPosition);
			SmartDashboard.putNumber("Climber Servo Position Right", rightServoPosition);
		}
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed from -1 to 1.
	 */
	public void setSpeed(double speed) {
		m_speed = speed;
	}
}
