// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

	private double m_leftSpeed;
	private double m_rightSpeed;

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftArmReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightArmReversed);
	}

	@Override
	public void periodic() {
		// Unlocks the servos before raising the left climber.
		m_leftServo.set(m_leftSpeed > 0 ? ClimberConstants.kLeftServoUnlockedPosition
				: ClimberConstants.kLeftServoLockedPosition);
		m_rightServo.set(m_rightSpeed > 0 ? ClimberConstants.kRightServoUnlockedPosition
				: ClimberConstants.kRightServoLockedPosition);

		m_leftClimber.set(m_leftSpeed);
		m_rightClimber.set(m_rightSpeed);

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Power Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Power Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", m_leftServo.get());
			SmartDashboard.putNumber("Climber Servo Position Right", m_rightServo.get());
		}
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed from -1 to 1.
	 */
	public void setSpeed(double speed) {
		setSpeed(speed, speed);
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param leftSpeed  Speed from -1 to 1.
	 * @param rightSpeed Speed from -1 to 1.
	 */
	public void setSpeed(double leftSpeed, double rightSpeed) {
		m_leftSpeed = leftSpeed;
		m_rightSpeed = rightSpeed;
	}
}
