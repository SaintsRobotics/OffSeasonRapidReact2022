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

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftClimberReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightClimberReversed);
	}

	@Override
	public void periodic() {
		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Speed Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Speed Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", m_leftServo.get());
			SmartDashboard.putNumber("Climber Servo Position Right", m_rightServo.get());
		}
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed from -1 to 1.
	 */
	public void set(double speed) {
		set(speed, speed);
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param leftSpeed  Speed from -1 to 1.
	 * @param rightSpeed Speed from -1 to 1.
	 */
	public void set(double leftSpeed, double rightSpeed) {
		// Unlocks the servos before raising the left climber.
		m_leftServo.set(leftSpeed > 0 ? ClimberConstants.kLeftServoUnlockedPosition
				: ClimberConstants.kLeftServoLockedPosition);
		m_rightServo.set(rightSpeed > 0 ? ClimberConstants.kRightServoUnlockedPosition
				: ClimberConstants.kRightServoLockedPosition);

		m_leftClimber.set(leftSpeed);
		m_rightClimber.set(rightSpeed);
	}
}
