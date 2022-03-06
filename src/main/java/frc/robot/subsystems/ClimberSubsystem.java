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

	private double m_climbSpeed;

	public boolean m_isLocked = true;

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftArmReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightArmReversed);
	}

	public void setSpeed(double speed) {
		m_climbSpeed = speed;
	}

	public void unlock() {
		m_leftServo.set(ClimberConstants.kLeftServoReleasedPos);
		m_rightServo.set(ClimberConstants.kRightServoReleasedPos);
		m_isLocked = false;
	}

	public void lock() {
		m_leftServo.set(ClimberConstants.kLeftServoLockedPos);
		m_rightServo.set(ClimberConstants.kRightServoLockedPos);
		m_isLocked = true;
	}

	@Override
	public void periodic() {
		if (!m_isLocked || m_climbSpeed < 0) {
			m_leftClimber.set(m_climbSpeed);
			m_rightClimber.set(m_climbSpeed);
		} else {
			m_leftClimber.set(0);
			m_rightClimber.set(0);
			m_climbSpeed = 0;
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putBoolean("Climber Locked", m_isLocked);
			SmartDashboard.putNumber("Climber Speed Desired", m_climbSpeed);
			SmartDashboard.putNumber("Climber Speed Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Speed Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", m_leftServo.get());
			SmartDashboard.putNumber("Climber Servo Position Right", m_rightServo.get());
		}
	}
}
