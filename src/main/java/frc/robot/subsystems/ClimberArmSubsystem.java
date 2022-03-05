// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberArmSubsystem extends SubsystemBase {
	private CANSparkMax m_leftClimberArm = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);
	private CANSparkMax m_rightClimberArm = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);

	// private CANCoder m_leftEncoder = new
	// CANCoder(ClimberConstants.kLeftEncoderPort);
	// private CANCoder m_rightEncoder = new
	// CANCoder(ClimberConstants.kRightEncoderPort);

	private PIDController m_leftPID = new PIDController(0.3, 0, 0);
	private PIDController m_rightPID = new PIDController(0.3, 0, 0);

	private Servo m_leftServo = new Servo(ClimberConstants.kLeftServoPort);
	private Servo m_rightServo = new Servo(ClimberConstants.kRightServoPort);

	private double m_climbSpeed;

	public boolean m_isLocked = true;

	/** Creates a new {@link ClimberArmSubsystem}. */
	public ClimberArmSubsystem() {
		m_isLocked = true;
		m_leftClimberArm.setIdleMode(IdleMode.kCoast);
		m_rightClimberArm.setIdleMode(IdleMode.kCoast);
		m_leftClimberArm.setInverted(true);
		m_rightClimberArm.setInverted(false);
	}

	// public void realignArms() {
	// if (m_leftEncoder.getPosition() < m_rightEncoder.getPosition()) {
	// m_leftPID.setSetpoint(m_rightEncoder.getPosition());
	// m_leftSpeed = m_leftPID.calculate(m_leftEncoder.getPosition());
	// } else {
	// m_rightPID.setSetpoint(m_leftEncoder.getPosition());
	// m_rightSpeed = m_rightPID.calculate(m_rightEncoder.getPosition());
	// }
	// }

	public void setSpeed(double speed) {
		m_climbSpeed = speed;

	}

	public void releaseServos() {
		m_leftServo.set(ClimberConstants.kLeftServoReleasedPos);
		m_rightServo.set(ClimberConstants.kRightServoReleasedPos);
		m_isLocked = false;
	}

	public void lockServos() {
		m_leftServo.set(ClimberConstants.kLeftServoLockedPos);
		m_rightServo.set(ClimberConstants.kRightServoLockedPos);
		m_isLocked = true;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Speed", m_leftClimberArm.get()); // Same as right arm
		if (!m_isLocked || m_climbSpeed < 0) {
			m_leftClimberArm.set(m_climbSpeed);
			m_rightClimberArm.set(m_climbSpeed);
		}
		else {
			m_leftClimberArm.set(0);
			m_rightClimberArm.set(0);
			m_climbSpeed = 0;
		} 

		SmartDashboard.putBoolean("should_lock", m_isLocked);
		SmartDashboard.putNumber("Desired Climber Speed", m_climbSpeed);
		SmartDashboard.putNumber("Climber Left Servo Position", m_leftServo.get());
		SmartDashboard.putNumber("Climber Right Servo Position", m_rightServo.get());
		// SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
		// SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
	}
}
