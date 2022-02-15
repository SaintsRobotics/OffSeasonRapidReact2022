// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that controls the RPM of the shooter by using a bang bang
 * controller.
 */
public class ShooterSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

	private final BangBangController m_bangBangController = new BangBangController();

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
	}

	@Override
	public void periodic() {
		m_shooterMotor.set(m_bangBangController.getSetpoint() == 0 ? 0
				: m_bangBangController.calculate(m_shooterMotor.getSelectedSensorVelocity()));

		SmartDashboard.putNumber("Current Shooter Power", m_shooterMotor.get());
		SmartDashboard.putNumber("Current Shooter Speed", m_shooterMotor.getSelectedSensorVelocity());
	}

	/**
	 * Sets the speed of the shooter.
	 * 
	 * @param speed Speed of the shooter in ticks per decisecond.
	 */
	public void set(double speed) {
		m_bangBangController.setSetpoint(speed);
		SmartDashboard.putNumber("Target Shooter Speed", speed);
	}
}
