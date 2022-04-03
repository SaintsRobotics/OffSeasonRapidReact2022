// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Command that shoots up to two balls */
public class ShootFender extends CommandBase {
	private final ShooterSubsystem m_subsystem;
	private final Timer m_timer = new Timer();

	/**
	 * Creates a new {@link ShootFender}.
	 * 
	 * @param subsystem The required subsystem.
	 */
	public ShootFender(ShooterSubsystem subsystem) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		m_subsystem.setShooterSpeed(ShooterSubsystem.Mode.kFender);
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.setShooterSpeed(ShooterSubsystem.Mode.kEnd);
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 5;
	}
}
