// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonShoot extends CommandBase {
	ShooterSubsystem m_subsystem;
	private final Timer m_timer = new Timer();

	/** Creates a new AutonIntake. */
	public AutonShoot(ShooterSubsystem subsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
		m_subsystem = subsystem;
	}
	@Override
	public void initialize() {
		m_timer.reset();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_timer.start();
		m_subsystem.setShooterSpeeds(ShooterConstants.kBottomShooterSpeedRPM, ShooterConstants.kTopShooterSpeedRPM);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_subsystem.setShooterSpeeds(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_timer.get() > 3;
	}
}
