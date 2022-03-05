// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonIntake extends CommandBase {
	ShooterSubsystem m_subsystem;
	double timer = 0;

	/** Creates a new AutonIntake. */
	public AutonIntake(ShooterSubsystem subsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
		m_subsystem = subsystem;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.intake();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		timer += 0.02;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_subsystem.intakeOff();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer > 1;
	}
}
