// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Command for raising or lowering the arm. */
public class ArmCommand extends CommandBase {
	private final ShooterSubsystem m_subsystem;
	private final double m_setpoint;
	private final Timer m_timer = new Timer();

	/**
	 * Creates a new {@link ArmCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 * @param setpoint  Set to {@link ShooterConstants#kUpperArmAngle} to raise the
	 *                  arm. Set to {@link ShooterConstants#kLowerArmAngle} to lower
	 *                  the arm.
	 */
	public ArmCommand(ShooterSubsystem subsystem, double setpoint) {
		addRequirements(subsystem);
		m_subsystem = subsystem;
		m_setpoint = setpoint;
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		if (m_setpoint == ShooterConstants.kUpperArmAngle) {
			m_subsystem.raiseArm();
		} else if (m_setpoint == ShooterConstants.kLowerArmAngle) {
			m_subsystem.lowerArm();
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.stopArm();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > 0.6;
	}
}
