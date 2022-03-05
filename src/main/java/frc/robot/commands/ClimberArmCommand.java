// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberArmSubsystem;

public class ClimberArmCommand extends CommandBase {
	private ClimberArmSubsystem m_climberSubsystem;
	private XboxController m_operatorController;

	/** Creates a new ClimberArmCommand. */
	public ClimberArmCommand(ClimberArmSubsystem climberSubsystem, XboxController operatorController) {
		m_climberSubsystem = climberSubsystem;
		m_operatorController = operatorController;
		addRequirements(m_climberSubsystem);
		
	}

	@Override
	public void execute() {
		if (m_operatorController.getLeftY() > .5) {
			m_climberSubsystem.setSpeed(-.5);
		}
		else if (m_operatorController.getLeftY() < -.5) {
			m_climberSubsystem.setSpeed(.5);
		}
		else {
			m_climberSubsystem.setSpeed(0);
		}
	}
}
