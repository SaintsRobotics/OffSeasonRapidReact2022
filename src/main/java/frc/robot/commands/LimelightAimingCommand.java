// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;

/** Accesses {@link Limelight} values to aim the robot at a target. */
public class LimelightAimingCommand extends CommandBase {
	private final PIDController m_pid = new PIDController(0.2, 0, 0);

	private final MoveCommand m_moveCommand;
	private final int m_pipeline;

	/**
	 * Creates a new {@link LimelightAimingCommand}.
	 * 
	 * @param moveCommand {@link MoveCommand} that controls the translation of the
	 *                    robot.
	 * @param pipeline    Index of the pipeline to use.
	 */
	public LimelightAimingCommand(MoveCommand moveCommand, int pipeline) {
		m_moveCommand = moveCommand;
		m_pipeline = pipeline;
	}

	@Override
	public void initialize() {
		Limelight.setPipeline(m_pipeline);
		Limelight.setLED(0);
		Limelight.setCameraMode(0);
		m_moveCommand.withRotSpeedSupplier(() -> m_pid.calculate(Limelight.getX(), 0)).schedule();
	}

	@Override
	public void end(boolean interrupted) {
		Limelight.setLED(1);
		Limelight.setCameraMode(1);
		m_moveCommand.cancel();
	}
}
