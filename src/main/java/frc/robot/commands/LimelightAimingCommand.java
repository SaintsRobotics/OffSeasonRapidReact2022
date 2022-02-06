// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Accesses {@link Limelight} values to aim the robot at a target. */
public class LimelightAimingCommand extends CommandBase {
  private final SwerveDriveSubsystem m_swerveSubsystem;

  private final PIDController m_pid = new PIDController(0.03, 0, 0);

  private final int m_pipeline;

  /**
   * Creates a new {@link LimelightAimingCommand}.
   * 
   * @param subsystem The required subsystem.
   * @param pipeline  Index of the pipeline to use.
   */
  public LimelightAimingCommand(SwerveDriveSubsystem subsystem, int pipeline) {
    m_swerveSubsystem = subsystem;
    addRequirements(m_swerveSubsystem);

    m_pipeline = pipeline;
  }

  @Override
  public void initialize() {
    Limelight.setPipeline(m_pipeline);
    Limelight.setLed(0);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.drive(0, 0, m_pid.calculate(Limelight.getX(), 0), false);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
    Limelight.setLed(1);
  }
}
