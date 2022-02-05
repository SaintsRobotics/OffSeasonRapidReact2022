// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LimelightAimingCommand extends CommandBase {
  private final PIDController m_pid = new PIDController(0.03, 0, 0);
  private SwerveDriveSubsystem m_swerveSubsystem;

  private final int m_pipeline;

  /** Creates a new {@link LimelightAimingCommand}. */
  public LimelightAimingCommand(SwerveDriveSubsystem swerveSubsystem, int pipeline) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_swerveSubsystem);

    m_pipeline = pipeline;
  }

  @Override
  public void initialize() {
    m_pid.reset();
    Limelight.setPipeline(m_pipeline);
    Limelight.setLed(0);
    m_pid.setSetpoint(0);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.drive(0, 0, m_pid.calculate(Limelight.getX()), false);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
    Limelight.setLed(1);
  }
}
