// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** Command that turns the shooter on and off. */
public class ShooterCommand extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;

  /**
   * Creates a new {@link ShooterCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public ShooterCommand(ShooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
    addRequirements(m_shooterSubsystem);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.set(ShooterConstants.kShooterSpeedTicksPerDecisecond);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.set(0);
  }
}
