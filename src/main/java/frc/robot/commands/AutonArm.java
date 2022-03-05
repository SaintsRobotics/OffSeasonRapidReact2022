// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonArm extends CommandBase {
  ShooterSubsystem m_subsystem;
  double m_setpoint;
  double timer = 0;
  /** Creates a new AutonIntake. */
  public AutonArm(ShooterSubsystem subsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_subsystem = subsystem;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 0.02;
    if (m_setpoint == ShooterConstants.kUpperArmAngle) {
      m_subsystem.raiseArm();
    } else if (m_setpoint == ShooterConstants.kLowerArmAngle) {
      m_subsystem.lowerArm();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2;
  }
}
