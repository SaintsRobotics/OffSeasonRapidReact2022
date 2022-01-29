// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveDrivetrainHardware;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutonMoveCommand extends CommandBase {

  SwerveDriveSubsystem m_swerveSubsystem;
  double xSpeed;
  double ySpeed;
  double rotSpeed;

  /** Creates a new AutonCommand. */
  public AutonMoveCommand(SwerveDriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_swerveSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  /**
   * movex command
   */
  public AutonMoveCommand changeXBy(){

    return this; 
  }
  public AutonMoveCommand changeYBy(){

    return this; 
  }
  public AutonMoveCommand changeHeadingBy(){

    return this; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
