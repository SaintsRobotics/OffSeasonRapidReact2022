// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Moves the robot using robot relative, field relative, or absolute values. */
public class MoveCommand extends CommandBase {

  SwerveDriveSubsystem m_swerveSubsystem;
  double xSpeed;
  double ySpeed;
  double rotSpeed;

  PIDController xPID = new PIDController(0.3, 0, 0);
  PIDController yPID = new PIDController(0.3, 0, 0);
  PIDController rotPID = new PIDController(0.3, 0, 0);

  Pose2d currentPose = new Pose2d();

  /** Creates a new {@link MoveCommand}. */
  public MoveCommand(SwerveDriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = subsystem;
    addRequirements(subsystem);
    xPID.setTolerance(0.1);
    yPID.setTolerance(0.1);
    rotPID.setTolerance(Math.PI/18);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = m_swerveSubsystem.getPose();
    xPID.setSetpoint(currentPose.getX());
    yPID.setSetpoint(currentPose.getY());
    rotPID.setSetpoint(currentPose.getRotation().getRadians());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = m_swerveSubsystem.getPose();
    xSpeed = xPID.calculate(currentPose.getX());
    ySpeed = yPID.calculate(currentPose.getY());
    rotSpeed = rotPID.calculate(currentPose.getRotation().getRadians());

    m_swerveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint());
  }

  /**
   * Sets the robot relative X position to drive to.
   * 
   * @param x Robot relative X position.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeX(double x) {
    xPID.setSetpoint(Math.cos(currentPose.getRotation().getRadians()) * x);
    yPID.setSetpoint(Math.sin(currentPose.getRotation().getRadians()) * x);
    return this; 
  }

  /**
   * Sets the robot relative Y position to drive to.
   * 
   * @param y Robot relative Y position.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    xPID.setSetpoint(Math.sin(currentPose.getRotation().getRadians()) * y);
    yPID.setSetpoint(Math.cos(currentPose.getRotation().getRadians()) * y);
    return this; 
  }

  /**
   * Sets the field relative X position to drive to.
   * 
   * @param x Field relative X position.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    xPID.setSetpoint(x + m_swerveSubsystem.getPose().getX());
    return this; 
  }

  /**
   * Sets the field relative Y position to drive to.
   * 
   * @param y Field relative Y position.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    yPID.setSetpoint(y + m_swerveSubsystem.getPose().getY());
    return this; 
  }

  /**
   * Sets the absolute X position to drive to.
   * 
   * @param y Absolute X position.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteX(double x) {
    xPID.setSetpoint(x);
    return this;
  }

  /**
   * Sets the absolute Y position to drive to.
   * 
   * @param y Absolute Y position.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteY(double y) {
    yPID.setSetpoint(y);
    return this;
  }

  /**
   * Sets the robot relative heading to turn to.
   * 
   * @param rot Robot relative heading.
   * @return This, for method chaining.
   */
  public MoveCommand withRelativeHeading(double rot) {
    rotPID.setSetpoint(Math.toRadians(rot) + m_swerveSubsystem.getPose().getRotation().getRadians());
    return this;
  }

  /**
   * Sets the absolute heading to turn to.
   * 
   * @param rot Absolute heading.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    rotPID.setSetpoint(Math.toRadians(rot));
    return this;
  }
}
