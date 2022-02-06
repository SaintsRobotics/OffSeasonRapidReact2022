// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Moves the robot using robot relative, field relative, or absolute values. */
public class MoveCommand extends CommandBase {
  private final SwerveDriveSubsystem m_driveSubsystem;

  private final PIDController m_xPID = new PIDController(0.3, 0, 0);
  private final PIDController m_yPID = new PIDController(0.3, 0, 0);
  private final PIDController m_rotPID = new PIDController(0.3, 0, 0);

  // Double suppliers are necessary because we want to use the position of the
  // robot when the command is run and not when the command is constructed.
  private DoubleSupplier m_xSupplier;
  private DoubleSupplier m_ySupplier;
  private DoubleSupplier m_rotSupplier;

  /**
   * Creates a new {@link MoveCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public MoveCommand(SwerveDriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;
    addRequirements(m_driveSubsystem);

    m_xPID.setTolerance(0.05);
    m_yPID.setTolerance(0.05);
    m_rotPID.setTolerance(0.1);

    // Sets the default position for the desired position suppliers to the current
    // position. Can be overridden by calling methods.
    m_xSupplier = () -> m_driveSubsystem.getPose().getX();
    m_ySupplier = () -> m_driveSubsystem.getPose().getY();
    m_rotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians();
  }

  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_xSupplier.getAsDouble());
    m_yPID.setSetpoint(m_ySupplier.getAsDouble());
    m_rotPID.setSetpoint(m_rotSupplier.getAsDouble());
  }

  @Override
  public void execute() {
    m_driveSubsystem.drive(
        m_xPID.calculate(m_driveSubsystem.getPose().getX()),
        m_yPID.calculate(m_driveSubsystem.getPose().getY()),
        m_rotPID.calculate(m_driveSubsystem.getPose().getRotation().getRadians()),
        true);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
  }

  /**
   * Sets the robot relative X position to drive to.
   * 
   * @param x Robot relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeX(double x) {
    m_xSupplier = () -> m_driveSubsystem.getPose().getX()
        + (x * Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()));
    m_ySupplier = () -> m_driveSubsystem.getPose().getY()
        + (x * Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()));
    return this;
  }

  /**
   * Sets the robot relative Y position to drive to.
   * 
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    m_xSupplier = () -> m_driveSubsystem.getPose().getX()
        + (y * Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()));
    m_ySupplier = () -> m_driveSubsystem.getPose().getY()
        + (y * Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()));
    return this;
  }

  /**
   * // TODO y value will override the x value
   * Sets the robot relative position to drive to.
   * 
   * @param x Robot relative X position in meters.
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativePos(double x, double y) {
    withRobotRelativeX(x);
    withRobotRelativeY(y);
    return this;
  }

  /**
   * Changes the bot's field relative X position.
   * 
   * @param x change in field relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    m_xSupplier = () -> m_driveSubsystem.getPose().getX() + x;
    return this;
  }

  /**
   * Changes the bot's field relative Y position.
   * 
   * @param y change in field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    m_ySupplier = () -> m_driveSubsystem.getPose().getY() + y;
    return this;
  }

  /**
   * Changes the bot's field relative position.
   * 
   * @param x change in field relative X position in meters.
   * @param y change in field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativePos(double x, double y) {
    withFieldRelativeX(x);
    withFieldRelativeY(y);
    return this;
  }

  /**
   * Sets the absolute X position to drive to.
   * 
   * @param y Absolute X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteX(double x) {
    m_xSupplier = () -> x;
    return this;
  }

  /**
   * Sets the absolute Y position to drive to.
   * 
   * @param y Absolute Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteY(double y) {
    m_ySupplier = () -> y;
    return this;
  }

  /**
   * changes the robot's heading to turn to.
   * 
   * @param rot Robot relative heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withChangeInHeading(double rot) {
    m_rotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians() + Math.toRadians(rot);
    return this;
  }

  /**
   * Sets the absolute heading to turn to.
   * 
   * @param rot Absolute heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    m_rotSupplier = () -> Math.toRadians(rot);
    return this;
  }
}
