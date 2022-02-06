// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
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
  private DoubleSupplier m_desiredXPosSupplier;
  private DoubleSupplier m_desiredYPosSupplier;
  private DoubleSupplier m_desiredRotSupplier;

  private DoubleSupplier m_xSpeedSupplier;
  private DoubleSupplier m_ySpeedSupplier;
  private DoubleSupplier m_rotSpeedSupplier;
  private BooleanSupplier m_fieldRelativeSupplier;

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
    // position and the default speed for the speed suppliers to PID.calculate().
    // Can be overridden by calling methods.
    m_desiredXPosSupplier = () -> m_driveSubsystem.getPose().getX();
    m_desiredYPosSupplier = () -> m_driveSubsystem.getPose().getY();
    m_desiredRotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians();

    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX());
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY());
    m_rotSpeedSupplier = () -> m_rotPID.calculate(m_driveSubsystem.getPose().getRotation().getRadians());
    m_fieldRelativeSupplier = () -> true;
  }

  @Override
  public void initialize() {
    m_xPID.setSetpoint(m_desiredXPosSupplier.getAsDouble());
    m_yPID.setSetpoint(m_desiredYPosSupplier.getAsDouble());
    m_rotPID.setSetpoint(m_desiredRotSupplier.getAsDouble());
  }

  @Override
  public void execute() {
    // TODO Add math to support driving in robot relative mode.
    m_driveSubsystem.drive(
        m_xSpeedSupplier.getAsDouble(),
        m_ySpeedSupplier.getAsDouble(),
        m_rotSpeedSupplier.getAsDouble(),
        m_fieldRelativeSupplier.getAsBoolean());
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
   * Sets the X speed of the robot using a {@link DoubleSupplier}.
   * 
   * @param x {@link DoubleSupplier} that returns the X speed.
   * @return This, for method chaining.
   */
  public MoveCommand withXSpeedSupplier(DoubleSupplier x) {
    m_xSpeedSupplier = x;
    return this;
  }

  /**
   * Sets the Y speed of the robot using a {@link DoubleSupplier}.
   * 
   * @param y {@link DoubleSupplier} that returns the Y speed.
   * @return This, for method chaining.
   */
  public MoveCommand withYSpeedSupplier(DoubleSupplier y) {
    m_ySpeedSupplier = y;
    return this;
  }

  /**
   * Sets the rotational speed of the robot using a {@link DoubleSupplier}.
   * 
   * @param rot {@link DoubleSupplier} that returns the rotational speed.
   * @return This, for method chaining.
   */
  public MoveCommand withRotSpeedSupplier(DoubleSupplier rot) {
    m_ySpeedSupplier = rot;
    return this;
  }

  /**
   * Sets whether the robot drives in field relative mode using a
   * {@link BooleanSupplier}.
   * 
   * @param fieldRelative {@link BooleanSupplier} that returns the whether the
   *                      robot is in field relative mode.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeSupplier(BooleanSupplier fieldRelative) {
    m_fieldRelativeSupplier = fieldRelative;
    return this;
  }

  /**
   * Sets the robot relative X position to drive to.
   * 
   * @param x Robot relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeX(double x) {
    m_desiredXPosSupplier = () -> Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()) * x;
    m_desiredYPosSupplier = () -> Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()) * x;
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX());
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY());
    return this;
  }

  /**
   * Sets the robot relative Y position to drive to.
   * 
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    m_desiredXPosSupplier = () -> Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()) * y;
    m_desiredYPosSupplier = () -> Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()) * y;
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX());
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY());
    return this;
  }

  /**
   * Changes the bot's field relative X position.
   * 
   * @param x change in field relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    m_desiredXPosSupplier = () -> m_driveSubsystem.getPose().getX() + x;
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX());
    return this;
  }

  /**
   * Changes the bot's field relative Y position.
   * 
   * @param y change in field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    m_desiredYPosSupplier = () -> m_driveSubsystem.getPose().getY() + y;
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY());
    return this;
  }

  /**
   * Sets the absolute X position to drive to.
   * 
   * @param y Absolute X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteX(double x) {
    m_desiredXPosSupplier = () -> x;
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX());
    return this;
  }

  /**
   * Sets the absolute Y position to drive to.
   * 
   * @param y Absolute Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteY(double y) {
    m_desiredYPosSupplier = () -> y;
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY());
    return this;
  }

  /**
   * changes the robot's heading to turn to.
   * 
   * @param rot Robot relative heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withChangeInHeading(double rot) {
    m_desiredRotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians() + Math.toRadians(rot);
    m_rotSpeedSupplier = () -> m_rotPID.calculate(m_driveSubsystem.getPose().getRotation().getRadians());
    return this;
  }

  /**
   * Sets the absolute heading to turn to.
   * 
   * @param rot Absolute heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    m_desiredRotSupplier = () -> Math.toRadians(rot);
    m_rotSpeedSupplier = () -> m_rotPID.calculate(m_driveSubsystem.getPose().getRotation().getRadians());
    return this;
  }
}
