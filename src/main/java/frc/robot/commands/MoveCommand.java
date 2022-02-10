// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Moves the robot using robot relative, field relative, or absolute values.
 * Calling multiple relative methods will add the positions together.
 */
public class MoveCommand extends CommandBase {
  private final SwerveDriveSubsystem m_driveSubsystem;

  private final PIDController m_xPID = new PIDController(4, 0, 0);
  private final PIDController m_yPID = new PIDController(4, 0, 0);
  private final PIDController m_rotPID = new PIDController(3, 0, 0);

  private Pose2d m_startPose;

  /**
   * Change in the field relative X position of the robot based on the
   * {@link #m_startPose starting position}. Equal to 0 by default.
   */
  private double m_deltaX = 0;

  /**
   * Change in the field relative Y position of the robot based on the
   * {@link #m_startPose starting position}. Equal to 0 by default.
   */
  private double m_deltaY = 0;

  /**
   * Change in the angle of the robot based on the {@link #m_startPose starting
   * position}. Equal to 0 by default.
   */
  private double m_deltaRot = 0;

  private DoubleSupplier m_xSpeedSupplier;
  private DoubleSupplier m_ySpeedSupplier;
  private DoubleSupplier m_rotSpeedSupplier;
  private BooleanSupplier m_fieldRelativeSupplier = () -> true;

  /**
   * Creates a new {@link MoveCommand}. Call methods to make the robot move.
   * 
   * @param subsystem The required subsystem.
   */
  public MoveCommand(SwerveDriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;
    addRequirements(m_driveSubsystem);

    m_xPID.setTolerance(0.05);
    m_yPID.setTolerance(0.05);
    m_rotPID.setTolerance(0.05);
    m_rotPID.enableContinuousInput(-Math.PI, Math.PI);

    // Sets the default to drive to the starting position. Can be overridden by
    // calling methods.
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX(), m_startPose.getX());
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY(), m_startPose.getY());
    m_rotSpeedSupplier = () -> m_rotPID.calculate(
        m_driveSubsystem.getPose().getRotation().getRadians(), m_startPose.getRotation().getRadians());
  }

  @Override
  public void initialize() {
    m_startPose = m_driveSubsystem.getPose();
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
    // TODO add logic to determine when to finish
    return false;
  }

  /**
   * Sets the X speed of the robot using a {@link DoubleSupplier}.
   * 
   * @param x {@link DoubleSupplier} that returns the X speed.
   * @return This, for method chaining.
   */
  public MoveCommand withXSpeedSupplier(DoubleSupplier x) {
    m_xSpeedSupplier = x;
    m_deltaX = 0;
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
    m_deltaY = 0;
    return this;
  }

  /**
   * Sets the rotational speed of the robot using a {@link DoubleSupplier}.
   * 
   * @param rot {@link DoubleSupplier} that returns the rotational speed.
   * @return This, for method chaining.
   */
  public MoveCommand withRotSpeedSupplier(DoubleSupplier rot) {
    m_rotSpeedSupplier = rot;
    m_deltaRot = 0;
    return this;
  }

  /**
   * TODO Driving in robot relative while moving autonomously causes errors.
   * <p>
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
   * Changes the robot relative X position to drive to based on the current position.
   * 
   * @param x Robot relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeX(double x) {
    return withRobotRelativePos(x, 0);
  }

  /**
   * Changes the robot relative Y position to drive to based on the current
   * position.
   * 
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    return withRobotRelativePos(0, y);
  }

  /**
   * Changes the robot relative position to drive to based on the current
   * position.
   * 
   * @param x Robot relative X position in meters.
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativePos(double x, double y) {
    return withFieldRelativePos(
        x * Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()) +
            y * Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()),
        x * Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()) +
            y * Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()));
  }

  /**
   * Changes the field relative X position to drive to based on the current
   * position.
   * 
   * @param x change in field relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    return withFieldRelativePos(x, 0);
  }

  /**
   * Changes the field relative Y position to drive to based on the current
   * position.
   * 
   * @param y change in field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    return withFieldRelativePos(0, y);
  }

  /**
   * Changes the field relative position to drive to based on the current
   * position.
   * 
   * @param x change in field relative X position in meters.
   * @param y change in field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativePos(double x, double y) {
    m_deltaX += x;
    m_deltaY += y;
    m_xSpeedSupplier = () -> m_xPID.calculate(m_driveSubsystem.getPose().getX(), m_startPose.getX() + m_deltaX);
    m_ySpeedSupplier = () -> m_yPID.calculate(m_driveSubsystem.getPose().getY(), m_startPose.getY() + m_deltaY);
    return this;
  }

  /**
   * Sets the absolute X position to drive to based on the current position.
   * 
   * @param x Absolute X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteX(double x) {
    return withFieldRelativeX(x - m_driveSubsystem.getPose().getX());
  }

  /**
   * Sets the absolute Y position to drive to based on the current position.
   * 
   * @param y Absolute Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteY(double y) {
    return withFieldRelativeY(y - m_driveSubsystem.getPose().getY());
  }

  /**
   * Sets the absolute position to drive to based on the current position.
   * 
   * @param x Absolute X position in meters.
   * @param y Absolute Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsolutePos(double x, double y) {
    withAbsoluteX(x);
    withAbsoluteY(y);
    return this;
  }

  /**
   * Sets the relative heading to turn to based on the current position.
   * 
   * @param rot Robot relative heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withChangeInHeading(double rot) {
    m_deltaRot += Math.toRadians(rot);
    m_rotSpeedSupplier = () -> m_rotPID.calculate(
        m_driveSubsystem.getPose().getRotation().getRadians(), m_startPose.getRotation().getRadians() + m_deltaRot);
    return this;
  }

  /**
   * Sets the absolute heading to turn to based on the current position.
   * 
   * @param rot Absolute heading in degrees.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    return withChangeInHeading(rot - m_driveSubsystem.getPose().getRotation().getDegrees());
  }
}
