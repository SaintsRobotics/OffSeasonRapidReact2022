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

    m_xPID.setTolerance(0.1);
    m_yPID.setTolerance(0.1);
    m_rotPID.setTolerance(Math.PI / 18);

    // If a position is not set it needs to be set to the current position or it
    // will error.
    if (m_xSupplier == null) {
      m_xSupplier = () -> m_driveSubsystem.getPose().getX();
    }

    if (m_ySupplier == null) {
      m_ySupplier = () -> m_driveSubsystem.getPose().getY();
    }

    if (m_rotSupplier == null) {
      m_rotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians();
    }
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
    m_xSupplier = () -> Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()) * x;
    m_ySupplier = () -> Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()) * x;
    return this;
  }

  /**
   * Sets the robot relative Y position to drive to.
   * 
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    m_xSupplier = () -> Math.sin(m_driveSubsystem.getPose().getRotation().getRadians()) * y;
    m_ySupplier = () -> Math.cos(m_driveSubsystem.getPose().getRotation().getRadians()) * y;
    return this;
  }

  /**
   * Sets the field relative X position to drive to.
   * 
   * @param x Field relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    m_xSupplier = () -> m_driveSubsystem.getPose().getX() + x;
    return this;
  }

  /**
   * Sets the field relative Y position to drive to.
   * 
   * @param y Field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    m_ySupplier = () -> m_driveSubsystem.getPose().getY() + y;
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
   * Sets the robot relative heading to turn to.
   * 
   * @param rot Robot relative heading in radians.
   * @return This, for method chaining.
   */
  public MoveCommand withRelativeHeading(double rot) {
    m_rotSupplier = () -> m_driveSubsystem.getPose().getRotation().getRadians() + rot;
    return this;
  }

  /**
   * Sets the absolute heading to turn to.
   * 
   * @param rot Absolute heading in radians.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    m_rotSupplier = () -> rot;
    return this;
  }
}
