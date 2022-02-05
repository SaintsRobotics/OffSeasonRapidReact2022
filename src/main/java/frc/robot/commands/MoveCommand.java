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
  private final SwerveDriveSubsystem m_swerveSubsystem;

  private final PIDController xPID = new PIDController(0.3, 0, 0);
  private final PIDController yPID = new PIDController(0.3, 0, 0);
  private final PIDController rotPID = new PIDController(0.3, 0, 0);

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier rotSupplier;

  /** Creates a new {@link MoveCommand}. */
  public MoveCommand(SwerveDriveSubsystem subsystem) {
    m_swerveSubsystem = subsystem;
    addRequirements(m_swerveSubsystem);

    xPID.setTolerance(0.1);
    yPID.setTolerance(0.1);
    rotPID.setTolerance(Math.PI / 18);

    // If a position is not set it needs to be set to the current position or it will error.
    if(xSupplier == null) {
      xSupplier = () -> m_swerveSubsystem.getPose().getX();
    }

    if(ySupplier == null) {
      ySupplier = () -> m_swerveSubsystem.getPose().getY();
    }

    if(rotSupplier == null) {
      rotSupplier = () -> m_swerveSubsystem.getPose().getRotation().getRadians();
    }
  }

  @Override
  public void initialize() {
    xPID.setSetpoint(xSupplier.getAsDouble());
    yPID.setSetpoint(ySupplier.getAsDouble());
    rotPID.setSetpoint(rotSupplier.getAsDouble());
  }

  @Override
  public void execute() {
    m_swerveSubsystem.drive(
        xPID.calculate(m_swerveSubsystem.getPose().getX()),
        yPID.calculate(m_swerveSubsystem.getPose().getY()),
        rotPID.calculate(m_swerveSubsystem.getPose().getRotation().getRadians()),
        true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }

  /**
   * Sets the robot relative X position to drive to.
   * 
   * @param x Robot relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeX(double x) {
    xSupplier = () -> Math.cos(m_swerveSubsystem.getPose().getRotation().getRadians()) * x;
    ySupplier = () -> Math.sin(m_swerveSubsystem.getPose().getRotation().getRadians()) * x;
    return this;
  }

  /**
   * Sets the robot relative Y position to drive to.
   * 
   * @param y Robot relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withRobotRelativeY(double y) {
    xSupplier = () -> Math.sin(m_swerveSubsystem.getPose().getRotation().getRadians()) * y;
    ySupplier = () -> Math.cos(m_swerveSubsystem.getPose().getRotation().getRadians()) * y;
    return this;
  }

  /**
   * Sets the field relative X position to drive to.
   * 
   * @param x Field relative X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeX(double x) {
    xSupplier = () -> x + m_swerveSubsystem.getPose().getX();
    return this;
  }

  /**
   * Sets the field relative Y position to drive to.
   * 
   * @param y Field relative Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withFieldRelativeY(double y) {
    ySupplier = () -> y + m_swerveSubsystem.getPose().getY();
    return this;
  }

  /**
   * Sets the absolute X position to drive to.
   * 
   * @param y Absolute X position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteX(double x) {
    xSupplier = () -> x;
    return this;
  }

  /**
   * Sets the absolute Y position to drive to.
   * 
   * @param y Absolute Y position in meters.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteY(double y) {
    ySupplier = () -> y;
    return this;
  }

  /**
   * Sets the robot relative heading to turn to.
   * 
   * @param rot Robot relative heading in radians.
   * @return This, for method chaining.
   */
  public MoveCommand withRelativeHeading(double rot) {
    rotSupplier = () -> m_swerveSubsystem.getPose().getRotation().getRadians() + rot;
    return this;
  }

  /**
   * Sets the absolute heading to turn to.
   * 
   * @param rot Absolute heading in radians.
   * @return This, for method chaining.
   */
  public MoveCommand withAbsoluteHeading(double rot) {
    rotSupplier = () -> rot;
    return this;
  }
}
