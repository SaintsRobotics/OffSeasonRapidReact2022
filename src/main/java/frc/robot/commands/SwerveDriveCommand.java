// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Controls the {@link SwerveDriveSubsystem} using a 3 {@link DoubleSupplier}.
 */
public class SwerveDriveCommand extends CommandBase {
  private final SwerveDriveSubsystem m_subsystem;

  private final DoubleSupplier m_xSupplier;
  private final DoubleSupplier m_ySupplier;
  private final DoubleSupplier m_rotSupplier;

  /**
   * Creates a new {@link SwerveDriveCommand}.
   * 
   * @param subsystem    The required subsystem.
   * @param xSupplier   Supplier that returns the x speed for the robot. [-1 to
   *                     1]
   * @param ySupplier   Supplier that returns the y speed for the robot. [-1 to
   *                     1]
   * @param rotSupplier Supplier that returns the angular speed for the robot.
   *                     [-1 to 1]
   */
  public SwerveDriveCommand(SwerveDriveSubsystem subsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_xSupplier = xSupplier;
    m_ySupplier = ySupplier;
    m_rotSupplier = rotSupplier;
  }

  @Override
  public void execute() {
    double x = Utils.oddSquare(
        Utils.deadZone(m_xSupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond, 0.25)) * 0.2;
    double y = Utils.oddSquare(
        Utils.deadZone(m_ySupplier.getAsDouble() * SwerveConstants.kMaxSpeedMetersPerSecond, 0.25)) * 0.2;
    double rot = Utils.oddSquare(Utils.deadZone(
        m_rotSupplier.getAsDouble() * SwerveConstants.kMaxAngularSpeedRadiansPerSecond, 0.25)) * 0.2;

    m_subsystem.drive(x, y, rot);

    SmartDashboard.putNumber("ControllerX", x);
    SmartDashboard.putNumber("ControllerY", y);
    SmartDashboard.putNumber("ControllerRot", rot);
  }
}
