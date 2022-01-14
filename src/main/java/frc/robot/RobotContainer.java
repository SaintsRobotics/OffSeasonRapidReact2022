// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController m_driveController = new XboxController(XboxControllerConstants.idNumber);

  private SwerveDriveSubsystem m_drivetrainSubsystem = new SwerveDriveSubsystem(
      new HardwareMap().swerveDrivetrainHardware);

  private SwerveDriveCommand m_command = new SwerveDriveCommand(m_drivetrainSubsystem, m_driveController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_drivetrainSubsystem.setDefaultCommand(m_command);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   * <p>
   * Configure buttons such that button A moves motor clockwise and button B moves
   * motor counterclockwise
   * </p>
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
