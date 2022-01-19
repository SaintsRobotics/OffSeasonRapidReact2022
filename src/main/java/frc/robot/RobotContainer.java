// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
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
  private SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      new HardwareMap().swerveDrivetrainHardware);

  private XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    DoubleSupplier x = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getLeftY(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxSpeedMetersPerSecond * 0.2;
    DoubleSupplier y = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getLeftX(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxSpeedMetersPerSecond * 0.2;
    DoubleSupplier rot = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getRightX(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxAngularSpeedRadiansPerSecond * 0.2;
    m_swerveDriveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(m_swerveDriveSubsystem, x, y, rot, () -> m_driveController.getRightBumper()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Resets the odometry when the back button is pressed.
    new JoystickButton(m_driveController, Button.kBack.value)
        .whenPressed(() -> m_swerveDriveSubsystem.resetOdometry(new Pose2d()), m_swerveDriveSubsystem);

    // Zeroes the heading when the start button is pressed
    new JoystickButton(m_driveController, Button.kStart.value).whenPressed(() -> m_swerveDriveSubsystem.zeroHeading(),
        m_swerveDriveSubsystem);
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
