// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.LimelightAimingCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();

	private final MoveCommand m_defaultMoveCommand;
	private final MoveCommand m_aimingMoveCommand;

	private XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DoubleSupplier x = () -> Utils
				.oddSquare(MathUtil.applyDeadband(-m_driveController.getLeftY(),
						OIConstants.kControllerDeadband))
				* SwerveConstants.kMaxSpeedMetersPerSecond;
		DoubleSupplier y = () -> Utils
				.oddSquare(MathUtil.applyDeadband(-m_driveController.getLeftX(),
						OIConstants.kControllerDeadband))
				* SwerveConstants.kMaxSpeedMetersPerSecond;
		DoubleSupplier rot = () -> Utils
				.oddSquare(MathUtil.applyDeadband(-m_driveController.getRightX(),
						OIConstants.kControllerDeadband))
				* SwerveConstants.kMaxAngularSpeedRadiansPerSecond;
		BooleanSupplier fieldRelative = () -> m_driveController.getRightBumper();
		m_defaultMoveCommand = new MoveCommand(m_swerveDriveSubsystem)
				.withXSpeedSupplier(x)
				.withYSpeedSupplier(y)
				.withRotSpeedSupplier(rot)
				.withFieldRelativeSupplier(fieldRelative);
		m_aimingMoveCommand = new MoveCommand(m_swerveDriveSubsystem)
				.withXSpeedSupplier(x)
				.withYSpeedSupplier(y)
				.withFieldRelativeSupplier(fieldRelative);

		configureButtonBindings();
		Limelight.setLED(1);
		Limelight.setCameraMode(1);

		m_swerveDriveSubsystem.setDefaultCommand(m_defaultMoveCommand);
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
				.whenPressed(() -> m_swerveDriveSubsystem.resetOdometry(new Pose2d()),
						m_swerveDriveSubsystem);

		// Zeroes the heading when the start button is pressed
		new JoystickButton(m_driveController, Button.kStart.value)
				.whenPressed(() -> m_swerveDriveSubsystem.zeroHeading(), m_swerveDriveSubsystem);

		// Aims at target while the A button is held.
		new JoystickButton(m_driveController, Button.kA.value)
				.whenHeld(new LimelightAimingCommand(m_aimingMoveCommand, 0));

		// Aims at the color of ball that matches the alliance color while the B button
		// is held.
		new JoystickButton(m_driveController, Button.kB.value)
				.whenHeld(new LimelightAimingCommand(m_aimingMoveCommand,
						DriverStation.getAlliance() == Alliance.Blue ? 1 : 2));
		// Toggles the shooter when Y button is pressed.
		new JoystickButton(m_driveController, Button.kY.value)
				.toggleWhenPressed(new ShooterCommand(new ShooterSubsystem()));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		/*
		 * The plan for two ball autonomous mode:
		 * Starting Position: facing towards the ball, at the corner parallel to the
		 * line
		 * 1. Go to ball (drive forwards 1 meter)
		 * 2. Run intake for 1 second
		 * 2. Turn around and get into position for shooting the ball (drive field
		 * relative backwards 3 meters, turn 180-ish degrees)
		 * 3. Shoot the ball
		 */
		// TODO ADD INTAKE COMMAND AND SHOOT COMMAND TO THE SEQUENCE
		return new SequentialCommandGroup(new MoveCommand(m_swerveDriveSubsystem).withRobotRelativeX(1),
				new MoveCommand(m_swerveDriveSubsystem).withChangeInHeading(180)
						.withFieldRelativeX(-3));
	}
}