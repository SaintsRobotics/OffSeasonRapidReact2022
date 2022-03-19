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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimelightAimingCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.PathWeaverCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
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
	private final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();
	private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

	private final MoveCommand m_defaultMoveCommand;
	private final MoveCommand m_aimingMoveCommand;

	private final XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);
	private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

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

		final DoubleSupplier leftClimbSpeed = () -> -Utils
				.oddSquare(MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kControllerDeadband))
				* 0.5;
		final DoubleSupplier rightClimbSpeed = () -> -Utils
				.oddSquare(MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kControllerDeadband))
				* 0.5;

		// Allows for independent control of climbers when enabled in test mode.
		// Otherwise climbers are controlled together.
		m_climberSubsystem.setDefaultCommand(new RunCommand(() -> {
			if (DriverStation.isTest()) {
				m_climberSubsystem.set(leftClimbSpeed.getAsDouble(), rightClimbSpeed.getAsDouble());
			} else {
				m_climberSubsystem.set(leftClimbSpeed.getAsDouble());
			}
		}, m_climberSubsystem));

		SmartDashboard.putString("Autonomous Path", "BlueHangar");
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

		// Allows the bot to drift while left bumper is held
		new JoystickButton(m_driveController, Button.kLeftBumper.value)
				.whileHeld(() -> m_swerveDriveSubsystem.setMotorIdle())
				.whenReleased(() -> m_swerveDriveSubsystem.setMotorBrake());

		// Slowly drives forward while X is held.
		new JoystickButton(m_driveController, Button.kX.value)
				.whileHeld(() -> m_swerveDriveSubsystem.drive(0.3, 0, 0, false), m_swerveDriveSubsystem)
				.whenReleased(() -> m_swerveDriveSubsystem.drive(0, 0, 0, false), m_swerveDriveSubsystem);

		// raises the arm while left bumper held
		new JoystickButton(m_operatorController, Button.kLeftBumper.value)
				.whileHeld(() -> m_shooterSubsystem.raiseArm())
				.whenReleased(() -> m_shooterSubsystem.stopArm());

		// lowers arm while right bumper held
		new JoystickButton(m_operatorController, Button.kRightBumper.value)
				.whileHeld(() -> m_shooterSubsystem.lowerArm())
				.whenReleased(() -> m_shooterSubsystem.stopArm());

		// Turns on shooter when Y button is held.
		new JoystickButton(m_operatorController, Button.kY.value)
				.whenHeld(new ShooterCommand(m_shooterSubsystem));

		// runs intake forward while left trigger is held
		new Trigger(() -> m_operatorController.getRawAxis(Axis.kLeftTrigger.value) > 0.5)
				.whenActive(new InstantCommand(() -> m_shooterSubsystem.intake()))
				.whenInactive(new InstantCommand(() -> m_shooterSubsystem.intakeOff()));

		// runs intake backwards while right trigger is held
		new Trigger(() -> m_operatorController.getRawAxis(Axis.kRightTrigger.value) > 0.5)
				.whenActive(new InstantCommand(() -> m_shooterSubsystem.intakeReverse()))
				.whenInactive(new InstantCommand(() -> m_shooterSubsystem.intakeOff()));

		new JoystickButton(m_operatorController, Button.kA.value)
				.whileHeld(() -> m_shooterSubsystem.topFeederOn())
				.whenReleased(() -> m_shooterSubsystem.topFeederOff());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * 
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Two ball autonomous routine.
		return new SequentialCommandGroup(
				new ArmCommand(m_shooterSubsystem, ShooterConstants.kLowerArmAngle),
				new ParallelCommandGroup(
						new PathWeaverCommand(m_swerveDriveSubsystem,
								SmartDashboard.getString("Autonomous Path", "BlueHangar") + "TwoBall1", true),
						new IntakeCommand(m_shooterSubsystem)),
				new ArmCommand(m_shooterSubsystem, ShooterConstants.kUpperArmAngle),
				new PathWeaverCommand(m_swerveDriveSubsystem,
						SmartDashboard.getString("Autonomous Path", "BlueHangar") + "TwoBall2", false),
				new ShootCommand(m_shooterSubsystem));
	}
}
