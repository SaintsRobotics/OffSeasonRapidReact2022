// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Controls the {@link SwerveDriveSubsystem} using an {@link XboxController}. */
public class JoystickControllerCommand extends CommandBase {
    private final SwerveDriveSubsystem m_subsystem;
    private final XboxController m_xboxController;

    /**
     * Creates a new {@link JoystickControllerCommand}.
     * 
     * @param subsystem the required subsystem
     * @param xboxController xbox controller for driving the robot
     */
    public JoystickControllerCommand(SwerveDriveSubsystem subsystem, XboxController xboxController) {
        m_subsystem = subsystem;
        m_xboxController = xboxController;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double y = Utils.deadZone(-m_xboxController.getLeftX() * Constants.SwerveConstants.MAX_DRIVE_SPEED_MPS, 0.25);
        double x = Utils.deadZone(m_xboxController.getLeftY() * Constants.SwerveConstants.MAX_DRIVE_SPEED_MPS, 0.25);
        double rotation = Utils.deadZone(-m_xboxController.getRightX() * Constants.SwerveConstants.MAX_TURNING_SPEED_RADIANS_PER_SECOND, 0.25);
        m_subsystem.drive(x, y, rotation);

        SmartDashboard.putNumber("ControllerX", x);
        SmartDashboard.putNumber("ControllerY", y);
        SmartDashboard.putNumber("ControllerRot", rotation);
    }
}
