// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.OIConstants;

/**
 * Class for holding utility methods that do not apply to any specific command
 * or subsystem.
 */
public class Utils {
	// Plugging a wire into DIO port 0 enables telemetry.
	private static DigitalInput m_digitalInput = new DigitalInput(0);

	/**
	 * Makes lower inputs smaller which allows for finer joystick control.
	 * 
	 * @param input The number to apply odd square to.
	 * @return The odd squared number.
	 */
	public static double oddSquare(double input) {
		return input * Math.abs(input);
	}

	/**
	 * Returns whether the robot should print values to {@link SmartDashboard}.
	 * 
	 * @return Whether the robot should print values to {@link SmartDashboard}.
	 */
	public static boolean isTelemetryEnabled() {
		return !m_digitalInput.get() || Robot.isSimulation() || OIConstants.kTelemetry;
	}
}
