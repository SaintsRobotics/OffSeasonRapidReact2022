/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Class for holding utility methods that do not apply to any specific command
 * or subsystem.
 */
public class Utils {

	/**
	 * If the input is close enough to zero then treat it as zero.
	 * 
	 * @param input    The input to apply a dead zone to.
	 * @param deadZone The absolute range to apply the deadzone.
	 * @return The dead zoned value.
	 */
	public static double deadZone(double input, double deadZone) {
		if (Math.abs(input) < deadZone) {
			return 0;
		}
		return input;
	}

  /**
   * If input is within tolerance of desiredVal, it returns desiredVal. If
   * input is outside the tolerance, then it returns input.
   * 
   * @param input      The input number.
   * @param desiredVal The desired value.
   * @param tolerance  The tolerance.
   */
  public static double tolerance(double input, double desiredVal, double tolerance) {
    if (Math.abs(input - desiredVal) < tolerance) {
      return desiredVal;
    }
    return input;
  }

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
   * Returns an angle between 0 and max if the angle exceeds normal bounds
   * 
   * @param angle The angle to use.
   * @param max   The maximum possible value.
   * @return The angle with range of 0 to max.
   */
  public static double normalizeAngle(double angle, double max) {
    return ((angle % max) + max) % max;
  }
}
