// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Utility commands */
public class Utils {
    /**
     * If the input is close enough to zero then it returns zero otherwise it
     * returns the input.
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
}
