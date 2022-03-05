// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants.ConversionConstants;

public class DutyCycleAbsoluteEncoder {
    private final DutyCycle m_dutyCycle;

    public DutyCycleAbsoluteEncoder(int channel) {
        m_dutyCycle = new DutyCycle(new DigitalInput(channel));

    }

    /**
     * Get the absolute position of the duty cycle encoder.
     *
     * <p>
     * getAbsolutePosition() - getPositionOffset() will give an encoder absolute
     * position relative
     * to the last reset.
     *
     * <p>
     * This will not account for rollovers, and will always be just the raw absolute
     * position.
     * 
     * <p>
     * Ranges from 0 degrees to 360 degrees
     *
     * @return the absolute position
     */
    public double getAbsolutePosition() {
        return m_dutyCycle.getOutput() * ConversionConstants.kDegreesInARotation;
    }

}
