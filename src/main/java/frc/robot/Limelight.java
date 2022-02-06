// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Class to access the limelight through network tables. */
public final class Limelight {

    /**
     * Vertical Offset From Crosshair To Target
     * 
     * @return vertical offset
     */
    public static double getY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    /**
     * Horizontal Offset From Crosshair To Target
     * 
     * @return horizontal offset
     */
    public static double getX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    /**
     * Skew or rotation (-90 degrees to 0 degrees)
     * 
     * @return skew angle
     */
    public static double getAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }

    /**
     * The pipeline’s latency contribution (ms)
     * 
     * @return latency value
     */
    public static double getLatency() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }

    /**
     * Target Area (0% of image to 100% of image)
     * 
     * @return area value
     */
    public static double getArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    /**
     * Whether the limelight has any valid targets (0 or 1)
     * 
     * @return true if the camera sees target
     */
    public static boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 0 ? false : true;
    }

    /**
     * Sets limelight’s LED state.
     * <p>
     * 0: use LED Mode set in the current pipeline.
     * <p>
     * 1: force off.
     * <p>
     * 2: force blink.
     * <p>
     * 3: force on.
     * 
     * @param state Limelight LED state.
     */
    public static void setLed(double state) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state);
    }

    /**
     * Sets limelight’s current pipeline.
     * 
     * @param pipeline Select pipeline 0-9.
     */
    public static void setPipeline(double pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
}
