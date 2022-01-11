// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AbsoluteEncoder;
import frc.robot.HardwareMap.SwerveModuleHardware;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
    private AbsoluteEncoder m_absoluteEncoder;
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor;
    private PIDController m_pidController;
    private Translation2d m_location;

    /**
     * Creates a new {@link SwerveModule}.
     * 
     * @param hardware     the hardware for the swerve module
     * @param driveMotor   motor that drives the wheel
     * @param turningMotor motor that changes the angle of the wheel
     * @param x            x position of the wheel on the robot
     * @param y            y position of the wheel on the robot
     * @param absEncoder   absolute encoder for the swerve module
     */
    public SwerveModule(SwerveModuleHardware hardware, CANSparkMax driveMotor, CANSparkMax turningMotor, double x,
            double y, AbsoluteEncoder absEncoder) {
        m_absoluteEncoder = absEncoder;
        m_pidController = new PIDController(0, 0, 0);
        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        m_location = new Translation2d(x, y);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turningMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setState(SwerveModuleState desiredState) {
        m_turningMotor
                .set(m_pidController.calculate(m_absoluteEncoder.getAngleRadians(), desiredState.angle.getRadians()));
        m_driveMotor.set(desiredState.speedMetersPerSecond);
    }

    /**
     * Returns the location of the wheel on the robot.
     * 
     * @return the location of the wheel on the robot
     */
    public Translation2d getLocation() {
        return m_location;
    }

    /**
     * Returns the current radian value from the encoder.
     * 
     * @return the current radian value from the encoder
     */
    public double getRadians() {
        return m_absoluteEncoder.getAngleRadians();
    }
}
