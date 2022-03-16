// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MUX;
import frc.robot.REVColorSensorV3;
import frc.robot.Utils;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmPort, MotorType.kBrushless);
	private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(9);

	private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakeWheelsPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders;
	private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);

	private final WPI_TalonFX m_bottomFlywheel = new WPI_TalonFX(ShooterConstants.kBottomFlywheelPort);
	private final WPI_TalonFX m_topFlywheel = new WPI_TalonFX(ShooterConstants.kTopFlywheelPort);

	private final MUX m_MUX = new MUX();
	private final REVColorSensorV3 m_queueColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kQueueColorSensorPort);
	private final REVColorSensorV3 m_shooterColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kShooterColorSensorPort);

	private final PIDController m_armPID = new PIDController(0.005, 0, 0);
	private final PIDController m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterP, 0, 0);
	private final PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopShooterP, 0, 0); // TODO: THIS
																											// PID NEEDS
																											// TUNING
	private final SimpleMotorFeedforward m_bottomFeedforward = new SimpleMotorFeedforward(0.35, 0);
	private final SimpleMotorFeedforward m_topFeedforward = new SimpleMotorFeedforward(0.8, 0);

	private boolean m_runningIntake = false;
	private boolean m_reversingIntake = false;
	private Timer m_feederTimer = new Timer();

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_arm.setIdleMode(IdleMode.kBrake);
		m_arm.setInverted(true);

		// TODO change to getAngle if WPILib adds it
		m_armEncoder.setDistancePerRotation(360);

    m_bottomFlywheel.setNeutralMode(NeutralMode.Coast);
		m_topFlywheel.setNeutralMode(NeutralMode.Coast);

    CANSparkMax leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
		CANSparkMax rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
		m_intake.setInverted(true);
		leftFeeder.setInverted(true);
		rightFeeder.setInverted(false);
		m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
		m_bottomShooterPID.setTolerance(0.05 * ShooterConstants.kTopShooterSpeedRPM, 100 / 0.02);
		m_topShooterPID.setTolerance(0.05 * ShooterConstants.kBottomShooterSpeedRPM, 100 / 0.02);
		m_armPID.setTolerance(2);

		m_feederTimer.start();
	}

	// top feeder run for how long?
	@Override
	public void periodic() {
		double bottomPIDOutput = m_bottomShooterPID
				.calculate(Utils.toRPM(m_bottomFlywheel.getSelectedSensorVelocity()));
		double topPIDOutput = m_topShooterPID.calculate(Utils.toRPM(m_topFlywheel.getSelectedSensorVelocity()));
		final boolean queueIsBlue = m_queueColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean queueIsRed = m_queueColorSensor.getRed() > ShooterConstants.kRedThreshold;
		final boolean shooterIsBlue = m_shooterColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean shooterIsRed = m_shooterColorSensor.getRed() > ShooterConstants.kRedThreshold;

		// Checks if the color of ball is opposite that of the alliance.
		if ((queueIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(queueIsRed && DriverStation.getAlliance() == Alliance.Blue) ||
				(shooterIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(shooterIsRed && DriverStation.getAlliance() == Alliance.Blue)) {
			// TODO also run the feeders in reverse
			intakeReverse();
		}

		if (m_bottomShooterPID.getSetpoint() > 0) {
			m_bottomFlywheel.set(bottomPIDOutput + m_bottomFeedforward.calculate(m_bottomShooterPID.getSetpoint()));
			m_topFlywheel.set(topPIDOutput + m_topFeedforward.calculate(m_topShooterPID.getSetpoint()));
		} else {
			m_bottomFlywheel.set(0);
			m_topFlywheel.set(0);
		}

		if (m_feederTimer.get() > 0) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			if (m_feederTimer.get() > 3) {
				m_feederTimer.stop();
				m_feederTimer.reset();
			}
		} else if (m_bottomShooterPID.atSetpoint() && m_topShooterPID.atSetpoint() && isShooterPrimed()
				&& m_bottomShooterPID.getSetpoint() > 0) {
			m_feederTimer.start();
		} else if (!isShooterPrimed() && m_runningIntake) { // if we're trying to intake - prime the first ball
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedSlow);
		} else if (!m_reversingIntake) { // as long as we're not trying to spit out the wrong color, set to zero
			m_topFeeder.set(0);
		}

		SmartDashboard.putNumber("Arm Encoder", m_armEncoder.getAbsolutePosition());

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Bottom Shooter PID Output", bottomPIDOutput);
			SmartDashboard.putNumber("Top Shooter PID Output", topPIDOutput);

			SmartDashboard.putNumber("Bottom Shooter Feedforward Output",
					m_bottomFeedforward.calculate(m_bottomShooterPID.getSetpoint()));
			SmartDashboard.putNumber("Top Shooter Feedforward Output",
					m_topFeedforward.calculate(m_topShooterPID.getSetpoint()));

			SmartDashboard.putBoolean("top at setpoint", m_topShooterPID.atSetpoint());
			SmartDashboard.putBoolean("bottom at setpoint", m_bottomShooterPID.atSetpoint());

			SmartDashboard.putNumber("Bottom Shooter Power", m_bottomFlywheel.get());
			SmartDashboard.putNumber("Top Shooter Power", m_topFlywheel.get());

			SmartDashboard.putNumber("Bottom Shooter RPM", Utils.toRPM(m_bottomFlywheel.getSelectedSensorVelocity()));
			SmartDashboard.putNumber("Top Shooter RPM", Utils.toRPM(m_topFlywheel.getSelectedSensorVelocity()));

			SmartDashboard.putNumber("Bottom Shooter RPM Error", m_bottomShooterPID.getPositionError());
			SmartDashboard.putNumber("Top Shooter RPM Error", m_topShooterPID.getPositionError());

			SmartDashboard.putNumber("Arm Encoder", m_armEncoder.getAbsolutePosition());

			SmartDashboard.putNumber("Side Feeder Speed", m_sideFeeders.get());
			SmartDashboard.putNumber("Top Feeder Speed", m_topFeeder.get());
			SmartDashboard.putNumber("Intake Wheel Speed", m_intake.get());
			SmartDashboard.putNumber("Arm Motor Speed", m_arm.get());
			SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());

			SmartDashboard.putNumber("Queue Proximity", m_queueColorSensor.getProximity());
			SmartDashboard.putBoolean("Queue Is Blue", queueIsBlue);
			SmartDashboard.putBoolean("Queue Is Red", queueIsRed);

			SmartDashboard.putNumber("Shooter Proximity", m_shooterColorSensor.getProximity());
			SmartDashboard.putBoolean("Shooter Is Blue", shooterIsBlue);
			SmartDashboard.putBoolean("Shooter Is Red", shooterIsRed);

			SmartDashboard.putBoolean("is shooter primed", isShooterPrimed());
		}
	}

	/** Raises the arm. */
	public void raiseArm() {
		m_armPID.setSetpoint(ShooterConstants.kUpperArmAngle);
		if (m_armPID.atSetpoint())
			m_arm.set(0);
		else
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), -0.25, -0.1));
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(ShooterConstants.kLowerArmAngle);
		if (m_armPID.atSetpoint())
			m_arm.set(0);
		else
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), 0.1, 0.25));
	}

	public void stopArm() {
		m_arm.set(0);
	}

	/** Runs the intake. */
	public void intake() {
		// if there is a ball at the top: don't run top feeder
		// run the intake wheels and side feeders
		m_runningIntake = true;
		m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		m_intake.set(ShooterConstants.kIntakeSpeed);
	}

	/** Runs the intake in reverse. */
	public void intakeReverse() {
		m_reversingIntake = true;
		m_intake.set(-ShooterConstants.kIntakeSpeed);
		m_sideFeeders.set(-ShooterConstants.kSideFeederSpeed);
		m_topFeeder.set(-ShooterConstants.kTopFeederSpeedFast);
	}

	/** Turns off the intake. */
	public void intakeOff() {
		m_runningIntake = false;
		m_reversingIntake = false;
		m_intake.set(0);
		m_sideFeeders.set(0);
	}

	/**
	 * Shoots the ball(s)
	 * 
	 * @param speed Speed of the black flywheel in RPM.
	 */
	public void setShooterSpeeds(double bottomTargetRPM, double topTargetRPM) {
		m_bottomShooterPID.setSetpoint(bottomTargetRPM);
		m_topShooterPID.setSetpoint(topTargetRPM);
		if (bottomTargetRPM == 0) {
			m_sideFeeders.set(0);
		} else {
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Bottom Target Shooter Speed", bottomTargetRPM);
			SmartDashboard.putNumber("Top Target Shooter Speed", topTargetRPM);

		}
	}

	/**
	 * Shoots the ball(s)
	 * 
	 * @param speed Speed of the roller in RPM.
	 */

	private boolean isShooterPrimed() {
		return m_queueColorSensor.getProximity() >= 180;
	}

	public void topFeederOn() {
		m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
	}

	public void topFeederOff() {
		m_topFeeder.set(0);
	}

}
