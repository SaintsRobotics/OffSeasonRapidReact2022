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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MUX;
import frc.robot.REVColorSensorV3;
import frc.robot.Robot;
import frc.robot.Utils;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmMotorPort, MotorType.kBrushless);
	private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ShooterConstants.kArmEncoderPort);

	private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakePort, MotorType.kBrushless);

	private final CANSparkMax m_leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
	private final CANSparkMax m_rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders = new MotorControllerGroup(m_leftFeeder, m_rightFeeder);

	private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);

	private final WPI_TalonFX m_bottomFlywheel = new WPI_TalonFX(ShooterConstants.kBottomFlywheelPort);
	private final WPI_TalonFX m_topFlywheel = new WPI_TalonFX(ShooterConstants.kTopFlywheelPort);

	private final MUX m_MUX = new MUX();
	private final REVColorSensorV3 m_queueColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kQueueColorSensorPort);
	private final REVColorSensorV3 m_shooterColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kShooterColorSensorPort);

	private final PIDController m_armPID = new PIDController(0.008, 0, 0);
	private PIDController m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterPTarmac, 0, 0);
	private PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopShooterPTarmac, 0, 0);
	private SimpleMotorFeedforward m_bottomFeedforward = new SimpleMotorFeedforward(
			ShooterConstants.kBottomFeedforwardTarmac, 0);
	private SimpleMotorFeedforward m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardTarmac,
			0);

	private boolean m_runningIntake = false;
	private boolean m_reversingIntake = false;
	private Timer m_feederTimer = new Timer();

	public static enum Mode {
		kFender,
		kTarmac,
		kEnd;
	}

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		// TODO change to getAngle if WPILib adds it
		m_armEncoder.setDistancePerRotation(360);

		m_arm.setIdleMode(IdleMode.kBrake);
		m_bottomFlywheel.setNeutralMode(NeutralMode.Coast);
		m_topFlywheel.setNeutralMode(NeutralMode.Coast);

		m_arm.setInverted(ShooterConstants.kArmReversed);
		m_intake.setInverted(ShooterConstants.kIntakeReversed);
		m_leftFeeder.setInverted(ShooterConstants.kLeftFeederReversed);
		m_rightFeeder.setInverted(ShooterConstants.kRightFeederReversed);
		m_topFeeder.setInverted(ShooterConstants.kTopFeederReversed);
		m_bottomFlywheel.setInverted(ShooterConstants.kBottomFlywheelReversed);
		m_topFlywheel.setInverted(ShooterConstants.kTopFlywheelReversed);

		m_bottomShooterPID.setTolerance(0.08 * ShooterConstants.kBottomMotorRPMTarmac, 100 / 0.02);
		m_topShooterPID.setTolerance(0.08 * ShooterConstants.kTopMotorRPMTarmac, 100 / 0.02);
		m_armPID.setTolerance(2);
		m_armPID.enableContinuousInput(-180, 180);
	}

	@Override
	public void periodic() {
		double bottomPIDOutput = m_bottomShooterPID
				.calculate(toRPM(m_bottomFlywheel.getSelectedSensorVelocity()));
		double topPIDOutput = m_topShooterPID.calculate(toRPM(m_topFlywheel.getSelectedSensorVelocity()));
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
			if (m_bottomShooterPID.atSetpoint() && m_topShooterPID.atSetpoint()) {
				m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			}

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

		if (Robot.isReal()) {
			SmartDashboard.putNumber("Temperature Arm", m_arm.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Intake", m_intake.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Left Feeder", m_leftFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Right Feeder", m_rightFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Top Feeder", m_topFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Bottom Flywheel", m_topFlywheel.getTemperature());
			SmartDashboard.putNumber("Temperature Top Flywheel", m_bottomFlywheel.getTemperature());

			SmartDashboard.putNumber("Current Arm", m_arm.getOutputCurrent());
			SmartDashboard.putNumber("Current Intake", m_intake.getOutputCurrent());
			SmartDashboard.putNumber("Current Left Feeder", m_leftFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Right Feeder", m_rightFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Top Feeder", m_topFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Bottom Flywheel", m_topFlywheel.getStatorCurrent());
			SmartDashboard.putNumber("Current Top Flywheel", m_bottomFlywheel.getStatorCurrent());
		}

		if (Utils.isTelemetryEnabled()) {
			SmartDashboard.putBoolean("Bottom Shooter At Setpoint", m_bottomShooterPID.atSetpoint());
			SmartDashboard.putBoolean("Top Shooter At Setpoint", m_topShooterPID.atSetpoint());
			SmartDashboard.putNumber("Bottom Target RPM", m_bottomShooterPID.getSetpoint());
			SmartDashboard.putNumber("Top Target RPM", m_topShooterPID.getSetpoint());
			SmartDashboard.putBoolean("Is shooter primed", isShooterPrimed());
			SmartDashboard.putNumber("Shooter proximity", m_shooterColorSensor.getProximity());
			SmartDashboard.putNumber("Bottom Shooter RPM", toRPM(m_bottomFlywheel.getSelectedSensorVelocity()));
			SmartDashboard.putNumber("Top Shooter RPM", toRPM(m_topFlywheel.getSelectedSensorVelocity()));
			SmartDashboard.putNumber("Top Feeder Speed", m_topFeeder.get());
			SmartDashboard.putNumber("Top feedforward", m_topFeedforward.calculate(ShooterConstants.kTopFeedforwardTarmac));
			SmartDashboard.putBoolean("Top at Setpoint", m_topShooterPID.atSetpoint());
		}

	}

	/** Raises the arm. */
	public void raiseArm() {
		m_armPID.setSetpoint(ShooterConstants.kUpperArmAngle);
		if (m_armPID.atSetpoint()) {
			m_arm.set(0);
		} else {
				m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), -0.3, -0.1));
		}
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(ShooterConstants.kLowerArmAngle);
		if (m_armPID.atSetpoint()) {
			m_arm.set(0);
		} else {
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), 0.1, 0.5)); 
		}
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
	 * Sets the speed of the shooter.
	 * 
	 * @param bottomRPM Target RPM for the bottom flywheel.
	 * @param topRPM    Target RPM for the top flywheel.
	 */
	public void setShooterSpeed(Mode mode) {
		double bottomRPM = 0;
		double topRPM = 0;
		if (mode == Mode.kFender) {
			bottomRPM = ShooterConstants.kBottomMotorRPMFender;
			topRPM = ShooterConstants.kTopMotorRPMFender;
			m_bottomFeedforward = new SimpleMotorFeedforward(ShooterConstants.kBottomFeedforwardFender, 0);
			m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardFender, 0);
			m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterPFender, 0, 0);
			m_topShooterPID = new PIDController(ShooterConstants.kTopShooterPFender, 0, 0);
		} else if (mode == Mode.kTarmac) {
			bottomRPM = ShooterConstants.kBottomMotorRPMTarmac;
			topRPM = ShooterConstants.kTopMotorRPMTarmac;
			m_bottomFeedforward = new SimpleMotorFeedforward(ShooterConstants.kBottomFeedforwardTarmac, 0);
			m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardTarmac, 0);
			m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterPTarmac, 0, 0);
			m_topShooterPID = new PIDController(ShooterConstants.kTopShooterPTarmac, 0, 0);
		}

		m_bottomShooterPID.setSetpoint(bottomRPM);
		m_topShooterPID.setSetpoint(topRPM);
		m_bottomShooterPID.setTolerance(0.08 * bottomRPM, 100 / 0.02);
		m_topShooterPID.setTolerance(0.08 * topRPM, 100 / 0.02);

		if (bottomRPM == 0) {
			m_sideFeeders.set(0);
		} else {
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
	}

	public void setShooterSpeed (Mode approximateMode, double bottomRPM, double topRPM) {
		setShooterSpeed(approximateMode);
		m_bottomShooterPID.setSetpoint(bottomRPM);
		m_topShooterPID.setSetpoint(topRPM);
		m_bottomShooterPID.setTolerance(0.08 * bottomRPM, 100 / 0.02);
		m_topShooterPID.setTolerance(0.08 * topRPM, 100 / 0.02);
	}

	private boolean isShooterPrimed() {
		return m_shooterColorSensor.getProximity() >= 140;
	}

	public void topFeederOn() {
		m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
	}

	public void topFeederOff() {
		m_topFeeder.set(0);
	}

	/**
	 * Converts the speed of a TalonFX from the default units of ticks per
	 * decisecond to RPM.
	 * 
	 * @param ticksPerDecisecond The speed in ticks per decisecond.
	 * @return The speed in RPM.
	 */
	private double toRPM(double ticksPerDecisecond) {
		return ticksPerDecisecond * 600 / 2048;
	}
}
