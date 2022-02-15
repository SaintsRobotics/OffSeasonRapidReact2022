package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder {
	private final AnalogInput m_analogIn;

	private final boolean m_reversed;
	private final double m_offset;

	/**
	 * Construct an absolute encoder, most likely a US Digital MA3 encoder.
	 * 
	 * @param channel  analog in (aka sometime also refered to as AIO) port on the
	 *                 robotRIO
	 * @param reversed set this to <i>TRUE</i> if physically turning the swerve
	 *                 wheel <i>CLOCKWISE</i> (looking down from the top of the bot)
	 *                 <i>INCREASES</i> the raw voltage that the encoder provides.
	 * @param offset   swerve offset in <i>RADIANS</i>. This value is
	 *                 <i>SUBTRACTED</i> from the encoder output.
	 */
	public AbsoluteEncoder(int channel, boolean reversed, double offset) {
		m_analogIn = new AnalogInput(channel);
		m_reversed = reversed;
		m_offset = offset;
	}

	/**
	 * Returns the angle as a {@link Rotation2d}. Zero points toward the front of
	 * the robot.
	 * <i>The value INCREASES as the wheel is turned COUNTER-CLOCKWISE</i>
	 * 
	 * @return The angle as a {@link Rotation2d}.
	 */
	public Rotation2d get() {
		double angle = (m_analogIn.getVoltage() / 5 * 2 * Math.PI) - m_offset;

		return m_reversed ? new Rotation2d(5 - angle) : new Rotation2d(angle);
	}
}
