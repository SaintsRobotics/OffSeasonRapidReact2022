package frc.robot;

import edu.wpi.first.math.MathUtil;
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
	 * @param offset   Offset of the analog input in volts. Set this to the voltage
	 *                 the analog input returns when the wheel is pointed forward.
	 */
	public AbsoluteEncoder(int channel, boolean reversed, double offset) {
		m_analogIn = new AnalogInput(channel);
		m_reversed = reversed;
		m_offset = offset;
	}

	/**
	 * Returns the angle of the encoder. Zero points toward the front of the robot.
	 * The value increases as the wheel is turned counterclockwise.
	 * 
	 * @return The angle between -pi and pi.
	 */
	public double get() {
		// Takes the voltage of the analog input (0 to 5) and converts it to an angle.
		return MathUtil.angleModulus((m_analogIn.getVoltage() - m_offset) / 5 * 2 * Math.PI * (m_reversed ? -1 : 1));
	}
}
