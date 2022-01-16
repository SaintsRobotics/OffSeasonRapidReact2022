package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder {
  private AnalogInput m_analogIn;

  private boolean m_reversed;
  private double m_offset;

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
   * Returns the angle as a {@link Rotation2d}.
   * 
   * @todo check if this is equal to the 0
   * @return The angle as a {@link Rotation2d}.
   */
  public Rotation2d getRotation2d() {
    if (m_reversed) {
      return new Rotation2d(5 - (m_analogIn.getVoltage() / 5 * 2 * Math.PI) - m_offset);
    } else {
      return new Rotation2d((m_analogIn.getVoltage() / 5 * 2 * Math.PI) - m_offset);
    }
  }
}
